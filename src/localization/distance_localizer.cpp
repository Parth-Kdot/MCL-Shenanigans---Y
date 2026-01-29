#include "localization/distance_localizer.h"

#include "config.h"
#include "localization/distance.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>

namespace distance_localization {
namespace {

constexpr double kMillimetersToMeters = 0.001;
constexpr double kMetersToInches = 39.37007874015748;
constexpr double kInvalidReadingThreshold = 9000.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = kPi / 2.0;
constexpr double kTwoPi = 2.0 * kPi;

struct DistanceSensorDefinition {
  SensorID id;
  Eigen::Vector3f offset;
  double scale;
  const char *label;
  DistanceSensorReading DistanceSensorSnapshot::*snapshotMember;
};

struct WallInfo {
  double coordinate;
  double normalAngle;
  bool isX;
  double sign;
};

struct AxisContribution {
  bool isX;
  double coordinateMeters;
  double weight;
};

struct AxisAccumulator {
  double weightedSum{0.0};
  double totalWeight{0.0};

  void add(const double value, const double weight) {
    weightedSum += value * weight;
    totalWeight += weight;
  }

  [[nodiscard]] std::optional<double> average() const {
    if (totalWeight <= 0.0) {
      return std::nullopt;
    }
    return weightedSum / totalWeight;
  }
};

const std::array<DistanceSensorDefinition, 4> kSensors{{
    {SensorID::LEFT, CONFIG::DISTANCE_LEFT_OFFSET, 0.987, "Left",
     &DistanceSensorSnapshot::left},
    {SensorID::FRONT, CONFIG::DISTANCE_FRONT_OFFSET, 0.986, "Front",
     &DistanceSensorSnapshot::front},
    {SensorID::RIGHT, CONFIG::DISTANCE_RIGHT_OFFSET, 0.980, "Right",
     &DistanceSensorSnapshot::right},
    {SensorID::BACK, CONFIG::DISTANCE_BACK_OFFSET, 0.979, "Back",
     &DistanceSensorSnapshot::back},
}};

const std::array<WallInfo, 4> kWalls{{
    {WALL_0_X, 0.0, true, 1.0},
    {WALL_1_Y, kHalfPi, false, 1.0},
    {WALL_2_X, kPi, true, -1.0},
    {WALL_3_Y, 3.0 * kHalfPi, false, -1.0},
}};

DistanceSensorSnapshot g_lastSnapshot{};
std::optional<lemlib::Pose> g_lastPose;

double wrapAngle(const double angle) { return std::remainder(angle, kTwoPi); }

std::optional<std::pair<const WallInfo *, double>>
selectWall(const double sensorHeading) {
  const double ninetyDeg = kHalfPi;
  std::optional<std::pair<const WallInfo *, double>> selection;
  double bestDiff = std::numeric_limits<double>::max();

  for (const auto &wall : kWalls) {
    const double diff = wrapAngle(wall.normalAngle - sensorHeading);
    const double absDiff = fabs(diff);
    if (absDiff >= ninetyDeg) {
      continue;
    }
    if (!selection.has_value() || absDiff < bestDiff) {
      bestDiff = absDiff;
      selection = std::make_pair(&wall, diff);
    }
  }

  return selection;
}

DistanceSensorReading sampleSingle(const DistanceSensorDefinition &definition) {
  DistanceSensorReading reading{};

  // Helper to get raw sensor for confidence/size checks
  pros::Distance *rawSensor = nullptr;
  switch (definition.id) {
  case SensorID::LEFT:
    rawSensor = &distLeft;
    break;
  case SensorID::FRONT:
    rawSensor = &distFront;
    break;
  case SensorID::RIGHT:
    rawSensor = &distRight;
    break;
  case SensorID::BACK:
    rawSensor = &distBack;
    break;
  }

  if (!rawSensor)
    return reading;

  reading.installed = rawSensor->is_installed();
  if (!reading.installed) {
    return reading;
  }

  // Use the corrected distance function that applies offsets
  reading.distance_mm = getDistance(definition.id);
  reading.confidence = rawSensor->get_confidence();
  reading.object_size = rawSensor->get_object_size();
  reading.object_velocity = rawSensor->get_object_velocity();
  return reading;
}

std::optional<AxisContribution>
buildContribution(const DistanceSensorDefinition &definition,
                  const DistanceSensorReading &reading,
                  const double headingRad) {
  if (!reading.installed || reading.distance_mm <= 0 ||
      reading.distance_mm >= kInvalidReadingThreshold) {
    return std::nullopt;
  }

  const double measurementMeters = definition.scale *
                                   static_cast<double>(reading.distance_mm) *
                                   kMillimetersToMeters;
  const double sensorHeading =
      headingRad + static_cast<double>(definition.offset.z());
  const auto selection = selectWall(sensorHeading);

  if (!selection.has_value()) {
    return std::nullopt;
  }

  const double cosTheta = cos(selection->second);
  if (fabs(cosTheta) < 1e-3) {
    return std::nullopt;
  }

  const double sensorCoord =
      selection->first->coordinate -
      selection->first->sign * measurementMeters * cosTheta;

  const double cosHeading = cos(headingRad);
  const double sinHeading = sin(headingRad);
  const double offsetX = static_cast<double>(definition.offset.x());
  const double offsetY = static_cast<double>(definition.offset.y());
  const double rotatedX = offsetX * cosHeading - offsetY * sinHeading;
  const double rotatedY = offsetX * sinHeading + offsetY * cosHeading;

  const double robotCoord = selection->first->isX ? (sensorCoord - rotatedX)
                                                  : (sensorCoord - rotatedY);

  const double weight =
      std::clamp(static_cast<double>(reading.confidence) / 63.0, 0.05, 1.0);

  return AxisContribution{selection->first->isX, robotCoord, weight};
}

} // namespace

DistanceSensorSnapshot sampleSensors() {
  DistanceSensorSnapshot snapshot{};
  for (const auto &definition : kSensors) {
    snapshot.*(definition.snapshotMember) = sampleSingle(definition);
  }
  g_lastSnapshot = snapshot;
  return snapshot;
}

bool isValidReading(const DistanceSensorReading &reading) {
  return reading.installed && reading.distance_mm > 0 &&
         reading.distance_mm < kInvalidReadingThreshold;
}

std::optional<lemlib::Pose> estimatePose(const DistanceSensorSnapshot &snapshot,
                                         const double headingDeg) {
  if (!std::isfinite(headingDeg)) {
    g_lastPose = std::nullopt;
    return std::nullopt;
  }

  const double headingRad = headingDeg * kPi / 180.0;
  AxisAccumulator xAccumulator;
  AxisAccumulator yAccumulator;

  for (const auto &definition : kSensors) {
    const auto &reading = snapshot.*(definition.snapshotMember);
    if (auto contribution = buildContribution(definition, reading, headingRad);
        contribution.has_value()) {
      if (contribution->isX) {
        xAccumulator.add(contribution->coordinateMeters, contribution->weight);
      } else {
        yAccumulator.add(contribution->coordinateMeters, contribution->weight);
      }
    }
  }

  const auto xMeters = xAccumulator.average();
  const auto yMeters = yAccumulator.average();

  if (!xMeters.has_value() || !yMeters.has_value()) {
    g_lastPose = std::nullopt;
    return std::nullopt;
  }

  // LemLib poses are expressed in inches, so convert the meter-based estimate.
  g_lastPose = lemlib::Pose(static_cast<float>(*xMeters * kMetersToInches),
                            static_cast<float>(*yMeters * kMetersToInches),
                            static_cast<float>(headingDeg));

  return g_lastPose;
}

const DistanceSensorSnapshot &getLastSnapshot() { return g_lastSnapshot; }

std::optional<lemlib::Pose> getLastPose() { return g_lastPose; }

bool applyLastPoseToChassis(lemlib::Chassis &chassis) {
  if (!g_lastPose.has_value()) {
    return false;
  }
  chassis.setPose(*g_lastPose);
  return true;
}

} // namespace distance_localization

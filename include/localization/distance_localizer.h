#pragma once

#include "lemlib/api.hpp"
#include "pros/distance.hpp"

#include <cstdint>
#include <optional>

namespace distance_localization {

struct DistanceSensorReading {
  double distance_mm{0.0};
  std::int32_t confidence{0};
  std::int32_t object_size{0};
  double object_velocity{0.0};
  bool installed{false};
};

struct DistanceSensorSnapshot {
  DistanceSensorReading left;
  DistanceSensorReading front;
  DistanceSensorReading right;
  DistanceSensorReading back;
};

DistanceSensorSnapshot sampleSensors();
bool isValidReading(const DistanceSensorReading &reading);
std::optional<lemlib::Pose> estimatePose(const DistanceSensorSnapshot &snapshot,
                                         double headingDeg);
const DistanceSensorSnapshot &getLastSnapshot();
std::optional<lemlib::Pose> getLastPose();
bool applyLastPoseToChassis(lemlib::Chassis &chassis);

} // namespace distance_localization

#pragma once

#include "sensor.h"
#include "config.h"
#include <cmath>
#include <algorithm>
#include <optional>

const std::vector<std::pair<Eigen::Vector2f, Eigen::Vector2f>> WALLS = {
	{{1.78308, 1.78308}, {1.78308, -1.78308}},
	{{1.78308, -1.78308}, {-1.78308, -1.78308}},
	{{-1.78308, -1.78308}, {-1.78308, 1.78308}},
	{{-1.78308, 1.78308}, {1.78308, 1.78308}},
};

inline constexpr float WALL_0_X = 1.78308F;
inline constexpr float WALL_1_Y = 1.78308F;
inline constexpr float WALL_2_X = -1.78308F;
inline constexpr float WALL_3_Y = -1.78308F;

class Distance : public Sensor {
private:
	Eigen::Vector3f sensorOffset;
	pros::Distance distance;

	QLength measured = 0.0;
	bool isMaxRange = false;
	QLength std = 0.0;

	double tuningConstant;

    // Pre-computed max measurement to treat as "infinity"
    const QLength MAX_VALID_RANGE = 2.5_m; 

public:
	Distance(Eigen::Vector3f sensor_offset, const double tuningConstant, pros::Distance distance)
		: sensorOffset(std::move(sensor_offset)), tuningConstant(tuningConstant),
		  distance(std::move(distance)) {
	}

	void update() override {
		const auto measuredMM = distance.get();

		isMaxRange = (measuredMM >= 9999);

        if (isMaxRange) {
            measured = MAX_VALID_RANGE + 0.5_m; // Treat as 3.0m (outside field effectively)
            std = 0.5_m; // High uncertainty but definitely "far"
        } else {
            measured = tuningConstant * measuredMM * millimetre;
            // Adaptive noise model: error grows with distance
		    std = 0.2 * measured / sqrt(std::max(1.0, (double)distance.get_confidence() / 64.0));
        }
	}

	[[nodiscard]] std::optional<double> p(const Eigen::Vector3f& X) override {
        // Optimisation: Pre-calculate trig
        float angle = X.z() + sensorOffset.z();
        float cos_a = std::cos(angle);
        float sin_a = std::sin(angle);

        // Vector composition manually to avoid Eigen overhead if critical, 
        // but keeping Eigen for readability as per prompt instruction to focus on algo correctness.
        // Actually, let's just do the offset rotation manually to be fast as requested.
        float offset_x = sensorOffset.x();
        float offset_y = sensorOffset.y();
        float x_sensor = X.x() + (offset_x * std::cos(X.z()) - offset_y * std::sin(X.z()));
        float y_sensor = X.y() + (offset_x * std::sin(X.z()) + offset_y * std::cos(X.z()));

		auto predicted = 100.0f; // Start with a large value (meters)

        // Wall 0 (+X): Defined by x = WALL_0_X. Ray intersects if cos_a > 0.
        // Dist = (WallX - SensorX) / cos_a
        if (cos_a > 1e-4f) {
            float d = (WALL_0_X - x_sensor) / cos_a;
            if (d >= 0) predicted = std::min(predicted, d);
        }

        // Wall 1 (+Y): Defined by y = WALL_1_Y. Ray intersects if sin_a > 0.
        // Dist = (WallY - SensorY) / sin_a
        if (sin_a > 1e-4f) {
            float d = (WALL_1_Y - y_sensor) / sin_a;
            if (d >= 0) predicted = std::min(predicted, d);
        }

        // Wall 2 (-X): Defined by x = WALL_2_X. Ray intersects if cos_a < 0.
        // Dist = (SensorX - WallX) / (-cos_a)
        if (cos_a < -1e-4f) {
            float d = (x_sensor - WALL_2_X) / (-cos_a);
            if (d >= 0) predicted = std::min(predicted, d);
        }

        // Wall 3 (-Y): Defined by y = WALL_3_Y. Ray intersects if sin_a < 0.
        // Dist = (SensorY - WallY) / (-sin_a)
        if (sin_a < -1e-4f) {
            float d = (y_sensor - WALL_3_Y) / (-sin_a);
            if (d >= 0) predicted = std::min(predicted, d);
        }

        // If 'isMaxRange', measured is ~3.0m.
        // If predicted is small (e.g. 0.5m), error is 2.5m -> low probability.
        // If predicted is large (e.g. >2m), error is small -> high probability.
        // This effectively punishes particles near walls when sensor sees nothing.
        
		return cheap_norm_pdf((predicted - measured.getValue())/std.getValue()) * LOCO_CONFIG::DISTANCE_WEIGHT;
	}

	~Distance() override = default;
};

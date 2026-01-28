#pragma once

class Sensor {
public:
	virtual std::optional<double> p(const Eigen::Vector3f& x) = 0;
	virtual void update() = 0;
	virtual ~Sensor() = default;
};

#pragma once
#include "api.h"

// =============================================================
//  SENSOR CONFIGURATION (Edit Ports Here)
// =============================================================

// "inline" allows us to define these in the .h file without errors
inline pros::Distance distFront(1);
inline pros::Distance distBack(14);
inline pros::Distance distLeft(3);
inline pros::Distance distRight(11);

// Enum to make names easy to read
enum SensorID {
    FRONT = 0,
    BACK = 1,
    LEFT = 2,
    RIGHT = 3
};

// Array to hold the offset values (Initialized to 0)
inline double sensorOffsets[4] = {0.0, 0.0, 0.0, 0.0};


// =============================================================
//  LOGIC FUNCTIONS
// =============================================================

// FUNCTION: Set the override offset
inline void setSensorOffset(int sensor_id, double offset_mm) {
    if(sensor_id < 0 || sensor_id > 3) return; // Safety check
    
    sensorOffsets[sensor_id] = offset_mm;
    
    // Using printf directly requires the terminal to be open
    printf(">> OVERRIDE: Sensor %d offset set to %f mm\n", sensor_id, offset_mm);
}

// FUNCTION: Get the corrected distance (Math applied automatically)
inline double getDistance(int sensor_id) {
    double raw_value = 0.0;

    // Select the correct sensor
    switch(sensor_id) {
        case FRONT: raw_value = distFront.get(); break;
        case BACK:  raw_value = distBack.get(); break;
        case LEFT:  raw_value = distLeft.get(); break;
        case RIGHT: raw_value = distRight.get(); break;
    }

    // Return the sensor value PLUS the offset
    return raw_value + sensorOffsets[sensor_id];
}
# Vex V5 Monte Carlo Localization (MCL)

![Version](https://img.shields.io/badge/version-2.0-blue) ![Model](https://img.shields.io/badge/model-Thrun_Probabilistic-green) ![ThreadSafe](https://img.shields.io/badge/safety-Mutex_Protected-orange)

A high-fidelity **Particle Filter** implementation for Vex V5 robotics, built on the principles of *Probabilistic Robotics* (Thrun et al.). This system fuses **LemLib Odometry** with **Distance Sensors** to provide an absolute position estimate that eliminates drift over time.

---

## 🚀 Features

*   **Thrun Odometry Motion Model:** accurately models the "banana-shaped" uncertainty distributions of skid-steer/tank-drive robots.
*   **Theta Filtering:** Unlike standard estimators, we filter heading ($\theta$) as part of the state vector, allowing wall geometry to correct gyroscope drift.
*   **Robust Sensor Model:** Handles "Max Range" (infinity) readings correctly by using them as evidence of *empty space*, preventing wall-hugging hallucinations.
*   **Thread-Safe Architecture:** Uses `pros::Mutex` and `std::atomic` to ensure safe data access between the background MCL task and your autonomous routines.
*   **Adaptive Resampling:** Uses Low-Variance Resampling to prevent particle deprivation.

---

## 🛠️ Quick Start

### 1. Installation
Copy the `localization` folder into your `src` and `include` directories. Ensure you have `Eigen` and `LemLib` installed.

### 2. Implementation (`src/main.cpp`)

```cpp
#include "localization/mcl_task.h"

void initialize() {
    chassis.calibrate();
    
    // Initialize MCL with chassis and IMU/Inertial sensor
    mcl::initializeMCL(chassis, inertial);
    
    // Optional: Use 3-sensor mode (Back, Left, Right) to save CPU
    mcl::g_mcl->useThreeSensorMode(true);
    
    // Start the background task
    mcl::g_mcl->start();
}

void autonomous() {
    // Set initial known position
    chassis.setPose(-50, 0, 90);
    mcl::g_mcl->initializeAtPose(-50, 0, 90);
    
    // Run path
    chassis.moveToPoint(0, 0, 2000);
    
    // ... later ...
    
    // Check if we can trust MCL
    if (mcl::g_mcl->isConverged()) {
        auto pose = mcl::g_mcl->getPose();
        printf("MCL Correction: x=%.2f, y=%.2f\n", pose.x(), pose.y());
    }
}
```

---

## ⚙️ Configuration

Tune the filter in `include/localization/mcl_task.h`:

```cpp
struct MCLConfig {
    static constexpr float LINEAR_NOISE = 0.05f;   // Drift per meter traveled
    static constexpr float ANGULAR_NOISE = 0.03f;  // Drift per radian turned
    static constexpr float CONVERGENCE_THRESHOLD = 0.1f;
};
```

---

## 📚 Theory Reference
This implementation adheres to:
*   **Motion:** *Probabilistic Robotics*, Table 5.6 (Sample_Motion_Model_Odometry)
*   **Sensor:** Likelihood Field Model with Max Range handling.
*   **Resampling:** Low Variance Sampler ($O(N)$ efficiency).

---

**Developed by Antigravity Integration Team**

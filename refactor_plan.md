# MCL Refactor Plan

## 1. Data Structures

### `include/localization/particleFilter.h`

*   **Struct Definition**: Introduce `struct Particle { float x; float y; float theta; float weight; };` inside `ParticleFilter` class.
*   **Storage**: Replace `std::array<std::array<float, 2>, L> particles` and `weights` with `std::array<Particle, L> particles`.
*   **Removal**: Remove `std::function<Angle()> angleFunction` member. Heading is now a filtered state, not an input injection.
*   **Method Update**:
    *   `update(const std::function<MotionDelta()>& motionModel)`: Accepts a generator for noisy motion control ($u_t$).
    *   `getPrediction()`: Computes circular mean for theta.
    *   `initNormal` / `initUniform`: Initialize theta as well.

## 2. Motion Model (Thrun Table 5.6)

### `src/localization/mcl_task.cpp`

*   **Logic**:
    *   Calculate odometry delta $(\delta x, \delta y, \delta \theta)$.
    *   Decompose into:
        *   $\delta_{rot1} = \text{atan2}(\delta y, \delta x) - \theta_{prev}$
        *   $\delta_{trans} = \sqrt{\delta x^2 + \delta y^2}$
        *   $\delta_{rot2} = \delta \theta - \delta_{rot1}$
    *   **Sampling**:
        *   For each particle $i$:
            *   Sample $\hat{\delta}_{rot1} \sim \mathcal{N}(\delta_{rot1}, \sigma_{rot1})$
            *   Sample $\hat{\delta}_{trans} \sim \mathcal{N}(\delta_{trans}, \sigma_{trans})$
            *   Sample $\hat{\delta}_{rot2} \sim \mathcal{N}(\delta_{rot2}, \sigma_{rot2})$
            *   $x' = x + \hat{\delta}_{trans} \cos(\theta + \hat{\delta}_{rot1})$
            *   $y' = y + \hat{\delta}_{trans} \sin(\theta + \hat{\delta}_{rot1})$
            *   $\theta' = \theta + \hat{\delta}_{rot1} + \hat{\delta}_{rot2}$
*   **Implementation**: Refactor `createPredictionFunction` to return a structure or lambda that performs this sampling per particle.

## 3. Sensor Model

### `include/localization/distance.h`

*   **Max Range Handling**:
    *   If `measuredMM >= 9999` (max range), return probability $P_{max}$ (e.g., 0.05). Do not return `std::nullopt`.
*   **Optimization**:
    *   Cache `cos(theta)` and `sin(theta)` or use `Eigen::Rotation2Df` efficiently.
    *   Refactor `p(X)` to avoid redundant trig calls if possible (though the ray casting depends on particle heading).

## 4. Concurrency & Performance

### `include/localization/mcl_task.h`

*   **Member**: Add `pros::Mutex m_stateMutex`.

### `src/localization/mcl_task.cpp`

*   **Function**: `getPose()`
    *   Take `std::lock_guard<pros::Mutex>` before reading `m_currentPose`.
*   **Function**: `updateLoop()`
    *   Take `std::lock_guard<pros::Mutex>` before writing `m_currentPose`.


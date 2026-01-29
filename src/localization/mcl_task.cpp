/**
 * @file mcl_task.cpp
 * @brief Implementation of the Monte Carlo Localization Task Runner
 *
 * This file implements the MCL background task that continuously runs the
 * particle filter to estimate robot position. See mcl_task.h for detailed
 * documentation and usage examples.
 *
 * ## Key Implementation Details
 *
 * 1. **Motion Model**: Uses LemLib odometry delta plus Gaussian noise
 * 2. **Sensor Model**: Leverages existing Distance class from distance.h
 * 3. **Resampling**: Handled by ParticleFilter class in particleFilter.h
 * 4. **Thread Safety**: Uses atomic flags for task coordination
 */

#include "localization/mcl_task.h"

// Standard library includes
#include <cmath>
#include <random>
#include <algorithm>

namespace mcl {

// ============================================================================
// GLOBAL INSTANCE
// ============================================================================

/// Global MCL instance pointer (initialized via initializeMCL())
MCLTask* g_mcl = nullptr;

// ============================================================================
// CONSTRUCTOR / DESTRUCTOR
// ============================================================================

MCLTask::MCLTask(lemlib::Chassis& chassis, pros::Imu& imu)
    : m_chassis(chassis)
    , m_imu(imu)
    , m_filter(nullptr)
    , m_leftSensor(nullptr)
    , m_rightSensor(nullptr)
    , m_backSensor(nullptr)
    , m_frontSensor(nullptr)
    , m_task(nullptr)
    , m_running(false)
    , m_currentPose{0, 0, 0}
    , m_currentVariance(999.0f)
    , m_updateCount(0)
    , m_lastOdomPose(0, 0, 0)
    , m_hasLastOdom(false)
    , m_threeSensorMode(false)
{
    // ========================================================================
    // CREATE PARTICLE FILTER
    // ========================================================================
    // The particle filter needs a function that returns the current heading.
    // We use a lambda that reads from the IMU.
    //
    // Why a lambda? The particle filter was designed to be independent of
    // specific sensor implementations. By passing a function, we can use
    // any heading source (IMU, gyro, encoders, etc.).

    m_filter = new ParticleFilter<CONFIG::NUM_PARTICLES>(
        [this]() -> Angle {
            // Get heading from IMU in degrees, convert to our Angle type
            // IMU returns 0-360 degrees, we want continuous heading
            double heading = m_imu.get_heading();

            // Handle IMU calibration state
            if (m_imu.is_calibrating() || !std::isfinite(heading)) {
                // Return last known heading if IMU is unavailable
                return m_currentPose.theta_rad * radian;
            }

            // Convert degrees to radians for internal use
            return heading * degree;
        }
    );

    // ========================================================================
    // LOG INITIALIZATION
    // ========================================================================
    printf("[MCL] Task constructed with %zu particles\n",
           static_cast<size_t>(CONFIG::NUM_PARTICLES));
}

MCLTask::~MCLTask() {
    // Stop the task if running
    stop();

    // Clean up allocated memory
    // Note: Sensors are added to particle filter which doesn't own them,
    // so we need to delete them here

    if (m_filter != nullptr) {
        delete m_filter;
        m_filter = nullptr;
    }

    if (m_leftSensor != nullptr) {
        delete m_leftSensor;
        m_leftSensor = nullptr;
    }

    if (m_rightSensor != nullptr) {
        delete m_rightSensor;
        m_rightSensor = nullptr;
    }

    if (m_backSensor != nullptr) {
        delete m_backSensor;
        m_backSensor = nullptr;
    }

    if (m_frontSensor != nullptr) {
        delete m_frontSensor;
        m_frontSensor = nullptr;
    }

    printf("[MCL] Task destroyed\n");
}

// ============================================================================
// LIFECYCLE METHODS
// ============================================================================

void MCLTask::start() {
    // Don't start if already running
    if (m_running.load()) {
        printf("[MCL] Already running, ignoring start()\n");
        return;
    }

    // ========================================================================
    // INITIALIZE SENSORS
    // ========================================================================
    // Create and register distance sensors with the particle filter.
    // This must be done before starting the update loop.

    initializeSensors();

    // ========================================================================
    // START BACKGROUND TASK
    // ========================================================================
    // PROS tasks are cooperative - they run until they yield or block.
    // Our task yields every UPDATE_INTERVAL_MS milliseconds.

    m_running.store(true);

    m_task = new pros::Task(
        [this]() {
            this->updateLoop();
        },
        "MCL Task"  // Task name (useful for debugging)
    );

    printf("[MCL] Started background task (3-sensor mode: %s)\n",
           m_threeSensorMode ? "enabled" : "disabled");
}

void MCLTask::stop() {
    if (!m_running.load()) {
        return;
    }

    // Signal the task to stop
    m_running.store(false);

    // Give the task time to notice and exit
    pros::delay(MCLConfig::UPDATE_INTERVAL_MS * 2);

    // Clean up task handle
    if (m_task != nullptr) {
        delete m_task;
        m_task = nullptr;
    }

    printf("[MCL] Stopped (updates: %lu)\n",
           static_cast<unsigned long>(m_updateCount));
}

bool MCLTask::isRunning() const {
    return m_running.load();
}

// ============================================================================
// INITIALIZATION METHODS
// ============================================================================

void MCLTask::initializeAtPose(float x, float y, float theta) {
    // ========================================================================
    // CONVERT UNITS
    // ========================================================================
    // External interface uses inches/degrees (LemLib convention)
    // Internal uses meters/radians (SI units, particle filter convention)

    constexpr float kInchesToMeters = 0.0254f;
    constexpr float kDegreesToRadians = M_PI / 180.0f;

    float x_meters = x * kInchesToMeters;
    float y_meters = y * kInchesToMeters;
    float theta_rad = theta * kDegreesToRadians;

    // ========================================================================
    // INITIALIZE PARTICLES
    // ========================================================================
    // Use the particle filter's initNormal() to place all particles near
    // the specified position with a small initial covariance.

    Eigen::Vector2f mean(x_meters, y_meters);

    // Small initial covariance - particles start clustered around the pose
    // The 0.05 value means particles spread within ~5cm of the initial position
    Eigen::Matrix2f covariance = Eigen::Matrix2f::Identity() * 0.01f;

    // flip=false because we're using absolute field coordinates
    m_filter->initNormal(mean, covariance, false);

    // ========================================================================
    // UPDATE INTERNAL STATE
    // ========================================================================

    m_currentPose = {x_meters, y_meters, theta_rad};
    m_currentVariance = 0.01f;  // Start with low variance (high confidence)

    // Reset odometry tracking
    m_lastOdomPose = m_chassis.getPose();
    m_hasLastOdom = true;

    printf("[MCL] Initialized at (%.2f, %.2f, %.1f) inches\n", x, y, theta);
}

void MCLTask::initializeUniform() {
    // ========================================================================
    // SPREAD PARTICLES ACROSS FIELD
    // ========================================================================
    // Field is 144" x 144" = ~3.66m x 3.66m
    // Use slightly smaller bounds to keep particles away from walls

    m_filter->initUniform(-70_in, -70_in, 70_in, 70_in);

    // Reset internal state
    m_currentPose = {0, 0, 0};
    m_currentVariance = 999.0f;  // High variance - not converged
    m_hasLastOdom = false;

    printf("[MCL] Initialized uniformly across field\n");
}

// ============================================================================
// POSITION QUERY METHODS
// ============================================================================

MCLPose MCLTask::getPose() const {
    return m_currentPose;
}

lemlib::Pose MCLTask::getLemLibPose() const {
    return m_currentPose.toLemLib();
}

float MCLTask::getVariance() const {
    return m_currentVariance;
}

bool MCLTask::isConverged() const {
    // Need minimum updates AND low variance
    return (m_updateCount >= MCLConfig::MIN_UPDATES_FOR_CONVERGENCE) &&
           (m_currentVariance < MCLConfig::CONVERGENCE_THRESHOLD);
}

MCLState MCLTask::getState() const {
    MCLState state;
    state.pose = m_currentPose;
    state.variance_meters = m_currentVariance;
    state.isConverged = isConverged();
    state.validSensorCount = m_threeSensorMode ? 3 : 4;  // Simplified
    state.isRunning = m_running.load();
    state.updateCount = m_updateCount;
    state.lastUpdateTime = pros::millis();
    return state;
}

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================

void MCLTask::useThreeSensorMode(bool enable) {
    m_threeSensorMode = enable;
    printf("[MCL] Three-sensor mode %s\n", enable ? "enabled" : "disabled");
}

bool MCLTask::isThreeSensorMode() const {
    return m_threeSensorMode;
}

// ============================================================================
// PRIVATE METHODS
// ============================================================================

void MCLTask::initializeSensors() {
    // ========================================================================
    // CREATE DISTANCE SENSOR OBJECTS
    // ========================================================================
    // Each Distance sensor needs:
    // 1. Offset from robot center (position and angle of sensor on robot)
    // 2. Tuning constant (calibration scale factor)
    // 3. Reference to the PROS Distance sensor object

    // The offsets are defined in config.h as Eigen::Vector3f:
    // (x_offset, y_offset, angle_offset) in inches/radians

    // LEFT SENSOR: Faces left (90 degrees from front)
    m_leftSensor = new Distance(
        CONFIG::DISTANCE_LEFT_OFFSET,  // (4.5", 1.5", 90deg) from config.h
        0.987,                          // Calibration scale (from testing)
        distLeft                        // Global sensor object from sensor.h
    );

    // RIGHT SENSOR: Faces right (-90 degrees from front)
    m_rightSensor = new Distance(
        CONFIG::DISTANCE_RIGHT_OFFSET,  // (4.5", -1.5", -90deg) from config.h
        0.980,                           // Calibration scale
        distRight                        // Global sensor object from sensor.h
    );

    // BACK SENSOR: Faces backward (180 degrees from front)
    m_backSensor = new Distance(
        CONFIG::DISTANCE_BACK_OFFSET,   // (-4.56", -4.25", 180deg) from config.h
        0.979,                           // Calibration scale
        distBack                         // Global sensor object from sensor.h
    );

    // ========================================================================
    // ADD SENSORS TO PARTICLE FILTER
    // ========================================================================
    // The particle filter uses these sensors to weight particles based on
    // how well their predicted sensor readings match actual readings.

    m_filter->addSensor(m_leftSensor);
    m_filter->addSensor(m_rightSensor);
    m_filter->addSensor(m_backSensor);

    // Only add front sensor if not in 3-sensor mode
    if (!m_threeSensorMode) {
        m_frontSensor = new Distance(
            CONFIG::DISTANCE_FRONT_OFFSET,  // (9.563", 5.938", 0deg) from config.h
            0.986,                           // Calibration scale
            distFront                        // Global sensor object from sensor.h
        );
        m_filter->addSensor(m_frontSensor);
        printf("[MCL] Sensors initialized: Left, Right, Back, Front (4 total)\n");
    } else {
        printf("[MCL] Sensors initialized: Left, Right, Back (3-sensor mode)\n");
    }
}

std::function<Eigen::Vector2f()> MCLTask::createPredictionFunction() {
    // ========================================================================
    // MOTION MODEL EXPLANATION
    // ========================================================================
    // The motion model predicts how particles move based on odometry.
    //
    // When the robot moves from pose A to pose B:
    // 1. We calculate the delta (change in x, y) from odometry
    // 2. We apply this delta to ALL particles
    // 3. We add Gaussian noise to account for uncertainty
    //
    // The noise prevents particles from becoming too clustered and allows
    // the filter to recover from odometry errors.

    // Get current odometry pose
    lemlib::Pose currentOdom = m_chassis.getPose();

    // Calculate delta from last pose
    float dx = 0.0f;
    float dy = 0.0f;

    if (m_hasLastOdom) {
        // Convert inches to meters for internal use
        constexpr float kInchesToMeters = 0.0254f;

        dx = (currentOdom.x - m_lastOdomPose.x) * kInchesToMeters;
        dy = (currentOdom.y - m_lastOdomPose.y) * kInchesToMeters;
    }

    // Store current as last for next iteration
    m_lastOdomPose = currentOdom;
    m_hasLastOdom = true;

    // ========================================================================
    // ADD NOISE TO MOTION PREDICTION
    // ========================================================================
    // We add Gaussian noise proportional to the movement amount.
    // This is a simplified "velocity model" for motion.

    // Static random number generator (thread-local for safety)
    static thread_local std::mt19937 rng(std::random_device{}());

    // Calculate movement magnitude
    float movement = std::sqrt(dx*dx + dy*dy);

    // Only add noise if there was significant movement
    if (movement > 0.001f) {  // > 1mm of movement
        // Noise standard deviation proportional to movement
        float noise_std = movement * MCLConfig::LINEAR_NOISE;
        std::normal_distribution<float> noise(0.0f, noise_std);

        dx += noise(rng);
        dy += noise(rng);
    }

    // Return a lambda that captures the computed delta
    // This is called once per particle, but they all get the same delta
    // (with the noise already applied above)
    return [dx, dy]() -> Eigen::Vector2f {
        return Eigen::Vector2f(dx, dy);
    };
}

float MCLTask::calculateVariance() {
    // ========================================================================
    // CALCULATE PARTICLE SPREAD
    // ========================================================================
    // Variance measures how spread out the particles are.
    // Low variance = particles agree on position = high confidence
    // High variance = particles disagree = low confidence

    // Get current prediction (mean of particles)
    Eigen::Vector3f prediction = m_filter->getPrediction();
    float mean_x = prediction.x();
    float mean_y = prediction.y();

    // Calculate sum of squared distances from mean
    float sum_sq_dist = 0.0f;

    for (size_t i = 0; i < CONFIG::NUM_PARTICLES; i++) {
        Eigen::Vector3f particle = m_filter->getParticle(i);
        float dx = particle.x() - mean_x;
        float dy = particle.y() - mean_y;
        sum_sq_dist += dx*dx + dy*dy;
    }

    // Variance is average squared distance
    return sum_sq_dist / static_cast<float>(CONFIG::NUM_PARTICLES);
}

void MCLTask::updateLoop() {
    // ========================================================================
    // MAIN MCL UPDATE LOOP
    // ========================================================================
    // This runs continuously in a background PROS task.
    // Each iteration:
    // 1. Create motion prediction function
    // 2. Call particle filter update (handles weighting & resampling)
    // 3. Extract position estimate
    // 4. Calculate variance (confidence)
    // 5. Sleep until next update

    printf("[MCL] Update loop started\n");

    while (m_running.load()) {
        // ====================================================================
        // STEP 1: CREATE MOTION MODEL
        // ====================================================================
        // This function tells the particle filter how to move particles
        // based on odometry changes.

        auto predictionFunction = createPredictionFunction();

        // ====================================================================
        // STEP 2: RUN PARTICLE FILTER UPDATE
        // ====================================================================
        // The update() method does several things:
        // 1. Applies motion prediction to all particles
        // 2. Reads sensor values and weights particles
        // 3. Performs resampling if enough distance has been traveled
        //
        // Note: The existing ParticleFilter class in particleFilter.h
        // handles all this internally!

        m_filter->update(predictionFunction);

        // ====================================================================
        // STEP 3: EXTRACT POSITION ESTIMATE
        // ====================================================================
        // getPrediction() returns the weighted mean of all particles.
        // This is our best estimate of robot position.

        Eigen::Vector3f prediction = m_filter->getPrediction();

        // Update current pose (already in meters/radians internally)
        m_currentPose.x_meters = prediction.x();
        m_currentPose.y_meters = prediction.y();
        m_currentPose.theta_rad = prediction.z();

        // ====================================================================
        // STEP 4: CALCULATE CONFIDENCE
        // ====================================================================
        // Variance tells us how spread out the particles are.
        // Lower = more confident in position estimate.

        m_currentVariance = calculateVariance();

        // ====================================================================
        // STEP 5: UPDATE STATISTICS
        // ====================================================================

        m_updateCount++;

        // Periodic logging (every 100 updates = every 2 seconds)
        if (m_updateCount % 100 == 0) {
            printf("[MCL] Update #%lu: (%.2f, %.2f) var=%.4f %s\n",
                   static_cast<unsigned long>(m_updateCount),
                   m_currentPose.x(),  // inches
                   m_currentPose.y(),  // inches
                   m_currentVariance,
                   isConverged() ? "CONVERGED" : "");
        }

        // ====================================================================
        // STEP 6: SLEEP UNTIL NEXT UPDATE
        // ====================================================================
        // pros::delay() yields to other tasks and sleeps for specified ms.
        // This controls the update rate of the MCL.

        pros::delay(MCLConfig::UPDATE_INTERVAL_MS);
    }

    printf("[MCL] Update loop exited\n");
}

// ============================================================================
// GLOBAL INITIALIZATION
// ============================================================================

void initializeMCL(lemlib::Chassis& chassis, pros::Imu& imu) {
    // Clean up existing instance if any
    if (g_mcl != nullptr) {
        delete g_mcl;
    }

    // Create new MCL instance
    g_mcl = new MCLTask(chassis, imu);

    printf("[MCL] Global instance initialized\n");
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

void blendMCLIntoOdometry(lemlib::Chassis& chassis, float blendFactor) {
    // ========================================================================
    // GRADUAL POSITION BLENDING
    // ========================================================================
    // Instead of sudden position jumps, this function smoothly blends
    // the MCL estimate into odometry over time.
    //
    // blendFactor controls how much MCL to use:
    // - 0.0 = use 100% odometry (no correction)
    // - 0.1 = use 10% MCL, 90% odometry (gentle correction)
    // - 1.0 = use 100% MCL (full override, not recommended)

    if (g_mcl == nullptr) {
        printf("[MCL] Warning: blendMCLIntoOdometry called but MCL not initialized\n");
        return;
    }

    // Clamp blend factor to valid range
    blendFactor = std::max(0.0f, std::min(1.0f, blendFactor));

    // Get both estimates
    MCLPose mclPose = g_mcl->getPose();
    lemlib::Pose odomPose = chassis.getPose();

    // Calculate blended position
    // newPos = odom + blendFactor * (mcl - odom)
    float newX = odomPose.x + blendFactor * (mclPose.x() - odomPose.x);
    float newY = odomPose.y + blendFactor * (mclPose.y() - odomPose.y);

    // Keep heading from IMU (most accurate source)
    // Don't blend heading - IMU is ground truth

    // Apply blended position
    chassis.setPose(newX, newY, odomPose.theta);

    printf("[MCL] Blended %.0f%% MCL -> odom: (%.1f, %.1f)\n",
           blendFactor * 100.0f, newX, newY);
}

} // namespace mcl

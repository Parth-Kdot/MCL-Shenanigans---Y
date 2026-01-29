/**
 * @file mcl_task.h
 * @brief Monte Carlo Localization (MCL) Task Runner for VEX V5 Robotics
 *
 * This module provides a background task that continuously runs a particle filter
 * to estimate the robot's position on the field. The MCL system operates independently
 * from LemLib's odometry and provides a READ-ONLY position estimate that autonomous
 * code can query at any time.
 *
 * ## How MCL Works (High-Level Overview)
 *
 * Monte Carlo Localization uses a set of "particles" - each particle represents a
 * hypothesis about where the robot might be on the field. The algorithm works in
 * three continuous steps:
 *
 * 1. **PREDICTION (Motion Model)**
 *    When the robot moves, we move all particles by the same amount (plus some noise).
 *    This spreads particles out to account for wheel slip and encoder errors.
 *
 * 2. **UPDATE (Sensor Model)**
 *    We use distance sensors to measure how far we are from walls. For each particle,
 *    we ask: "If the robot were at THIS position, what would the sensors read?"
 *    Particles whose predicted readings match actual readings get higher "weight".
 *
 * 3. **RESAMPLING**
 *    We duplicate high-weight particles and remove low-weight ones. Over time,
 *    particles cluster around the robot's true position.
 *
 * ## Architecture: MCL is READ-ONLY
 *
 * IMPORTANT: This MCL system does NOT automatically modify the chassis position.
 * It maintains its own independent estimate. Your autonomous code can:
 * - Query the MCL position via getPose()
 * - Compare MCL vs odometry to detect drift
 * - Optionally blend MCL into odometry (gradual correction)
 *
 * This prevents sudden "jumps" in robot position that could cause erratic movement.
 *
 * ## Usage Example
 *
 * ```cpp
 * // In initialize():
 * mcl::initializeMCL(chassis, imu);
 * mcl::g_mcl->useThreeSensorMode(true);  // Use back, left, right sensors only
 * mcl::g_mcl->start();
 *
 * // In skills():
 * chassis.setPose(-49.92, 15.12, 85.236);
 * mcl::g_mcl->initializeAtPose(-49.92, 15.12, 85.236);
 *
 * // Later, query MCL position:
 * auto mclPose = mcl::g_mcl->getPose();
 * printf("MCL: (%.1f, %.1f)\n", mclPose.x(), mclPose.y());
 * ```
 *
 * @author Claude (with guidance from robotics team)
 * @date 2026
 */

#pragma once

// ============================================================================
// INCLUDES
// ============================================================================

// NOTE: Include order matters! utils.h must come before distance.h because
// distance.h uses angleDifference() and cheap_norm_pdf() from utils.h

// Use relative path to get the main config.h (not localization/config.h)
// This is needed because we're in include/localization/ directory
#include "../utils/utils.h"               // Utility functions (angleDifference, cheap_norm_pdf)
#include "../config.h"                    // CONFIG namespace with parameters (NUM_PARTICLES, offsets)
#include "config.h"                       // LOCO_CONFIG namespace (sensor weights) - same dir
#include "particleFilter.h"               // Existing particle filter implementation
#include "distance.h"                     // Distance sensor class with wall model
#include "sensor.h"                       // Sensor base class and globals
#include "lemlib/api.hpp"                 // LemLib chassis for odometry reference
#include "pros/rtos.hpp"                  // PROS task system
#include "pros/imu.hpp"                   // IMU for heading
#include "Eigen/Eigen"                    // Eigen for vector math

#include <atomic>                         // Thread-safe flags
#include <mutex>                          // std::lock_guard
#include <optional>                       // Optional return values
#include <functional>                     // std::function for callbacks

namespace mcl {

// ============================================================================
// CONFIGURATION CONSTANTS
// ============================================================================
// These values control MCL behavior. Tune based on your robot's characteristics.

/**
 * @brief MCL configuration parameters
 *
 * Adjust these values to tune MCL performance:
 * - UPDATE_INTERVAL_MS: Lower = more responsive but more CPU usage
 * - CONVERGENCE_THRESHOLD: Lower = stricter convergence requirement
 */
struct MCLConfig {
    // --------------------------------------------------------------------
    // TIMING CONFIGURATION
    // --------------------------------------------------------------------

    /// How often the MCL task runs (milliseconds)
    /// 20ms = 50Hz update rate, good balance of responsiveness and CPU
    static constexpr uint32_t UPDATE_INTERVAL_MS = 20;

    // --------------------------------------------------------------------
    // MOTION MODEL NOISE
    // --------------------------------------------------------------------
    // These add randomness to particle movement to account for real-world
    // uncertainties like wheel slip, carpet friction variations, etc.

    /// Linear noise factor (unitless, multiplied by distance traveled)
    /// Higher = more particle spread during movement
    static constexpr float LINEAR_NOISE = 0.1f;

    /// Angular noise factor (unitless, multiplied by rotation amount)
    /// Higher = more heading uncertainty during turns
    static constexpr float ANGULAR_NOISE = 0.1f;

    // --------------------------------------------------------------------
    // CONVERGENCE DETECTION
    // --------------------------------------------------------------------
    // Used to determine if particles have "agreed" on a position

    /// Maximum particle spread (in meters) to consider "converged"
    /// Lower = stricter requirement for convergence
    static constexpr float CONVERGENCE_THRESHOLD = 0.1f;  // ~4 inches

    /// Minimum number of update cycles before checking convergence
    /// Prevents false convergence detection at startup
    static constexpr uint32_t MIN_UPDATES_FOR_CONVERGENCE = 10;
};

// ============================================================================
// MCL POSE STRUCTURE
// ============================================================================

/**
 * @brief Represents a 2D pose (position + heading) from MCL
 *
 * Uses meters internally (matching particle filter) but provides
 * conversion methods for inches (matching LemLib convention).
 */
struct MCLPose {
    float x_meters;      ///< X position in meters (field frame)
    float y_meters;      ///< Y position in meters (field frame)
    float theta_rad;     ///< Heading in radians (0 = facing +X, CCW positive)

    /// Get X position in inches (for LemLib compatibility)
    [[nodiscard]] float x() const { return x_meters * 39.3701f; }

    /// Get Y position in inches (for LemLib compatibility)
    [[nodiscard]] float y() const { return y_meters * 39.3701f; }

    /// Get heading in degrees (for LemLib compatibility)
    [[nodiscard]] float theta() const { return theta_rad * 180.0f / M_PI; }

    /// Convert to LemLib Pose format
    [[nodiscard]] lemlib::Pose toLemLib() const {
        return lemlib::Pose(x(), y(), theta());
    }
};

// ============================================================================
// MCL STATE STRUCTURE
// ============================================================================

/**
 * @brief Diagnostic information about MCL status
 *
 * Use this for debugging, display, or decision-making about MCL reliability.
 */
struct MCLState {
    // --------------------------------------------------------------------
    // POSITION ESTIMATE
    // --------------------------------------------------------------------
    MCLPose pose{0, 0, 0};           ///< Current best position estimate

    // --------------------------------------------------------------------
    // CONFIDENCE METRICS
    // --------------------------------------------------------------------
    float variance_meters;            ///< Particle spread (lower = more confident)
    bool isConverged;                 ///< True if particles have converged
    int validSensorCount;             ///< How many sensors gave good readings

    // --------------------------------------------------------------------
    // STATUS FLAGS
    // --------------------------------------------------------------------
    bool isRunning;                   ///< Is the MCL task active?
    uint32_t updateCount;             ///< Total number of MCL updates performed
    uint32_t lastUpdateTime;          ///< Timestamp of last update (ms)
};

// ============================================================================
// MCL TASK CLASS
// ============================================================================

/**
 * @brief Main MCL Task Runner
 *
 * This class manages the particle filter and runs it in a background task.
 * It provides a read-only interface for querying the robot's estimated position.
 *
 * ## Thread Safety
 * The MCL runs in a separate PROS task. Position queries are thread-safe
 * and can be called from any context (autonomous, opcontrol, or other tasks).
 *
 * ## Coordinate Systems
 * - MCL internally uses METERS (SI units, matching particle filter)
 * - External interface provides INCHES (matching LemLib convention)
 * - Heading uses RADIANS internally, DEGREES in external interface
 * - Field origin is at center, +X is "right", +Y is "forward"
 */
class MCLTask {
public:
    // ========================================================================
    // CONSTRUCTOR / DESTRUCTOR
    // ========================================================================

    /**
     * @brief Construct MCL task with references to robot systems
     *
     * @param chassis Reference to LemLib chassis (used for odometry delta)
     * @param imu Reference to IMU (used for heading)
     *
     * The MCL does NOT modify these systems - it only reads from them.
     */
    MCLTask(lemlib::Chassis& chassis, pros::Imu& imu);

    /**
     * @brief Destructor - stops the MCL task if running
     */
    ~MCLTask();

    // ========================================================================
    // LIFECYCLE METHODS
    // ========================================================================

    /**
     * @brief Start the MCL background task
     *
     * Call this in initialize() after chassis.calibrate().
     * The MCL will begin continuously updating its position estimate.
     *
     * Safe to call multiple times - subsequent calls are ignored if already running.
     */
    void start();

    /**
     * @brief Stop the MCL background task
     *
     * Stops the particle filter updates. The last position estimate is preserved
     * and can still be queried via getPose().
     */
    void stop();

    /**
     * @brief Check if MCL is currently running
     * @return true if the background task is active
     */
    [[nodiscard]] bool isRunning() const;

    // ========================================================================
    // INITIALIZATION METHODS
    // ========================================================================

    /**
     * @brief Initialize all particles at a known position
     *
     * Call this at the START of autonomous when you know exactly where the
     * robot is positioned. All particles will be placed at this position
     * with a small initial spread.
     *
     * @param x X position in INCHES (LemLib convention)
     * @param y Y position in INCHES (LemLib convention)
     * @param theta Heading in DEGREES (LemLib convention)
     *
     * Example:
     * ```cpp
     * chassis.setPose(-49.92, 15.12, 85.236);
     * mcl::g_mcl->initializeAtPose(-49.92, 15.12, 85.236);
     * ```
     */
    void initializeAtPose(float x, float y, float theta);

    /**
     * @brief Initialize particles uniformly across the entire field
     *
     * Use this if you don't know where the robot is. Particles will spread
     * across the field and converge as sensor data comes in.
     *
     * This takes longer to converge than initializeAtPose().
     */
    void initializeUniform();

    // ========================================================================
    // POSITION QUERY METHODS (READ-ONLY)
    // ========================================================================

    /**
     * @brief Get the current MCL position estimate
     *
     * This is the primary method to query MCL. Returns the weighted mean
     * of all particle positions.
     *
     * @return MCLPose with position in meters and heading in radians
     *         Use .x(), .y(), .theta() for inches/degrees
     *
     * Example:
     * ```cpp
     * auto pose = mcl::g_mcl->getPose();
     * printf("MCL: (%.1f, %.1f) @ %.1f deg\n", pose.x(), pose.y(), pose.theta());
     * ```
     */
    [[nodiscard]] MCLPose getPose() const;

    /**
     * @brief Get the current position as a LemLib Pose
     *
     * Convenience method that returns the MCL estimate in LemLib's format.
     * Equivalent to getPose().toLemLib().
     *
     * @return lemlib::Pose with position in inches and heading in degrees
     */
    [[nodiscard]] lemlib::Pose getLemLibPose() const;

    /**
     * @brief Get the particle variance (spread)
     *
     * Lower variance = particles are clustered = higher confidence.
     * Higher variance = particles are spread out = lower confidence.
     *
     * @return Variance in meters (typically 0.01 to 0.5)
     */
    [[nodiscard]] float getVariance() const;

    /**
     * @brief Check if particles have converged
     *
     * Returns true if particles are clustered tightly around a single position.
     * Use this to determine if MCL has "locked on" to the robot's position.
     *
     * @return true if variance < CONVERGENCE_THRESHOLD
     */
    [[nodiscard]] bool isConverged() const;

    /**
     * @brief Get full MCL state for diagnostics
     *
     * Returns all MCL state information including position, confidence,
     * and status flags. Useful for debugging or display.
     *
     * @return MCLState structure with all diagnostic info
     */
    [[nodiscard]] MCLState getState() const;

    // ========================================================================
    // SENSOR CONFIGURATION
    // ========================================================================

    /**
     * @brief Enable 3-sensor mode (back, left, right only)
     *
     * In 3-sensor mode, the front sensor is not used. This is useful when:
     * - Front sensor is often blocked by game elements
     * - You want to reduce computational load
     * - You want to rely on known-good sensors
     *
     * @param enable true to use only 3 sensors, false to use all 4
     *
     * Note: Must be called BEFORE start() to take effect.
     */
    void useThreeSensorMode(bool enable);

    /**
     * @brief Check if 3-sensor mode is enabled
     * @return true if using only back, left, right sensors
     */
    [[nodiscard]] bool isThreeSensorMode() const;

private:
    // ========================================================================
    // PRIVATE MEMBERS
    // ========================================================================

    // --------------------------------------------------------------------
    // EXTERNAL REFERENCES (read-only)
    // --------------------------------------------------------------------
    lemlib::Chassis& m_chassis;      ///< LemLib chassis for odometry delta
    pros::Imu& m_imu;                ///< IMU for heading

    // --------------------------------------------------------------------
    // PARTICLE FILTER
    // --------------------------------------------------------------------
    /// The actual particle filter with 250 particles
    /// Uses CONFIG::NUM_PARTICLES from config.h
    ParticleFilter<CONFIG::NUM_PARTICLES>* m_filter;

    // --------------------------------------------------------------------
    // DISTANCE SENSORS
    // --------------------------------------------------------------------
    /// Distance sensor objects (created in constructor)
    Distance* m_leftSensor;
    Distance* m_rightSensor;
    Distance* m_backSensor;
    Distance* m_frontSensor;  ///< Only used if not in 3-sensor mode

    // --------------------------------------------------------------------
    // TASK MANAGEMENT
    // --------------------------------------------------------------------
    pros::Task* m_task;              ///< Background task handle
    std::atomic<bool> m_running;     ///< Thread-safe running flag
    mutable pros::Mutex m_stateMutex; ///< Mutex for thread safety

    // --------------------------------------------------------------------
    // STATE TRACKING
    // --------------------------------------------------------------------
    MCLPose m_currentPose;           ///< Current best estimate
    float m_currentVariance;         ///< Current particle spread
    uint32_t m_updateCount;          ///< Number of updates performed

    // --------------------------------------------------------------------
    // ODOMETRY TRACKING (for motion model)
    // --------------------------------------------------------------------
    lemlib::Pose m_lastOdomPose;     ///< Last odometry reading
    bool m_hasLastOdom;              ///< True if we have a previous reading

    // --------------------------------------------------------------------
    // CONFIGURATION
    // --------------------------------------------------------------------
    bool m_threeSensorMode;          ///< Use only 3 sensors

    // ========================================================================
    // PRIVATE METHODS
    // ========================================================================

    /**
     * @brief Main update loop (runs in background task)
     *
     * This is the core MCL algorithm that runs continuously:
     * 1. Get odometry delta from chassis
     * 2. Apply motion model to particles
     * 3. Update sensor readings
     * 4. Particle filter handles weighting and resampling
     * 5. Extract position estimate
     */
    void updateLoop();

    /**
     * @brief Create the motion model prediction function
     *
     * Returns a function that provides the (dx, dy) movement delta
     * for each particle. Includes noise for uncertainty.
     *
     * @return Function that returns Eigen::Vector2f motion delta
     */
    std::function<void(Particle&)> createPredictionFunction();

    /**
     * @brief Calculate particle variance (spread)
     *
     * Computes the average distance of particles from the mean position.
     * Lower variance = higher confidence.
     *
     * @return Variance in meters
     */
    float calculateVariance();

    /**
     * @brief Initialize distance sensors and add to particle filter
     *
     * Creates Distance sensor objects with appropriate offsets and
     * calibration constants, then adds them to the particle filter.
     */
    void initializeSensors();
};

// ============================================================================
// GLOBAL INSTANCE
// ============================================================================

/**
 * @brief Global MCL instance
 *
 * Use this to access MCL from anywhere in your code:
 * ```cpp
 * auto pose = mcl::g_mcl->getPose();
 * ```
 *
 * Must be initialized via initializeMCL() before use.
 */
extern MCLTask* g_mcl;

/**
 * @brief Initialize the global MCL instance
 *
 * Call this in initialize() after chassis.calibrate():
 * ```cpp
 * mcl::initializeMCL(chassis, imu);
 * ```
 *
 * @param chassis Reference to LemLib chassis
 * @param imu Reference to IMU sensor
 *
 * @note Now also initializes the random seed.
 */
void initializeMCL(lemlib::Chassis& chassis, pros::Imu& imu);

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

/**
 * @brief Blend MCL position into odometry gradually
 *
 * Use this for smooth position correction without sudden jumps.
 * Call periodically (e.g., every 500ms) when robot is stationary.
 *
 * @param chassis LemLib chassis to update
 * @param blendFactor How much MCL to blend in (0.0 = none, 1.0 = full)
 *                    Recommended: 0.1 to 0.3
 *
 * Example:
 * ```cpp
 * // After stopping at a waypoint:
 * chassis.waitUntilDone();
 * if (mcl::g_mcl->isConverged()) {
 *     mcl::blendMCLIntoOdometry(chassis, 0.2);  // 20% correction
 * }
 * ```
 */
void blendMCLIntoOdometry(lemlib::Chassis& chassis, float blendFactor = 0.1f);

} // namespace mcl

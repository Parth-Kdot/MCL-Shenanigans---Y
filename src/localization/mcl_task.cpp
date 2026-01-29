/**
 * @file mcl_task.cpp
 * @brief Implementation of the Monte Carlo Localization Task Runner
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
    m_filter = new ParticleFilter<CONFIG::NUM_PARTICLES>();

    printf("[MCL] Task constructed with %zu particles\n",
           static_cast<size_t>(CONFIG::NUM_PARTICLES));
}

MCLTask::~MCLTask() {
    stop();

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
    if (m_running.load()) {
        printf("[MCL] Already running, ignoring start()\n");
        return;
    }

    initializeSensors();

    m_running.store(true);

    m_task = new pros::Task(
        [this]() {
            this->updateLoop();
        },
        "MCL Task"
    );

    printf("[MCL] Started background task\n");
}

void MCLTask::stop() {
    if (!m_running.load()) {
        return;
    }

    m_running.store(false);
    pros::delay(MCLConfig::UPDATE_INTERVAL_MS * 2);

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
    constexpr float kInchesToMeters = 0.0254f;
    constexpr float kDegreesToRadians = M_PI / 180.0f;

    float x_meters = x * kInchesToMeters;
    float y_meters = y * kInchesToMeters;
    float theta_rad = theta * kDegreesToRadians;

    Eigen::Vector3f mean(x_meters, y_meters, theta_rad);
    
    // Initial covariance
    // 5cm std dev for position, 5 degrees for angle
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Identity();
    covariance(0,0) = 0.0025f; // variance (0.05m)^2
    covariance(1,1) = 0.0025f;
    covariance(2,2) = (5.0f * kDegreesToRadians) * (5.0f * kDegreesToRadians);

    m_filter->initNormal(mean, covariance);

    {
        std::lock_guard<pros::Mutex> lock(m_stateMutex);
        m_currentPose = {x_meters, y_meters, theta_rad};
        m_currentVariance = 0.01f;
    }

    m_lastOdomPose = m_chassis.getPose();
    m_hasLastOdom = true;

    printf("[MCL] Initialized at (%.2f, %.2f, %.1f) inches\n", x, y, theta);
}

void MCLTask::initializeUniform() {
    m_filter->initUniform(-70_in, -70_in, 70_in, 70_in);

    {
        std::lock_guard<pros::Mutex> lock(m_stateMutex);
        m_currentPose = {0, 0, 0};
        m_currentVariance = 999.0f;
    }
    
    m_hasLastOdom = false;

    printf("[MCL] Initialized uniformly across field\n");
}

// ============================================================================
// POSITION QUERY METHODS
// ============================================================================

MCLPose MCLTask::getPose() const {
    std::lock_guard<pros::Mutex> lock(m_stateMutex);
    return m_currentPose;
}

lemlib::Pose MCLTask::getLemLibPose() const {
    return getPose().toLemLib();
}

float MCLTask::getVariance() const {
    std::lock_guard<pros::Mutex> lock(m_stateMutex);
    return m_currentVariance;
}

bool MCLTask::isConverged() const {
    // Note: getVariance locks mutex, so we are safe
    return (m_updateCount >= MCLConfig::MIN_UPDATES_FOR_CONVERGENCE) &&
           (getVariance() < MCLConfig::CONVERGENCE_THRESHOLD);
}

MCLState MCLTask::getState() const {
    MCLState state;
    {
        std::lock_guard<pros::Mutex> lock(m_stateMutex);
        state.pose = m_currentPose;
        state.variance_meters = m_currentVariance;
    }
    state.isConverged = isConverged();
    state.validSensorCount = m_threeSensorMode ? 3 : 4;
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
    m_leftSensor = new Distance(
        CONFIG::DISTANCE_LEFT_OFFSET, 0.987, distLeft
    );

    m_rightSensor = new Distance(
        CONFIG::DISTANCE_RIGHT_OFFSET, 0.980, distRight
    );

    m_backSensor = new Distance(
        CONFIG::DISTANCE_BACK_OFFSET, 0.979, distBack
    );

    m_filter->addSensor(m_leftSensor);
    m_filter->addSensor(m_rightSensor);
    m_filter->addSensor(m_backSensor);

    if (!m_threeSensorMode) {
        m_frontSensor = new Distance(
            CONFIG::DISTANCE_FRONT_OFFSET, 0.986, distFront
        );
        m_filter->addSensor(m_frontSensor);
    }
}

// Helper to normalize angle to [-PI, PI]
static float normalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle <= -M_PI) angle += 2 * M_PI;
    return angle;
}

static float angleDiff(float a, float b) {
    return normalizeAngle(a - b);
}

std::function<void(Particle&)> MCLTask::createPredictionFunction() {
    lemlib::Pose currentOdom = m_chassis.getPose();

    constexpr float kInchesToMeters = 0.0254f;
    constexpr float kDegreesToRadians = M_PI / 180.0f;

    float dx_odom = 0.0f;
    float dy_odom = 0.0f;
    float dtheta_odom = 0.0f;

    // Current state in SI
    float x_prime = currentOdom.x * kInchesToMeters;
    float y_prime = currentOdom.y * kInchesToMeters;
    float theta_prime = currentOdom.theta * kDegreesToRadians;

    if (m_hasLastOdom) {
        float x_prev = m_lastOdomPose.x * kInchesToMeters;
        float y_prev = m_lastOdomPose.y * kInchesToMeters;
        float theta_prev = m_lastOdomPose.theta * kDegreesToRadians;

        // Calculate Odometry components (Thrun Table 5.6)
        float delta_rot1 = angleDiff(std::atan2(y_prime - y_prev, x_prime - x_prev), theta_prev);
        float delta_trans = std::sqrt(std::pow(x_prime - x_prev, 2) + std::pow(y_prime - y_prev, 2));
        float delta_rot2 = angleDiff(theta_prime - theta_prev, delta_rot1);
        
        // Handle "no movement" case preventing atan2 instability or small noise accumulation
        if (delta_trans < 0.001f) {
            delta_rot1 = 0;
            delta_rot2 = angleDiff(theta_prime, theta_prev);
        }

        // We capture these values in the lambda
        dx_odom = delta_trans; // reusing vars for clarity
        dy_odom = delta_rot1;
        dtheta_odom = delta_rot2;
    }

    m_lastOdomPose = currentOdom;
    m_hasLastOdom = true;

    // Parameters for noise
    float alpha1 = MCLConfig::ANGULAR_NOISE; // Rotational error from rotation
    float alpha2 = MCLConfig::ANGULAR_NOISE; // Rotational error from translation
    float alpha3 = MCLConfig::LINEAR_NOISE;  // Translation error from translation
    float alpha4 = MCLConfig::LINEAR_NOISE;  // Translation error from rotation

    // Thrun's Odometry Model
    return [=](Particle& p) {
        // Thread-local RNG
        static thread_local std::mt19937 rng(std::random_device{}());
        std::normal_distribution<float> norm(0.0f, 1.0f);

        // Recover components
        float rot1 = dy_odom;
        float trans = dx_odom;
        float rot2 = dtheta_odom;

        // Sample noisy components
        // sample(b^2) -> normal(0, b) ?? No, sample(b^2) usually implies variance b^2
        // Thrun says: sample(b) generates from N(0, b) usually.
        // Let's assume proportional standard deviation.
        
        float sd_rot1 = alpha1 * std::abs(rot1) + alpha2 * trans;
        float sd_trans = alpha3 * trans + alpha4 * (std::abs(rot1) + std::abs(rot2));
        float sd_rot2 = alpha1 * std::abs(rot2) + alpha2 * trans;

        float hat_rot1 = rot1 - norm(rng) * sd_rot1;
        float hat_trans = trans - norm(rng) * sd_trans;
        float hat_rot2 = rot2 - norm(rng) * sd_rot2;

        p.x += hat_trans * std::cos(p.theta + hat_rot1);
        p.y += hat_trans * std::sin(p.theta + hat_rot1);
        p.theta += hat_rot1 + hat_rot2;
        
        // Normalization is handled in particle filter update loop, but good to be safe
    };
}

float MCLTask::calculateVariance() {
    Eigen::Vector3f prediction = m_filter->getPrediction(); // x, y, theta
    float mean_x = prediction.x();
    float mean_y = prediction.y();
    // Angular variance? 
    // Usually variance is distance based for MCL localization quality
    
    float sum_sq_dist = 0.0f;

    for (size_t i = 0; i < CONFIG::NUM_PARTICLES; i++) {
        Eigen::Vector3f particle = m_filter->getParticle(i);
        float dx = particle.x() - mean_x;
        float dy = particle.y() - mean_y;
        sum_sq_dist += dx*dx + dy*dy;
    }

    return sum_sq_dist / static_cast<float>(CONFIG::NUM_PARTICLES);
}

void MCLTask::updateLoop() {
    printf("[MCL] Update loop started\n");

    while (m_running.load()) {
        auto motionModel = createPredictionFunction();

        m_filter->update(motionModel);

        Eigen::Vector3f prediction = m_filter->getPrediction();

        {
            std::lock_guard<pros::Mutex> lock(m_stateMutex);
            m_currentPose.x_meters = prediction.x();
            m_currentPose.y_meters = prediction.y();
            m_currentPose.theta_rad = prediction.z();
            m_currentVariance = calculateVariance();
            if (m_currentVariance < 0.05f) m_currentVariance = 0.05f;
        }

        m_updateCount++;

        if (m_updateCount % 100 == 0) {
            std::lock_guard<pros::Mutex> lock(m_stateMutex);
            
            bool converged = (m_updateCount >= MCLConfig::MIN_UPDATES_FOR_CONVERGENCE) &&
                             (m_currentVariance < MCLConfig::CONVERGENCE_THRESHOLD);

            /* 
            printf("[MCL] Update #%lu: (%.2f, %.2f) var=%.4f %s\n",
                   static_cast<unsigned long>(m_updateCount),
                   m_currentPose.x(),
                   m_currentPose.y(),
                   m_currentVariance,
                   converged ? "CONVERGED" : "");
            */
        }

        pros::delay(MCLConfig::UPDATE_INTERVAL_MS);
    }

    printf("[MCL] Update loop exited\n");
}

void initializeMCL(lemlib::Chassis& chassis, pros::Imu& imu) {
    if (g_mcl != nullptr) {
        delete g_mcl;
    }
    g_mcl = new MCLTask(chassis, imu);
    printf("[MCL] Global instance initialized\n");
}

void blendMCLIntoOdometry(lemlib::Chassis& chassis, float blendFactor) {
    if (g_mcl == nullptr) {
        printf("[MCL] Warning: blendMCLIntoOdometry called but MCL not initialized\n");
        return;
    }

    blendFactor = std::max(0.0f, std::min(1.0f, blendFactor));

    MCLPose mclPose = g_mcl->getPose(); // Thread-safe
    lemlib::Pose odomPose = chassis.getPose();

    // Only blend X and Y.
    // Thrun suggests fusing theta is dangerous if they have different reference frames or offset issues.
    // But if we trust MCL more... 
    // For now, let's stick to X/Y as per audit focus on position logic.
    
    float newX = odomPose.x + blendFactor * (mclPose.x() - odomPose.x);
    float newY = odomPose.y + blendFactor * (mclPose.y() - odomPose.y);

    chassis.setPose(newX, newY, odomPose.theta);

    printf("[MCL] Blended %.0f%% MCL -> odom: (%.1f, %.1f)\n",
           blendFactor * 100.0f, newX, newY);
}

} // namespace mcl

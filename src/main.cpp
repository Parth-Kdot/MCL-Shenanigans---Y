#include "main.h"
#include "lemlib/api.hpp"
#include "localization/mcl_task.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "utils/intake.hpp"  // Kept to avoid breaking existing includes if any

// ============================================================================
// HARDWARE DEFINITIONS (Preserved from original setup)
// ============================================================================

// Motor Groups
pros::MotorGroup rightMotors({5, 6, -7}, pros::MotorGearset::blue);
pros::MotorGroup leftMotors({-8, -9, 4}, pros::MotorGearset::blue);
pros::MotorGroup bottom_intake({-1}, pros::MotorGearset::blue);
pros::MotorGroup top_intake({2}, pros::MotorGearset::blue);

// Intake Subsystem
Intake intake; 
// Stub for Intake::telOP to avoid link errors if utils/intake.cpp is compiled
void Intake::telOP(bool intake, bool scoreTop, bool scoreMid, bool outtake, bool prime) {}

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Pneumatics
pros::adi::DigitalOut piston1('A');
pros::adi::DigitalOut piston2('B');
pros::adi::DigitalOut piston3('C');

// Globals needed by other files (if any)
bool pRon = true;
bool pYon = false;
bool pL2on = false;
bool pR_prev = false;
bool pY_prev = false;
bool pL2_prev = false;

// Inertial Sensor
pros::Imu imu(10);

// Tracking Wheels
pros::Rotation verticalEnc(20);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 2.5);

// Drivetrain Settings
lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    12, // track width
    lemlib::Omniwheel::NEW_325, 
    450,
    2 // horizontal drift
);

// PIDs
lemlib::ControllerSettings lateral_controller(10, 0, 5, 0, 0, 0, 0, 0, 20);
lemlib::ControllerSettings angular_controller(4, 0, 18, 0, 0, 0, 0, 0, 0);

// Sensors
lemlib::OdomSensors sensors(
    &vertical,
    nullptr,
    nullptr,
    nullptr,
    &imu
);

// Input Curves
lemlib::ExpoDriveCurve throttleCurve(10, 0, 1.55);
lemlib::ExpoDriveCurve steerCurve(10, 0, 2.10);

// Chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttleCurve, &steerCurve);


// ============================================================================
// LOGGING TASK (The Source of Truth)
// ============================================================================

/**
 * @brief High-freq CSV logger for Telemetry analysis
 * 
 * Columns:
 * 1. TIME: Milliseconds since startup
 * 2. ODOM_X: LemLib's belief (Reference)
 * 3. ODOM_Y: LemLib's belief
 * 4. MCL_X: Particle Filter Mean (The Test Subject)
 * 5. MCL_Y: Particle Filter Mean
 * 6. SIGMA: Particle Variance in meters (Convergence metric)
 * 7. STATUS: SAFE (<0.1m var) or LOST
 */
void mclLoggingTask() {
    // Header for CSV
    printf("TIME, ODOM_X, ODOM_Y, MCL_X, MCL_Y, SIGMA, STATUS\n");

    while (true) {
        if (mcl::g_mcl == nullptr) {
            pros::delay(100);
            continue;
        }

        // 1. Fetch Data
        lemlib::Pose odom = chassis.getPose();
        mcl::MCLPose mclPose = mcl::g_mcl->getPose();
        float sigma = mcl::g_mcl->getVariance();
        bool converged = mcl::g_mcl->isConverged();

        // 2. CSV Output
        printf("DATA, %d, %.2f, %.2f, %.2f, %.2f, %.3f, %s\n",
               pros::millis(),
               odom.x, odom.y,         // Control Group (Inches)
               mclPose.x(), mclPose.y(), // Variable (Inches)
               sigma,                   // Confidence (Meters)
               converged ? "SAFE" : "LOST");
        
        pros::delay(100); // 10Hz logging
    }
}

// ============================================================================
// SYSTEM LIFECYCLE
// ============================================================================

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "MCL CALIBRATING...");
    
    // 1. Hardware Calibration
    chassis.calibrate();

    // 2. Initialize MCL
    // This creates the particle filter logic but doesn't assume location yet
    mcl::initializeMCL(chassis, imu);
    
    // Optimization: Use 3-sensor mode
    mcl::g_mcl->useThreeSensorMode(true);
    
    // 3. Start Background Computation
    mcl::g_mcl->start();

    // 4. Set Initial Guess (Optional, assuming starting at 0,0,0 for testing)
    // Uncomment actual start position for match
    // mcl::g_mcl->initializeAtPose(0, 0, 0); 
    mcl::g_mcl->initializeUniform(); // Use uniform for "Lost Robot" test

    pros::lcd::set_text(1, "READY - LOGGING ACTIVE");
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol() {
    // 1. Start Logging Task immediately
    pros::Task logger(mclLoggingTask, "TelemetryLogger");

    while (true) {
        // Simple Arcade Drive for Testing
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        chassis.arcade(leftY, rightX);
        
        // Control Pistons (Manual overrides for testing)
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            piston1.set_value(!piston1.get_value());
        }

        pros::delay(20);
    }
}
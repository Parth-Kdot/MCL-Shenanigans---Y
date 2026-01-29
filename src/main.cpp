#include "main.h"
#include "lemlib/api.hpp"
#include "localization/mcl_task.h"
#include "pros/misc.h"
#include "pros/motors.h"

// ============================================================================
// HARDWARE DEFINITIONS
// ============================================================================

// Motor Groups
pros::MotorGroup rightMotors({5, 6, -7}, pros::MotorGearset::blue);
pros::MotorGroup leftMotors({-8, -9, 4}, pros::MotorGearset::blue);
pros::MotorGroup bottom_intake({-1}, pros::MotorGearset::blue);
pros::MotorGroup top_intake({2}, pros::MotorGearset::blue);

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Pneumatics
pros::adi::DigitalOut piston1('A');
pros::adi::DigitalOut piston2('B');
pros::adi::DigitalOut piston3('C');

bool pRon = false; // Toggle state for piston1

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
// LOGGING & DISPLAY TASKS
// ============================================================================

/**
 * @brief High-freq CSV logger for Telemetry analysis
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

/**
 * @brief Dashboard for the VEX Controller
 */
void controllerTask() {
    while (true) {
        if (mcl::g_mcl != nullptr) {
            mcl::MCLPose p = mcl::g_mcl->getPose();
            bool conv = mcl::g_mcl->isConverged();
            
            // Line 0: MCL Position
            controller.print(0, 0, "MCL: %.1f %.1f", p.x(), p.y());
            pros::delay(55); // Controller update limit
            
            // Line 1: Status + Variance
            controller.print(1, 0, "%s V:%.2f", conv ? "LOCKED" : "SEARCH", mcl::g_mcl->getVariance());
            pros::delay(55);
        } else {
            controller.print(0, 0, "MCL: INITIALIZING");
            pros::delay(100);
        }
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
    mcl::initializeMCL(chassis, imu);
    mcl::g_mcl->useThreeSensorMode(true);
    mcl::g_mcl->start();
    
    // 3. Set Initial State (Uniform for testing)
    mcl::g_mcl->initializeUniform(); 

    // 4. Start Persistent Tasks (Heap allocated to survive initialize)
    new pros::Task(mclLoggingTask, "TelemetryLogger");
    new pros::Task(controllerTask, "ControllerDash");

    pros::lcd::set_text(1, "READY - LOGGING ACTIVE");
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol() {
    // Logic tasks are already running from initialize()!
    // Just handle driving here.

    while (true) {
        // Simple Arcade Drive
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        chassis.arcade(leftY, rightX);
        
        // Manual Piston Test (A Button)
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            pRon = !pRon;
            piston1.set_value(pRon);
        }

        pros::delay(20);
    }
}
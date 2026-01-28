#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motors.h"

// Define the sensors here so we can use them in both setup AND printing
pros::Distance distLeft(11);
pros::Distance distFront(12);
pros::Distance distRight(13);
pros::Distance distBack(14);

bool pRon = false;
bool pYon = false;
bool pL2on = false;

bool pR_prev = false;
bool pY_prev = false;
bool pL2_prev = false;


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup rightMotors({5, 6, -7}, pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup leftMotors({-8, -9, 4}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

pros::Motor bottom_intake(-1, pros::MotorGearset::blue); // intake motors on ports 7 and 20

pros::Motor top_intake(2, pros::MotorGearset::blue);

// Define pneumatics
pros::adi::DigitalOut piston1('A'); // piston on port A, score
pros::adi::DigitalOut piston2('B'); // piston on port B, match loader
pros::adi::DigitalOut piston3('C'); // piston on port C, doinker

// Inertial Sensor on port 10
pros::Imu imu(10);

// tracking wheels
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(20);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, 
                              450, 
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              5, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(4, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              18, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(10, // joystick deadband out of 127
                                     0, // minimum output where drivetrain will move out of 127
                                     1.55 // expo curve gain
);

// input curve for steer input during driver controlz
lemlib::ExpoDriveCurve steerCurve(10, // joystick deadband out of 127
                                  0, // minimum outputslew where drivetrain will move out of 127
                                  2.10 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(4, "L: %d mm", distLeft.get());
            pros::lcd::print(5, "F: %d mm", distFront.get());
            pros::lcd::print(6, "R: %d mm", distRight.get());
            pros::lcd::print(7, "B: %d mm", distBack.get());
            controller.print(0, 0, "X: %f", pros::c::motor_get_temperature(1)); // x            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());

            // delay to save resources
            pros::delay(100);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {
    // KADEN: put auton selector code here
}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    //set position to x:0, y:0, heading:0
    chassis.setPose(0, 0, 0);
    //set for high goal
    piston2.set_value(true);
    piston3.set_value(true);
    // turn to face heading 90 with a very long timeout
    bottom_intake.move_velocity(600);
    chassis.moveToPose(8.174, 34.154, 16.36, 2000, {.maxSpeed = 300});
    // move to match loader ready position
    chassis.turnToHeading(120, 2000, {.maxSpeed = 300});
    chassis.moveToPoint(31.34,-2, 2000, {.maxSpeed = 300});
    chassis.moveToPoint(31.34, 1.286, 2000, {.maxSpeed = 300});
    chassis.turnToHeading(180, 2000, {.maxSpeed = 300});
    piston1.set_value(true);
    pros::delay(500);
    chassis.moveToPose(31.34, -35, 179.0, 2500, {.maxSpeed = 300});
    chassis.moveToPose(31.834, 22.904, 178.05, 2000, {.forwards = false, .maxSpeed = 200});
    top_intake.move_velocity(600);   
}

/**
 * Runs in driver control
 */
void opcontrol() {
    pRon = true;
    pYon = true;
    pL2on = true;
    pR_prev = true;
    pY_prev = true;
    pL2_prev = true;
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		bool intakeForwardButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
		bool intakeBackwardButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool flapButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
		bool pR = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);	// score
		bool pY = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT); // match loader
        bool pL2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2); // doinker

        // move the robot
        chassis.curvature(leftY, leftX);

        // control the intake motors
        if (intakeForwardButton) {
            bottom_intake.move_velocity(600);
        } else if (intakeBackwardButton) {
            bottom_intake.move_velocity(600);
            top_intake.move_velocity(600);
        } else if (flapButton) {
            top_intake.move_velocity(-600);
            bottom_intake.move_velocity(-600);
        } else {
            bottom_intake.move_velocity(0);
            top_intake.move_velocity(0);
        }

		// control the pistons
		// X button toggle
		if (pR && !pR_prev) {           // just pressed this loop
			pRon = !pRon;               // flip state
			piston1.set_value(pRon);   // set piston to state
		}

		// B button toggle
		if (pY && !pY_prev) {           // just pressed this loop
			pYon = !pYon;
			piston2.set_value(pYon);
		}

        if (pL2 && !pL2_prev) {           // just pressed this loop
            pL2on = !pL2on;
            piston3.set_value(pL2on);
        }

		// remember button state for next loop
		pR_prev = pR;
		pY_prev = pY;
        pL2_prev = pL2;

        // delay to save resources
        pros::delay(10);
    }
}

#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
// #include "localization/distance_localizer.h"
//#include "localization/sensor.h"
#include "pros/misc.h"
#include "pros/motors.h"
//#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "utils/intake.hpp"

//#include <limits>

bool pRon = true;
bool pYon = false;
bool pL2on = false;

bool pR_prev = false;
bool pY_prev = false;
bool pL2_prev = false;

extern pros::MotorGroup bottom_intake;
extern pros::MotorGroup top_intake;
extern pros::adi::DigitalOut piston1;
extern pros::adi::DigitalOut piston2;
extern pros::adi::DigitalOut piston3;
extern Intake intake;

void set_score_piston_state(bool state) {
  pRon = state;
  piston1.set_value(state);
}

void set_matchload_piston_state(bool state) {
  pYon = state;
  piston2.set_value(state);
}

void set_doinker_piston_state(bool state) {
  pL2on = state;
  piston3.set_value(state);
}

using PistonSetter = void (*)(bool);

static void update_piston_toggle(bool buttonPressed, bool &previousButtonState,
                                 bool &pistonState, PistonSetter setter) {
  if (buttonPressed && !previousButtonState) {
    setter(!pistonState);
  }
  previousButtonState = buttonPressed;
}

void control_score_piston(bool buttonPressed) {
  update_piston_toggle(buttonPressed, pR_prev, pRon, set_score_piston_state);
}

void control_matchload_piston(bool buttonPressed) {
  update_piston_toggle(buttonPressed, pY_prev, pYon,
                       set_matchload_piston_state);
}

void control_doinker_piston(bool buttonPressed) {
  update_piston_toggle(buttonPressed, pL2_prev, pL2on,
                       set_doinker_piston_state);
}

// motor groups
pros::MotorGroup
    rightMotors({5, 6, -7},
                pros::MotorGearset::blue); // left motor group - ports 3
                                           // (reversed), 4, 5 (reversed)
pros::MotorGroup leftMotors(
    {-8, -9, 4},
    pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

pros::MotorGroup bottom_intake({-1}, pros::MotorGearset::blue);

pros::MotorGroup top_intake({2}, pros::MotorGearset::blue);

// intake subsystem
Intake intake;

void Intake::telOP(bool intake, bool scoreTop, bool scoreMid, bool outtake,
                   bool prime) {
  pros::MotorGroup &bottomMotor = bottom_intake;
  pros::MotorGroup &topMotor = top_intake;

  if (outtake) {
    topMotor.move_velocity(-600);
    bottomMotor.move_velocity(-600);
  } else if (intake) {
    bottomMotor.move_velocity(600);
    topMotor.move_velocity(0);
  } else if (scoreMid) {
    topMotor.move_velocity(-600);
    bottomMotor.move_velocity(-600);
    pros::delay(500);
    bottomMotor.move_velocity(600);
    topMotor.move_velocity(600);
  } else if (scoreTop) {
    topMotor.move_velocity(-600);
    bottomMotor.move_velocity(-600);
    pros::delay(500);
    bottomMotor.move_velocity(600);
    topMotor.move_velocity(600);
  } else {
    bottomMotor.move_velocity(0);
    topMotor.move_velocity(0);
  }

  set_score_piston_state(!scoreMid);
}

void middle_goal_score(bool state) {
  if (state) {
    bottom_intake.move_velocity(600);
    top_intake.move_velocity(600);
  } else {
    bottom_intake.move_velocity(0);
    top_intake.move_velocity(0);
  }
  set_score_piston_state(state);
}

void matchload_activate(bool state) { set_matchload_piston_state(state); }

void long_goal_score(bool state) {
  if (state) {
    bottom_intake.move_velocity(600);
    top_intake.move_velocity(600);
  } else {
    bottom_intake.move_velocity(0);
    top_intake.move_velocity(0);
  }
  set_score_piston_state(state);
}

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Define pneumatics
pros::adi::DigitalOut piston1('A'); // piston on port A, score
pros::adi::DigitalOut piston2('B'); // piston on port B, match loader
pros::adi::DigitalOut piston3('C'); // piston on port C, doinker

// Inertial Sensor on port 10
pros::Imu imu(10);

// tracking wheels
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-20);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot
// (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(
    &leftMotors,  // left motor group
    &rightMotors, // right motor group
    12,           // 10 inch track width
    lemlib::Omniwheel::NEW_325, 450,
    2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

lemlib::ControllerSettings
    lateral_controller(10, // proportional gain (kP)
                       0,  // integral gain (kI)
                       5,  // derivative gain (kD)
                       0,  // anti windup
                       0,  // small error range, in inches
                       0,  // small error range timeout, in milliseconds
                       0,  // large error range, in inches
                       0,  // large error range timeout, in milliseconds
                       .5  // maximum acceleration (slew)
    );

lemlib::ControllerSettings
    angular_controller(4,  // proportional gain (kP)
                       0,  // integral gain (kI)
                       18, // derivative gain (kD)
                       0,  // anti windup
                       0,  // small error range, in inches
                       0,  // small error range timeout, in milliseconds
                       0,  // large error range, in inches
                       0,  // large error range timeout, in milliseconds
                       0   // maximum acceleration (slew)
    );

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr,   // vertical tracking wheel 2, set to
                                       // nullptr as we don't have a second one
                            nullptr,   // horizontal tracking wheel
                            nullptr,   // horizontal tracking wheel 2, set to
                                       // nullptr as we don't have a second one
                            &imu       // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);


// create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors, &throttle_curve, &steer_curve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();      // initialize brain screen
  set_score_piston_state(true); // ensure score piston starts engaged
  chassis.calibrate();          // calibrate sensors
  // the default rate is 50. however, if you need to change the rate, you
  // can do the following.
  // lemlib::bufferedStdout().setRate(...);
  // If you use bluetooth or a wired connection, you will want to have a rate of
  // 10ms

  // for more information on how the formatting for the loggers
  // works, refer to the fmtlib docs

  // thread to for brain screen and position logging
   pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            controller.print(0, 0, "Intake Temp: %f", pros::c::motor_get_temperature(1)); // x            // log position telemetry
            //controller.print(1, 0, "Auton: %d", autonselected);
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
 * This is an example autonomous routine which demonstrates a lot of the
 * features LemLib has to offer
 */


void right_auton() {
// 1. Setup Initial State
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.setPose(0, 0, 0);

    // 2. Set Piston States
    set_doinker_piston_state(true);

    // 3. Start Intake and Move to First Position
    // We access the global motor group directly
    intake.telOP(true, false, false, false, false);

    // Move to x: 10.174, y: 34.154, theta: 20.36
    chassis.moveToPose(10.174, 34.154, 20.36, 2000, {.maxSpeed = 300});
    chassis.waitUntilDone();

    // 4. Move to Match Load Ready Position
    chassis.turnToHeading(120, 1300, {.maxSpeed = 300});
    chassis.waitUntilDone();

    chassis.moveToPoint(41.5556, 11, 2000, {.maxSpeed = 50});
    chassis.waitUntilDone();

    chassis.turnToHeading(180, 1500, {.maxSpeed = 300});
    chassis.waitUntilDone();

    // open match loader
    set_matchload_piston_state(true);
    pros::delay(500); // wait for piston to actuate

    // 6. Alignment / Interaction Phase
    chassis.moveToPoint(41.5556, -5, 3200, {.maxSpeed = 30, .minSpeed = 35});
    chassis.waitUntilDone();
    //pros::delay(1750); // wait for any oscillations to settle

    // Manual slow push logic
    // Your provided code defines 'leftMotors' and 'rightMotors' as global MotorGroups.
    // We use .move_velocity() directly on them, removing the 'drivetrain->' pointer syntax.
    leftMotors.move_velocity(10);
    rightMotors.move_velocity(10);
    pros::delay(2550);
    
    // Stop manual push
    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);

    // 7. Back away and Final Intake
    // Move backwards (forwards = false)
    chassis.moveToPose(42, 23, 180, 2000, {.forwards = false, .minSpeed = 70});
    chassis.waitUntilDone();

    intake.telOP(false, true, false, false, false);
    set_matchload_piston_state(false);
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
       int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
       bool intakeForwardButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
       bool intakeBackwardButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
       bool flapButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
       bool pR = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1); // score
       bool pY = controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT); // match loader
       bool pL2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2); // doinker


       // move the robot
       chassis.arcade(leftY, leftX, false, 0.7);


       // control the intake motors
       if (intakeForwardButton) {
           bottom_intake.move_velocity(590);
       } else if (intakeBackwardButton) {
           bottom_intake.move_velocity(590);
           top_intake.move_velocity(590);
       } else if (flapButton) {
           top_intake.move_velocity(-590);
           bottom_intake.move_velocity(-590);
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

// =============================================================
//  SKILLS AUTONOMOUS ROUTINE
// =============================================================
void skills() {
// 1. Setup Initial State
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.setPose(0, 0, 0);

    // 2. Set Piston States
    set_doinker_piston_state(true);

    // 3. Start Intake and Move to First Position
    // We access the global motor group directly
    intake.telOP(true, false, false, false, false);

    // Move to x: 10.174, y: 34.154, theta: 20.36
    chassis.moveToPose(10.174, 34.154, 20.36, 2000, {.maxSpeed = 300});
    chassis.waitUntilDone();

    // 4. Move to Match Load Ready Position
    chassis.turnToHeading(120, 2000, {.maxSpeed = 300});
    chassis.waitUntilDone();

    chassis.moveToPoint(40.2556, 7, 2000, {.maxSpeed = 50});
    chassis.waitUntilDone();

    chassis.turnToHeading(180, 2000, {.maxSpeed = 300});
    chassis.waitUntilDone();

    chassis.moveToPose(42, 23, 180, 2000, {.forwards = false, .maxSpeed = 100});
    chassis.waitUntilDone();

    intake.telOP(false, true, false, false, false);

    pros::delay(1500);

    intake.telOP(true, false, false, false, false);

    // open match loader
    set_matchload_piston_state(true);
    pros::delay(500); // wait for piston to actuate

    // 6. Alignment / Interaction Phase
    chassis.moveToPoint(40.2556, -7.11, 3500, {.maxSpeed = 85, .minSpeed = 80});
    chassis.waitUntilDone();
    //pros::delay(1750); // wait for any oscillations to settle

    // Manual slow push logic
    // Your provided code defines 'leftMotors' and 'rightMotors' as global MotorGroups.
    // We use .move_velocity() directly on them, removing the 'drivetrain->' pointer syntax.
    leftMotors.move_velocity(10);
    rightMotors.move_velocity(10);
    pros::delay(2750);
    
    // Stop manual push
    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);

    // 7. Back away and Final Intake
    // Move backwards (forwards = false)
    chassis.moveToPose(42, 23, 180, 2000, {.forwards = false, .maxSpeed = 100});
    chassis.waitUntilDone();

    intake.telOP(false, true, false, false, false);

set_matchload_piston_state(false);

pros::delay(2500);
	chassis.moveToPoint(42, 13, 1199); //node 31
	pros::delay(50);
	chassis.turnToHeading(230, 906);
    chassis.moveToPoint(28,-12.5, 2006); //node 32
	chassis.turnToHeading(250, 906);
    chassis.moveToPoint(19, -17.5, 1364); //node 33
	pros::delay(50);
    chassis.turnToHeading(260, 834);
    chassis.moveToPoint(16, -21.5, 1364); //node 33
	chassis.turnToHeading(270, 834);
    pros::delay(300);
    chassis.moveToPoint(12, -21.5, 2300); //node 34
	set_matchload_piston_state(true);
    pros::delay(500); // wait for piston to actuate
	chassis.moveToPoint(-11, -21.5, 1364, {.minSpeed = 450}); //node 33
    pros::delay(1500);
    set_matchload_piston_state(false);







//   intake.telOP(true, false, false, false, false);
//   // chassis.setPose(0, 0, 0);
//   chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

//   chassis.setPose(-49.920000, 15.120000, 90.000000);

// //   chassis.turnToHeading(90.0, 503);
// //   chassis.moveToPoint(-36.24, 15.12, 1245);
// //   pros::delay(50);
// //   chassis.turnToHeading(52.815294, 805);
// //   chassis.moveToPoint(-22.32, 25.68, 1379);
// //   pros::delay(50);
// //   chassis.moveToPoint(-22.32, 22.68, 400);
// //   chassis.turnToHeading(319.397752, 1192);
// //   chassis.moveToPoint(-4, -0.5, 1478, {.forwards = false});
// //   set_score_piston_state(false);
// //   pros::delay(2000);
// //   intake.telOP(false, false, true, false, false);

// //   pros::delay(1000);
//   chassis.moveToPoint(-45.64, 45, 2141);
//  // chassis.waitUntil(11.144602);
//  // middle_goal_score(false);
//   chassis.waitUntilDone();
//   pros::delay(50);
//   chassis.turnToHeading(270, 969);
//   chassis.moveToPoint(-45.6, 44, 1012);
// //   chassis.waitUntil(0.481858);

//   matchload_activate(true);
//   chassis.waitUntilDone();
//   intake.telOP(true, false, false, false, false);
//   pros::delay(300);
//   chassis.moveToPose(-58.5, 44, 270, 3420, {.maxSpeed = 50}, false); // node 6

//   pros::delay(2000);
//   chassis.moveToPoint(-45.12, 46.32, 1436, {.forwards = false}); // node 7
//   chassis.turnToHeading(55, 146);
//   chassis.waitUntil(14.629714);
//   matchload_activate(false);

//   intake.telOP(true, false, false, false, false);
//   chassis.waitUntilDone();
//   pros::delay(50);
//   chassis.turnToHeading(70, 400);
//   chassis.moveToPoint(-33.64, 64.25, 700); // node 8
  
//   pros::delay(50);
//   chassis.turnToHeading(90, 991);
//   chassis.moveToPoint(-19.922253, 62.75, 1373); // node 9
//   chassis.moveToPoint(42, 59.75, 2000, {.maxSpeed=100}); //node 13
//   chassis.turnToHeading(180, 750);
//   chassis.moveToPoint(42.0, 42, 1573); // node 14
//   chassis.turnToHeading(90, 750);

//   chassis.moveToPoint(20, 40, 1750, {.forwards = false, .maxSpeed=100}, false); //node 15
//   pros::delay(50);
//   long_goal_score(true);
//   pros::delay(2500);
//   matchload_activate(true);
//   chassis.moveToPoint(65, 40, 2000, {.maxSpeed=100}); //node 13



// //   chassis.moveToPoint(-19.922253, 62.630685, 1373);
// //   pros::delay(50);
// //   chassis.moveToPoint(-3.843288, 63.143162, 1332);
// //   pros::delay(50);
// //   chassis.moveToPoint(10.100794, 63.587595, 1255);
// //   pros::delay(50);
// //   chassis.moveToPoint(23.04, 62.0, 1217);
// //   pros::delay(50);
// //   chassis.turnToHeading(91.487868, 465);
// //   chassis.moveToPoint(41.52, 70.52, 1413); // node 13
// //   chassis.turnToHeading(80, 500);
// //   chassis.moveToPoint(-31.64, 61.25, 1413); // node 13
// //   chassis.turnToHeading(90.0000, 500);
// //   chassis.moveToPoint(51, 63.25, 1500, {.maxSpeed = 127}); // node 13
// //   chassis.moveToPoint(51, 63.25, 2000, {.maxSpeed = 80}); // node 13
// //   pros::delay(500);
// //   chassis.turnToHeading(180, 700);
// //   chassis.moveToPoint(51.0, 44, 2000); // node 14
// //   pros::delay(50);
// //   chassis.turnToHeading(90, 800);

// //   chassis.moveToPoint(24.5, 44, 2000, {.forwards = false}, false); // node 15
// //   long_goal_score(true);
// //   pros::delay(2500);
// //  // chassis.waitUntil(11.940754);

// // //   // initial match, top left score

// //   chassis.turnToHeading(90, 461);       // prev89
// //   chassis.moveToPoint(51.62, 44, 2000); // prey56 //node 16?? ///current 48
// //   //chassis.waitUntil(16.507699);
// //   long_goal_score(false);
// //   // top left matchload
// //   matchload_activate(true);
// //   intake.telOP(true, false, false, false, false);
// //   chassis.waitUntilDone();
// //   pros::delay(500);
// //   chassis.turnToHeading(90, 461);
// //   chassis.moveToPoint(67.12, 44, 2800, {.maxSpeed = 50}); // prev 5 //node 17
// //   // pros::delay(2000);
// //   chassis.turnToHeading(90.0, 461);
// //   set_score_piston_state(true);
// //   chassis.moveToPoint(24.5, 44, 2000, {.forwards = false},
// //                       false); // node 18 //scoring on long goal
// //   chassis.waitUntilDone();
// //   // chassis.waitUntil(32.81013);
// //   //__________above no change is good
// //   // top left 2nd matchload score
// //   matchload_activate(false);

// //   //pros::delay(2500);
// //   long_goal_score(true);
// //   pros::delay(1900);
// //   chassis.moveToPoint(47.32, 44, 1000); // node 19

// //   chassis.turnToHeading(180, 800);
}

void left_auton() {

    intake.telOP(true, false, false, false, false);
  // chassis.setPose(0, 0, 0);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

  chassis.setPose(-49.920000, 15.120000, 90.000000);

  chassis.turnToHeading(90.0, 503);
  chassis.moveToPoint(-36.24, 15.12, 1145);
  pros::delay(50);
  chassis.turnToHeading(52.815294, 605);
  chassis.moveToPoint(-22.32, 25.68, 1179);
  pros::delay(50);
  chassis.moveToPoint(-22.32, 22.68, 400);
  chassis.turnToHeading(319.397752, 992);
  chassis.moveToPoint(-4, -0.5, 1178, {.forwards = false});
  set_score_piston_state(false);
  pros::delay(1500);
  intake.telOP(false, false, true, false, false);


//   pros::delay(1000);
  chassis.moveToPoint(-45.64, 45, 1741);
 // chassis.waitUntil(11.144602);
 middle_goal_score(false);
  pros::delay(50);
  chassis.turnToHeading(270, 969);
  chassis.moveToPoint(-45.6, 44, 512);
//   chassis.waitUntil(0.481858);

  matchload_activate(true);
  chassis.waitUntilDone();
  intake.telOP(true, false, false, false, false);
  pros::delay(300);
  chassis.moveToPose(-58.5, 44, 270, 1720, {.maxSpeed = 50}, false); // node 6


  //pros::delay(2000);
  chassis.moveToPoint(-25, 46.32, 1836, {.forwards = false}); // node 7
  matchload_activate(false);
  intake.telOP(false, true, false, false, false);

//   pros::delay(2000);
//   tongue.set_value(false);
//   chassis.moveToPoint(-36, 17, 1000, {.minSpeed = 60}, false);
//   chassis.moveToPoint(-36, 40, 1000, {.forwards=false,.minSpeed = 200}, false);
  
}


void AWP_auton() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.setPose(-47.285, -16.636, 180);


    // Move in front of match loader   
    chassis.moveToPoint(-44.000, -50.113, 2500, { .maxSpeed = 300});

    chassis.turnToHeading(270.0, 1000);  

    // Open match loader
    set_matchload_piston_state(true);
    pros::delay(100);

    intake.telOP(true, false, false, false, false);

    // Alignment / interaction phase with matchloader 
    chassis.moveToPoint(-56.695, -54.350,2500,{ .maxSpeed = 80, .minSpeed = 55 });
    // chassis.waitUntilDone();
    
    pros::delay(1650);


    // Move backwards to score long goal
    chassis.moveToPoint(-29.484, -54.113,2200,{ .forwards = false, .maxSpeed = 50 });
    //chassis.waitUntilDone();

    pros::delay(800);

    intake.telOP(false, true, false, false, false);
    pros::delay(1700);

    intake.telOP(false, false, false, false, false);

    set_matchload_piston_state(false);

    chassis.moveToPoint(-37.845, -47.113,1300,{ .maxSpeed = 80, .minSpeed = 60 });
    //chassis.waitUntilDone();



    // Move towards middle balls
    intake.telOP(true, false, false, false, false);

    chassis.turnToHeading(20, 800, { .maxSpeed = 300 });
    //chassis.waitUntilDone();
    //pros::delay(150);
    //chassis.moveToPoint(-24.36, -24.36, 1800, { .maxSpeed = 100, .minSpeed = 30 });
    //chassis.waitUntilDone();
    pros::delay(150);
    chassis.moveToPoint(-24.899, 22.36, 3200, { .maxSpeed = 100, .minSpeed = 30 });
    //chassis.waitUntilDone();
    pros::delay(150);
    chassis.turnToHeading(315, 2100, { .maxSpeed = 90 });
    //chassis.waitUntilDone();
    pros::delay(150);
    // Score middle goal
    chassis.moveToPoint(-13.953, 8.299, 2200, { .forwards = false, .maxSpeed = 20 }, false);
    //chassis.waitUntilDone();
  set_score_piston_state(false);
  pros::delay(100);
  intake.telOP(false, false, true, false, false);
  pros::delay(1300);

  intake.telOP(false, false, false, false, false);

    // Move towards LEFT match loaders
    chassis.moveToPoint(
        -48.364, 44.0,2700,{ .maxSpeed = 100, .minSpeed = 20 });
        pros::delay(100);
    //chassis.waitUntilDone();
    chassis.turnToHeading(270, 800, {.maxSpeed = 300});
    //chassis.waitUntilDone();
    // Open match loader
    set_matchload_piston_state(true);
    

    intake.telOP(true, false, false, false, false);

    pros::delay(300);

    // Alignment / interaction phase with left matchloaders 
    chassis.moveToPoint(
        -61.452, 45.813,
        3000,
        { .maxSpeed = 80, .minSpeed = 15 }
    );
    pros::delay(1200);
    //chassis.waitUntilDone();

    // Move backwards to score long goal
    chassis.moveToPoint(
        -29.484, 45.516,
        1720,
        { .forwards = false, .maxSpeed = 80, .minSpeed = 40 }
    );
    //chassis.waitUntilDone();

    pros::delay(600);
    intake.telOP(false, true, false, false, false);
    pros::delay(2500);
}


void autonomous() {
  AWP_auton();
}
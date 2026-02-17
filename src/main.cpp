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

// Auton selector state: true = right_auton, false = left_auton
bool use_right_auton = true;
bool auton_btn_prev = false;

extern pros::MotorGroup bottom_intake;
extern pros::MotorGroup top_intake;
extern pros::adi::DigitalOut piston1;
extern pros::adi::DigitalOut piston2;
extern pros::adi::DigitalOut piston3;
extern pros::adi::DigitalIn auton_selector;
extern Intake intake;

pros::Distance distFront(14);   // Front distance sensor
pros::Distance distBack(12);   // Back distance sensor


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

void resetToDistance(int targetDist, bool useFront, int speed = 50) {
    pros::delay(50);  // Allow sensor to stabilize
    
    pros::Distance& sensor = useFront ? distFront : distBack;
    int currentDist = sensor.get_distance();
    
    if (currentDist > 3000) {
        pros::lcd::print(4, "Sensor error! Dist=%dmm", currentDist);
        return;
    }
    
    pros::lcd::print(4, "%s Target=%dmm Curr=%dmm", useFront ? "FRONT" : "BACK", targetDist, currentDist);
    
    const int MIN_SPEED = 10;  // Minimum speed to prevent stalling
    const int SLOW_RANGE = 100;  // Start slowing down within 100mm of target
    
    if (useFront) {
        if (currentDist > targetDist) {
            while (sensor.get_distance() > targetDist) {
                int dist = sensor.get_distance();
                int error = dist - targetDist;
                int moveSpeed = (error < SLOW_RANGE) ? (MIN_SPEED + (speed - MIN_SPEED) * error / SLOW_RANGE) : speed;
                leftMotors.move_velocity(moveSpeed);
                rightMotors.move_velocity(moveSpeed);
                pros::delay(10);
            }
        } else if (currentDist < targetDist) {
            while (sensor.get_distance() < targetDist) {
                int dist = sensor.get_distance();
                int error = targetDist - dist;
                int moveSpeed = (error < SLOW_RANGE) ? (MIN_SPEED + (speed - MIN_SPEED) * error / SLOW_RANGE) : speed;
                leftMotors.move_velocity(-moveSpeed);
                rightMotors.move_velocity(-moveSpeed);
                pros::delay(10);
            }
        }
    } else {
        if (currentDist > targetDist) {
            while (sensor.get_distance() > targetDist) {
                int dist = sensor.get_distance();
                int error = dist - targetDist;
                int moveSpeed = (error < SLOW_RANGE) ? (MIN_SPEED + (speed - MIN_SPEED) * error / SLOW_RANGE) : speed;
                leftMotors.move_velocity(-moveSpeed);
                rightMotors.move_velocity(-moveSpeed);
                pros::delay(10);
            }
        } else if (currentDist < targetDist) {
            while (sensor.get_distance() < targetDist) {
                int dist = sensor.get_distance();
                int error = targetDist - dist;
                int moveSpeed = (error < SLOW_RANGE) ? (MIN_SPEED + (speed - MIN_SPEED) * error / SLOW_RANGE) : speed;
                leftMotors.move_velocity(moveSpeed);
                rightMotors.move_velocity(moveSpeed);
                pros::delay(10);
            }
        }
    }
  }
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
    set_score_piston_state(false);
  } else if (scoreMid) {
    topMotor.move_velocity(-600);
    bottomMotor.move_velocity(-600);
    pros::delay(150);
    bottomMotor.move_velocity(600);
    topMotor.move_velocity(600);
  } else if (scoreTop) {
    set_score_piston_state(false);
    topMotor.move_velocity(-600);
    bottomMotor.move_velocity(-600);
    pros::delay(150);
    bottomMotor.move_velocity(600);
    topMotor.move_velocity(600);
    set_score_piston_state(false);
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
pros::adi::DigitalIn auton_selector('D');

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
    8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

lemlib::ControllerSettings
    lateral_controller(13, // proportional gain (kP)
                       0,  // integral gain (kI)
                       95,  // derivative gain (kD)
                       0,  // anti windup
                       1,  // small error range, in inches
                       100,  // small error range timeout, in milliseconds
                       3,  // large error range, in inches
                       500,  // large error range timeout, in milliseconds
                       1  // maximum acceleration (slew)
    );

lemlib::ControllerSettings
    angular_controller(4,  // proportional gain (kP)
                       0,  // integral gain (kI)
                       18, // derivative gain (kD)
                       0,  // anti windup
                       1,  // small error range, in inches
                       100,  // small error range timeout, in milliseconds
                       3,  // large error range, in inches
                       500,  // large error range timeout, in milliseconds
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
  set_doinker_piston_state(true);
  //set_matchload_piston_state(true);
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
  while (true) {
    bool auton_btn = auton_selector.get_value();
    // Toggle on rising edge (button press)
    if (auton_btn && !auton_btn_prev) {
      use_right_auton = !use_right_auton;
    }
    auton_btn_prev = auton_btn;

    // Show current selection on brain screen
    pros::lcd::print(5, "Auton: %s", use_right_auton ? "RIGHT" : "LEFT");

    pros::delay(20);
  }
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

  set_score_piston_state(false);
    // 3. Start Intake and Move to First Position
    // We access the global motor group directly
    intake.telOP(true, false, false, false, false);
  set_score_piston_state(false);
    // Move to x: 10.174, y: 34.154, theta: 20.36
    chassis.moveToPose(10.174, 34.154, 20.36, 2000);
//    chassis.waitUntil(12.5);
pros::delay(1000);
    set_matchload_piston_state(true);

    // 4. Move to Match Load Ready Position
    chassis.turnToHeading(120, 3300);

    pros::delay(400);
    set_matchload_piston_state(false);

//    chassis.waitUntilDone();

    chassis.moveToPoint(40, 10, 4000);
//    chassis.waitUntilDone();
    pros::delay(1000);
    set_matchload_piston_state(true);

    chassis.turnToHeading(180, 1000);
//    chassis.waitUntilDone();
chassis.moveToPoint(40,-11.5, 3000);
pros::delay(100);
      // 6. Alignment / Interaction Phase
//      chassis.arcade(85, 0);
  //    pros::delay(1000);
    //  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
      //chassis.arcade(0,0);
      //pros::delay(1000); // wait for any oscillations to settle

    // 7. Back away and Final Intake
    // Move backwards (forwards = false)
    chassis.moveToPoint(39, 24, 3000, {.forwards = false, .maxSpeed = 100});
//    chassis.waitUntilDone();
pros::delay(1000);
    set_score_piston_state(false);      
    intake.telOP(false, true, false, false, false);
    set_score_piston_state(false);
    set_matchload_piston_state(false);
    set_doinker_piston_state(true);
    pros::delay(1750);

    //8. Doinker everything in.
    chassis.moveToPoint(30.4, 11.3, 3000, {.minSpeed=30});
    chassis.turnToHeading(-180, 2000);
    chassis.moveToPoint(30.4, 33.8, 2000, {.forwards=false, .minSpeed=30});
    chassis.moveToPoint(30.4, 42.4, 2000, {.forwards=false});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

}



/**
* Runs in driver control
*/
void opcontrol() {
   chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
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

       if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
          printf("X: %f, Y: %f, Theta: %f\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
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

        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
// Set Initial Position
    chassis.setPose(-46.477, 8.166, 90);

//Run intake + matchload_piston

    chassis.turnToHeading(60.0, 900);  
    

    intake.telOP(true, false, false, false, false);
//    pros::delay(1500);

    // Get Center Balls
    chassis.moveToPoint(-21.757, 23.709, 1500, { .maxSpeed = 80}); 
       pros::delay(900);

//    set_matchload_piston_state(true);

// Stop intake + raise Matchload_piston

//    set_matchload_piston_state(false);

    chassis.turnToHeading(320.0, 900);  
    

//Score Center Goal

    chassis.moveToPoint(-11.186, 8.733, 2500, { .forwards = false, .maxSpeed = 50 });
    pros::delay(900);

    intake.telOP(false, false, true, false, false);
    pros::delay(1300);

    intake.telOP(false, false, false, false, false);

//Line up X with matchloader
    chassis.moveToPoint(-41.004, 46.526, 2000, {.maxSpeed = 50 });
    

    chassis.turnToHeading(270.0, 900);  
    

//Run intake + matchload_piston
    intake.telOP(true, false, false, false, false);

 set_matchload_piston_state(true);

    pros::delay(50);

//Matchloader interaction phase #1
 chassis.moveToPoint(-58.517,46.477, 1000, {.maxSpeed = 50 });    
pros::delay(2000);


// Stop intake + raise Matchload_piston
    intake.telOP(false, false, false, false, false);
    pros::delay(1500);

    set_matchload_piston_state(false);

//Move Back
    chassis.moveToPoint(-50.636,46.477, 1000, {.forwards = false,.minSpeed = 15});   
    chassis.turnToHeading(34.8, 900);  
    
//Motion Chaining
// ADD 4 POINTS

    chassis.moveToPose(-46.492,52.325, 56.1, 1500, {.minSpeed = 60 });
    
    chassis.moveToPose(-40.238,56.613, 74.5, 1500, {.minSpeed = 60 });

    chassis.moveToPose(-31.758,56.946, 90, 1500, {.minSpeed = 60});

    chassis.moveToPose(-22.333,57.895, 90, 1500, {.minSpeed = 60});
    
    chassis.moveToPoint(45.689,58.189, 4500, {.minSpeed = 60});

//Make all 4 Wall Reset impact Y

// First Wall Reset
resetToDistance(648, false);

        chassis.turnToHeading(90.0, 900);  

    chassis.moveToPoint(45.689,46.477, 1200, {.minSpeed = 60});

// Run intake
    intake.telOP(true, false, false, false, false);

   set_matchload_piston_state(true);
    pros::delay(50);


//Matchloader interaction phase #2
    chassis.moveToPoint(61.013,46.477, 1200, {.maxSpeed = 60});
    
pros::delay(2000);

//Score balls into left long goal

   chassis.moveToPoint(27.737,46.696, 2500, {.forwards=false, .maxSpeed = 60,});
   set_matchload_piston_state(false);
    pros::delay(750);
   intake.telOP(false, true, false, false, false);
    pros::delay(3000);
   intake.telOP(false, false, false, false, false);

    chassis.moveToPoint(38.245,46.696, 1200, {.maxSpeed = 60});
   
   chassis.turnToHeading(140, 900);  
    

//Line up with blue parking (make a curve so it can intake all balls!!!) 

    chassis.moveToPose(41.718,40.543, 130, 1800, {.minSpeed = 60});
    chassis.moveToPose(51.428,33.26,140, 1800, {.minSpeed = 60});


    

   chassis.turnToHeading(180, 900);  
    

//Cross Over Blue Parking 
   intake.telOP(true, false, false, false, false);
    chassis.moveToPoint(61.013,-25.548, 4000);
    


// Second Wall Reset
resetToDistance(1180, true);


   chassis.turnToHeading(270, 900);  
    

//Run Intake
   intake.telOP(true, false, false, false, false);
    pros::delay(1500);

//Get Center Balls
    chassis.moveToPoint(23.724,-22.203, 1500, {.maxSpeed = 80});

//    set_matchload_piston_state(true);
    
//    pros::delay(50);
//    set_matchload_piston_state(false);

// Score on Center Goal

    chassis.turnToHeading(310, 900 );
    chassis.moveToPoint(11.143,-11.143, 1500, {.forwards = false, .maxSpeed = 60});
    pros::delay(900);

   intake.telOP(false, false, true, false, false);
    pros::delay(1500);

   intake.telOP(false, false, false, false, false);

// Resume Back to normal track
    chassis.moveToPoint(54.446,-47.002, 1500, {.forwards = false, .maxSpeed = 60});
//    pros::delay(900);


// Third Wall Reset
chassis.turnToHeading(180,900);
resetToDistance(635, true);
 

// Line up X with matchloader

   chassis.turnToHeading(90, 900);  

// Start intake and lower matchload_piston

   intake.telOP(true, false, false, false, false);

   set_matchload_piston_state(true);
   pros::delay(50);
//Matchloader interaction phase #3
    chassis.moveToPoint(61.013,-46.477, 1000, {.maxSpeed = 60});
    
pros::delay(2000);

// raise Matchload_piston

    chassis.moveToPoint(49.848,-47.002, 1200, {.maxSpeed = 60});



   chassis.turnToHeading(220, 900);  
    

// Motion Chaining 
// ADD 4 POINTS
   chassis.moveToPose(46.418,-52.304,236.5, 1500, {.minSpeed = 60});

   chassis.moveToPose(46.695,-51.665,230.2, 1500, {.minSpeed = 60});

   chassis.moveToPose(42.624,-54.883,244.7, 1500, {.minSpeed = 60});

   chassis.moveToPose(37.391,-58.062,252, 1500, {.minSpeed = 60});

    chassis.moveToPose(31.881,-59.531, 270, 1500, {.minSpeed = 60});
    

// Move to our side of field
    chassis.moveToPoint(-40.347,-58.824, 5000, {.maxSpeed = 60});
    

  chassis.turnToHeading(220, 900);  
    

//Align X with matchloader
    chassis.moveToPoint(-40.347,-47.002, 1200, {.maxSpeed = 60});
    

// Fourth Wall Reset

resetToDistance(804, false);

chassis.turnToHeading(270, 900); 

//Score in Right Long Goal 
   chassis.moveToPoint(-26.876,-47.089, 2500, {.forwards = false, .maxSpeed = 60});

pros::delay(500);

   intake.telOP(false, true, false, false, false);
    pros::delay(1500);

// Start intake and lower matchload_piston
   intake.telOP(true, false, false, false, false);
   set_matchload_piston_state(true);
 
pros::delay(50); 
//Matchloader interaction phase #4
   chassis.moveToPoint(-64.428,-46.564, 1200);
    
pros::delay(2000);

//Score in Right Long Goal 
   chassis.moveToPoint(-26.876,-47.089, 2500, {.forwards = false, .maxSpeed = 60});


// raise Matchload_piston
 set_matchload_piston_state(false);

   intake.telOP(false, true, false, false, false);
    pros::delay(1500);

   intake.telOP(false, false, false, false, false);


// Move back
  chassis.moveToPoint(-35.31,-46.951, 1000, {.maxSpeed = 60});
    

 chassis.turnToHeading(310, 900);  
    


//Motion Chaining
// ADD MORE POINTS
   chassis.moveToPose(-45.394,-41.417,311.5, 1500, {.minSpeed = 60});
  chassis.moveToPose(-51.316,-36.139, 325, 1500, {.minSpeed = 60});
  chassis.moveToPose(-55.763,-29.405, 337.5, 1500, {.minSpeed = 60});
    chassis.moveToPose(-58.674,-21.097, 350, 1500, {.minSpeed = 60});
 chassis.moveToPose(-60.824,-13.321, 356.5, 1500, {.minSpeed = 60});

//Enter into Parking

  chassis.moveToPoint(-61.301,0.055, 2000, {.maxSpeed = 60});


// // 1. Setup Initial State
//     chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
//     chassis.setPose(0, 0, 0);

//     // 2. Set Piston States
//     set_doinker_piston_state(true);

//     // 3. Start Intake and Move to First Position
//     // We access the global motor group directly
//     intake.telOP(true, false, false, false, false);

//     // Move to x: 10.174, y: 34.154, theta: 20.36
//     chassis.moveToPose(10.174, 34.154, 20.36, 2000, {.maxSpeed = 300});
//     chassis.waitUntilDone();

//     // 4. Move to Match Load Ready Position
//     chassis.turnToHeading(120, 2000, {.maxSpeed = 300});
//     chassis.waitUntilDone();

//     chassis.moveToPoint(40.2556, 7, 2000, {.maxSpeed = 50});
//     chassis.waitUntilDone();

//     chassis.turnToHeading(180, 2000, {.maxSpeed = 300});
//     chassis.waitUntilDone();

//     chassis.moveToPose(42, 23, 180, 2000, {.forwards = false, .maxSpeed = 100});
//     chassis.waitUntilDone();

//     intake.telOP(false, true, false, false, false);

//     pros::delay(1500);

//     intake.telOP(true, false, false, false, false);

//     // open match loader
//     set_matchload_piston_state(true);
//     pros::delay(500); // wait for piston to actuate

//     // 6. Alignment / Interaction Phase
//     chassis.moveToPoint(40.2556, -7.11, 3500, {.maxSpeed = 85, .minSpeed = 80});
//     chassis.waitUntilDone();
//     //pros::delay(1750); // wait for any oscillations to settle

//     // Manual slow push logic
//     // Your provided code defines 'leftMotors' and 'rightMotors' as global MotorGroups.
//     // We use .move_velocity() directly on them, removing the 'drivetrain->' pointer syntax.
//     leftMotors.move_velocity(10);
//     rightMotors.move_velocity(10);
//     pros::delay(2750);
    
//     // Stop manual push
//     leftMotors.move_velocity(0);
//     rightMotors.move_velocity(0);

//     // 7. Back away and Final Intake
//     // Move backwards (forwards = false)
//     chassis.moveToPose(42, 23, 180, 2000, {.forwards = false, .maxSpeed = 100});
//     chassis.waitUntilDone();

//     intake.telOP(false, true, false, false, false);

// set_matchload_piston_state(false);

// pros::delay(2500);
// 	chassis.moveToPoint(42, 13, 1199); //node 31
// 	pros::delay(50);
// 	chassis.turnToHeading(230, 906);
//     chassis.moveToPoint(28,-12.5, 2006); //node 32
// 	chassis.turnToHeading(250, 906);
//     chassis.moveToPoint(19, -17.5, 1364); //node 33
// 	pros::delay(50);
//     chassis.turnToHeading(260, 834);
//     chassis.moveToPoint(16, -21.5, 1364); //node 33
// 	chassis.turnToHeading(270, 834);
//     pros::delay(300);
//     chassis.moveToPoint(12, -21.5, 2300); //node 34
// 	set_matchload_piston_state(true);
//     pros::delay(500); // wait for piston to actuate
// 	chassis.moveToPoint(-11, -21.5, 1364, {.minSpeed = 450}); //node 33
//     pros::delay(1500);
//     set_matchload_piston_state(false);







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
  // 1. Setup Initial State
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
  chassis.setPose(0, 0, 0);
  intake.telOP(true, false, false, false, false);

  // 2. Get middle blocks
  chassis.moveToPose(-5.06, 26.89, -11.36, 2000);
  chassis.waitUntil(10);
  set_matchload_piston_state(true);
  pros::delay(500);
  set_matchload_piston_state(false);
  chassis.waitUntilDone();
  chassis.turnToHeading(-136.8, 1000);
  chassis.waitUntilDone();

  // 3. Score middle blocks
  chassis.moveToPoint(10.75, 37.5, 2000, {.forwards = false, .earlyExitRange=2.5});
  chassis.waitUntilDone();
  intake.telOP(false, false, true, false, false);
  pros::delay(1500);
  intake.telOP(true, false, false, false, false);

  // 4. Move towards left match loader
  chassis.moveToPoint(-28, 10, 2000, {.maxSpeed=50});
  chassis.waitUntilDone();
  matchload_activate(true);
  chassis.turnToHeading(-180, 1000);
  chassis.waitUntilDone();

  // 5. Interact with left match loader
  chassis.arcade(85, 0);
  pros::delay(1100);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.arcade(5,0);
  pros::delay(750); // wait for any oscillations to settle

  // 6. Score in long goal
    set_score_piston_state(false);
  chassis.moveToPoint(-27, 24.25, 2000, {.forwards = false, .earlyExitRange=2.5});
  chassis.waitUntilDone();
  set_score_piston_state(false);
  intake.telOP(false, true, false, false, false);
  set_score_piston_state(false);
  pros::delay(1500);
}


void AWP_auton() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.setPose(-47.285, -16.636, 180);


    // Move in front of match loader   
    chassis.moveToPoint(-46.000, -49.913, 4500, { .maxSpeed = 200, .minSpeed = 15});

    chassis.turnToHeading(270.0, 1000);

    // Open match loader
    set_matchload_piston_state(true);
    pros::delay(20);

    intake.telOP(true, false, false, false, false);

    // Alignment / interaction phase with matchloader 
    chassis.moveToPoint(-60, -51.913,4000,{ .maxSpeed = 70, .minSpeed = 25 });
    // chassis.waitUntilDone();
    
    pros::delay(1400);


    // Move backwards to score long goal
    chassis.moveToPoint(-25.484, -51.913,2200,{ .forwards = false, .maxSpeed = 80, .minSpeed = 30 });
    //chassis.waitUntilDone();

    pros::delay(800);

    intake.telOP(false,true, false, false, false);
    pros::delay(1500);

    intake.telOP(false, false, false, false, false);

    set_matchload_piston_state(false);

    chassis.moveToPoint(-37.845, -47.113,1300,{ .maxSpeed = 80, .minSpeed = 60 });
    //chassis.waitUntilDone();



    // Move towards middle balls
    intake.telOP(true, false, false, false, false);

    chassis.turnToHeading(15, 800, { .maxSpeed = 300, .minSpeed = 40 });
    //chassis.waitUntilDone();
    //pros::delay(150);
    //chassis.moveToPoint(-24.36, -24.36, 1800, { .maxSpeed = 100, .minSpeed = 30 });
    //chassis.waitUntilDone();
    pros::delay(150);
    chassis.moveToPoint(-27.899, 26.16, 3200, { .maxSpeed = 100});
    pros::delay(1600);
    set_matchload_piston_state(true);
   //chassis.waitUntilDone();
    //pros::delay(150); 
    chassis.turnToHeading(315, 1200, {.minSpeed = 30});
    //chassis.waitUntilDone();
    
    // Score middle goal
    printf("X: %f, Y: %f, Theta: %f\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    chassis.moveToPoint(-15, 11.399, 2000, { .forwards = false });
    //chassis.waitUntilDone();
    printf("X: %f, Y: %f, Theta: %f\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
  
  pros::delay(100);
  intake.telOP(false, false, true, false, false);
  pros::delay(1200);
  set_score_piston_state(false);
  intake.telOP(false, false, false, false, false);
  set_matchload_piston_state(false);
// Move towards LEFT match loaders
    chassis.moveToPoint(
        -44.364, 44.5,2700,{ .maxSpeed = 100, .minSpeed = 20 });
        //pros::delay(100);
    //chassis.waitUntilDone();
    chassis.turnToHeading(270, 1000, {.maxSpeed = 300});
    //chassis.waitUntilDone();
    // Open match loader
    set_matchload_piston_state(true);
    

    intake.telOP(true, false, false, false, false);

    //pros::delay(300);

    // Alignment / interaction phase with left matchloaders 
    pros::delay(500);
     chassis.moveToPoint(
        -60, 46.5,
        4000,
        { .maxSpeed = 80, .minSpeed = 25 }
    );
    pros::delay(1300);
    //chassis.waitUntilDone();

    // Move backwards to score long goal
    chassis.moveToPoint(
        -30.484,  
          44.8,
        1720,
        { .forwards = false, .maxSpeed = 90, .minSpeed = 40 }
    );
    //chassis.waitUntilDone();

    pros::delay(600);
    intake.telOP(false, true, false, false, false);
    pros::delay(2500);
}

void lateral_tuning() {
    // 1. Use HOLD so it behaves like it will in a match
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    
    // 2. Reset position
    chassis.setPose(0, 0, 0);

    // 3. Drive Forward 24 inches
    chassis.moveToPoint(0, 24, 2000, {.maxSpeed = 100});
    chassis.waitUntilDone();

    // 4. WAIT! This 1 second delay is crucial.
    // It lets you see if the robot "oscillates" (wiggles) after it stops.
    pros::delay(3000);

    // 5. Drive Backwards to Start 
    chassis.moveToPoint(0, 0, 2000, {.forwards = false, .maxSpeed = 100});
    chassis.waitUntilDone();
}

void scorepistoncheck() {
    set_score_piston_state(true);
    pros::delay(2000);
    set_score_piston_state(false);
    pros::delay(2000);
    set_score_piston_state(true);
}

void autonomous() {
//  if (use_right_auton) {
  //  right_auton();
  //} else {
   // left_auton();
 // }
 left_auton();
}
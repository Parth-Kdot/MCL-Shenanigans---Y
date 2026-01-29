#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "localization/distance_localizer.h"
#include "localization/sensor.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "utils/intake.hpp"

#include <limits>

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
  } else if (scoreMid || scoreTop) {
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
pros::Rotation verticalEnc(20);
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
                       20  // maximum acceleration (slew)
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
lemlib::ExpoDriveCurve
    throttleCurve(10,  // joystick deadband out of 127
                  0,   // minimum output where drivetrain will move out of 127
                  1.55 // expo curve gain
    );

// input curve for steer input during driver controlz
lemlib::ExpoDriveCurve
    steerCurve(10,  // joystick deadband out of 127
               0,   // minimum outputslew where drivetrain will move out of 127
               2.10 // expo curve gain
    );

// create the chassis
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors, &throttleCurve, &steerCurve);

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
    uint32_t lastRangeLog = 0;
    while (true) { // print robot location to the brain screen
      const auto pose = chassis.getPose();
      pros::lcd::print(0, "X: %.2f in", pose.x);          // x
      pros::lcd::print(1, "Y: %.2f in", pose.y);          // y
      pros::lcd::print(2, "Theta: %.2f deg", pose.theta); // heading

      const auto sensorSnapshot = distance_localization::sampleSensors();
      const auto printSensor =
          [](const int line, const char label,
             const distance_localization::DistanceSensorReading &reading) {
            const auto distanceValue =
                distance_localization::isValidReading(reading)
                    ? reading.distance_mm
                    : -1.0;
        constexpr double kMillimetersToInches = 0.03937007874;
        pros::lcd::print(line, "%c:%4.2fin C:%02d S:%02d", label,
                         distanceValue * kMillimetersToInches,
                         static_cast<int>(reading.confidence),
                         static_cast<int>(reading.object_size));
          };
      printSensor(4, 'L', sensorSnapshot.left);
      printSensor(5, 'F', sensorSnapshot.front);
      printSensor(6, 'R', sensorSnapshot.right);
      printSensor(7, 'B', sensorSnapshot.back);

      const double headingDeg = imu.is_calibrating()
                                    ? std::numeric_limits<double>::quiet_NaN()
                                    : imu.get_heading();
      if (const auto distancePose =
              distance_localization::estimatePose(sensorSnapshot, headingDeg);
          distancePose.has_value()) {
        controller.print(0, 0, "dX:%5.1f dY:%5.1f", distancePose->x,
                         distancePose->y);
        if (pros::millis() - lastRangeLog > 250) {
          lemlib::telemetrySink()->info("Range pose: ({:.2f}, {:.2f}, {:.2f})",
                                        distancePose->x, distancePose->y,
                                        distancePose->theta);
          lastRangeLog = pros::millis();
        }
      } else {
        controller.print(0, 0, "dLoc unavailable   ");
      }
      lemlib::telemetrySink()->info("Chassis pose: {}", pose);

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
    bool intakeForwardButton =
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    bool intakeBackwardButton =
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    bool flapButton = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    bool pR = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1); // score
    bool pY = controller.get_digital(
        pros::E_CONTROLLER_DIGITAL_RIGHT); // match loader
    bool pL2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2); // doinker

    // move the robot
    chassis.curvature(leftY, leftX);

    // control the intake motors
    if (intakeForwardButton) {
      bottom_intake.move_velocity(600);
      top_intake.move_velocity(0);
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

    // control the pistons via toggle helpers
    control_score_piston(pR);
    control_matchload_piston(pY);
    control_doinker_piston(pL2);

    // delay to save resources
    pros::delay(10);
  }
}
// =============================================================
//  SKILLS AUTONOMOUS ROUTINE
// =============================================================
void skills() {
  intake.telOP(true, false, false, false, false);
  // chassis.setPose(0, 0, 0);

  chassis.setPose(-49.920000, 15.120000, 85.236000);

  chassis.turnToHeading(90.0, 503);
  chassis.moveToPoint(-36.24, 15.12, 1245);
  pros::delay(50);
  chassis.turnToHeading(52.815294, 805);
  chassis.moveToPoint(-22.32, 25.68, 1379);
  pros::delay(50);
  chassis.turnToHeading(312.633752, 1192);
  chassis.moveToPoint(-7.2, 11.76, 1478, {.forwards = false});
  pros::delay(1000);
  middle_goal_score(true);

  // pros::delay(1000);

  chassis.turnToHeading(318.868204, 536);
  chassis.moveToPoint(-38.64, 47.76, 2141);
  chassis.waitUntil(11.144602);
  middle_goal_score(false);
  chassis.waitUntilDone();
  pros::delay(50);
  chassis.turnToHeading(258.310631, 969);
  chassis.moveToPoint(-45.6, 46.32, 1012);
  chassis.waitUntil(0.481858);

  matchload_activate(true);
  chassis.waitUntilDone();
  pros::delay(300);
  chassis.turnToHeading(270, 646);
  chassis.moveToPoint(-74.32, 46.32, 3420, {.maxSpeed = 50}, false); // node 6

  // pros::delay(2000);
  chassis.moveToPoint(-45.12, 46.32, 1436, {.forwards = false}); // node 7
  chassis.waitUntil(14.629714);
  matchload_activate(false);

  intake.telOP(true, false, false, false, false);
  chassis.waitUntilDone();
  pros::delay(50);
  chassis.turnToHeading(24.034288, 1258);
  chassis.moveToPoint(-37.2, 64.08, 1443); // node 8
  // pros::delay(50);
  // chassis.turnToHeading(88.174458, 991);
  // chassis.moveToPoint(-19.922253, 64.630685, 1373);
  // pros::delay(50);
  // chassis.moveToPoint(-3.843288, 65.143162, 1332);
  // pros::delay(50);
  // chassis.moveToPoint(10.100794, 65.587595, 1255);
  // pros::delay(50);
  // chassis.moveToPoint(23.04, 66.0, 1217);
  // pros::delay(50);
  chassis.turnToHeading(91.487868, 465);
  chassis.moveToPoint(41.52, 70.52, 1413); // node 13
  pros::delay(50);
  chassis.turnToHeading(177.207298, 1116);
  chassis.moveToPoint(42.0, 53, 1573); // node 14
  pros::delay(50);
  chassis.turnToHeading(90.855097, 1120);

  chassis.moveToPoint(23, 53, 1000, {.forwards = false}, false); // node 15
  long_goal_score(true);
  pros::delay(2500);
  chassis.waitUntil(11.940754);

  // initial match, top left score

  chassis.turnToHeading(90, 461);       // prev89
  chassis.moveToPoint(47.76, 53, 1518); // prey56 //node 16?? ///current 48
  chassis.waitUntil(16.507699);

  // top left matchload
  matchload_activate(true);
  chassis.waitUntilDone();
  pros::delay(500);
  chassis.turnToHeading(85, 461);
  chassis.moveToPoint(65.52, 54, 3389, {.maxSpeed = 50}); // prev 5 //node 17
  // pros::delay(2000);
  chassis.turnToHeading(90.0, 461);
  chassis.moveToPoint(23, 52.5, 1500, {.forwards = false},
                      false); // node 18 //scoring on long goal
  // chassis.waitUntil(32.81013);
  //__________above no change is good

  // top left 2nd matchload score
  matchload_activate(false);

  long_goal_score(true);
  pros::delay(2500);
  chassis.moveToPoint(45, 53, 1539); // node 19

  chassis.turnToHeading(180, 1204);
  // LOCALIZATION RESET: Near left wall (X = -72), reset X coordinate
  // Robot is at the wall, heading 270 (facing left), so left sensor points at
  // wall localizer.resetXWithHeading(-72.0f);)

  // chassis.moveToPoint(30.48, -26.16, 2975); //node 20
  // pros::delay(50);
  // chassis.turnToHeading(144.833564, 883);
  chassis.moveToPoint(45, -49, 2000000, {.maxSpeed = 80}, false); // ode 21

  pros::delay(200);

  if (distFront.get_distance() > 600) {
    while (distFront.get_distance() > 530) {
      leftMotors.move_velocity(50);
      rightMotors.move_velocity(50);
    }
  } else if (distFront.get_distance() < 600) {
    while (distFront.get_distance() < 530) {
      leftMotors.move_velocity(-50);
      rightMotors.move_velocity(-50);
    }
  }
  leftMotors.move_velocity(0);
  rightMotors.move_velocity(0);
  chassis.turnToHeading(175, 1000);
  chassis.setPose(chassis.getPose().x, -53, 180);

  // localizer.resetXWithHeading(72)
  // chassis.waitUntil(21.38118);

  // top right matchload
  //  chassis.waitUntilDone();
  //  pros::delay(500);

  chassis.turnToHeading(90, 941, {}, false);

  long_goal_score(false);
  matchload_activate(true);
  pros::delay(400);

  chassis.moveToPoint(65, -53, 1397, {}, false); // node 22

  chassis.waitUntilDone();

  // 	//reset code HERE. node 6 is = node 22.
  // 	chassis.setPose(resetPosX, resetPosY, -90);

  // 	pros::delay(2000);

  // 	chassis.moveToPoint(-45.12, 46.32, 1436, {.forwards = false}); //node 7
  // 	chassis.waitUntil(14.629714);
  // 	matchload_activate(false);

  // 	intake.telOP(true, false, false, false, false);
  // 	chassis.waitUntilDone();
  // 	pros::delay(50);
  // 	chassis.turnToHeading(24.034288, 1258);
  // 	chassis.moveToPoint(-37.2, 64.08, 1443); //node 8
  // 	chassis.turnToHeading(91.487868, 465);
  // 	chassis.moveToPoint(41.52, 70.52, 1413); //node 13
  // 	pros::delay(50);
  // 	chassis.turnToHeading(177.207298, 1116);
  // 	chassis.moveToPoint(42.0, 53, 1573); // node 14
  // 	pros::delay(50);
  // 	chassis.turnToHeading(90.855097, 1120);

  // 	chassis.moveToPoint(23, 53, 1000, {.forwards = false}, false); //node 15
  // 	long_goal_score(true);
  // 	pros::delay(2500);
  // 	chassis.waitUntil(11.940754);

  // 	//initial match, top left score

  // 	chassis.turnToHeading(90, 461); //prev89
  // 	chassis.moveToPoint(47.76, 53, 1518); //prey56 //node 16?? ///current 48
  // 	chassis.waitUntil(16.507699);

  // 	//top left matchload
  // 	matchload_activate(true);
  // 	chassis.waitUntilDone();
  // 	pros::delay(500);
  // 	chassis.turnToHeading(85, 461);
  // 	chassis.moveToPoint(65.52, 54, 3389, {.maxSpeed = 50}); //prev 5 //node
  // 17
  // 	// pros::delay(2000);
  // 	chassis.turnToHeading(90.0, 461);
  // 	chassis.moveToPoint(23, 52.5, 1500, {.forwards = false}, false); //node
  // 18 //scoring on long goal
  // 	// chassis.waitUntil(32.81013);
  // //__________above no change is good

  // 	//top left 2nd matchload score
  // 	matchload_activate(false);

  // 	long_goal_score(true);
  // 	pros::delay(2500);
  // 	chassis.moveToPoint(45, 53, 1539); //node 19

  pros::delay(2000);
  chassis.turnToHeading(90, 461);
  // chassis.moveToPoint(44.88, -53, 3727, {.forwards = false}); //node 23
  chassis.moveToPoint(45, -49, 2000000, {.maxSpeed = 80},
                      false); // ode 21 NEW NODE 23
  chassis.waitUntil(13.722215);

  // top right finished matchload
  matchload_activate(false);
  chassis.waitUntilDone();
  pros::delay(50);
  chassis.turnToHeading(222.909841, 1317);
  chassis.moveToPoint(33, -73.08, 1561); // node 24
  pros::delay(50);
  chassis.turnToHeading(270, 884);
  chassis.moveToPoint(-45.36, -73.08, 2753); // node 25
  pros::delay(50);

  // bottom right section
  chassis.turnToHeading(354.897835, 1108);
  chassis.moveToPoint(-46.56, -47, 1237); // node 26
  pros::delay(50);
  chassis.turnToHeading(270.0, 1112);
  chassis.moveToPoint(-23, -48, 1500, {.forwards = false}); // node 27

  // bottom right score
  long_goal_score(true);
  pros::delay(2500);
  matchload_activate(true);
  pros::delay(50);
  chassis.moveToPoint(-61.44, -47, 3870,
                      {.maxSpeed = 50}); // nodem28 //bottom right matchload

  chassis.waitUntilDone();
  pros::delay(50);
  chassis.moveToPoint(-51.84, -47, 1941, {.forwards = false}); // node 29
  pros::delay(500);
  chassis.moveToPoint(-24.96, -47, 1663, {.forwards = false}); // node 30
  chassis.waitUntil(20.017067);

  // bottom right matchload score
  matchload_activate(false);
  long_goal_score(true);
  pros::delay(2500);
  chassis.waitUntilDone();
  pros::delay(50);
  chassis.turnToHeading(268.898294, 461);
  chassis.moveToPoint(-37.44, -49, 1199); // node 31
  pros::delay(50);
  chassis.turnToHeading(319.820766, 906);
  chassis.moveToPoint(-64.8, -17.28, 2006); // node 32
  pros::delay(50);
  chassis.turnToHeading(0.806929, 834);
  chassis.moveToPoint(-64.56, -0.24, 1364); // node 33
}

void autonomous() {
  skills(); 
}
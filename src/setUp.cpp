#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "setUp.h"


pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue); // left motor group
pros::MotorGroup rightMotors({1, 2, 3}, pros::MotorGearset::blue); // right motor group - all reversed.


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);


//Intake
pros::Motor intake(-15); // reverse the direction


//Piston mogo mech
pros::adi::Pneumatics mogoMech('A', false);


//Hang
pros::adi::Pneumatics hang('B', false);


//LED CLASS
pros::adi::Led led1('C', 30);


// Inertial Sensor on port 10
pros::Imu imu(10);


// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(17);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(16);
// horizontal tracking wheel.
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -0.944596);
// vertical tracking wheel.
lemlib::TrackingWheel vertical(&verticalEnc, 1.98, 0);


// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11, // 10 inch track width (from L-->center)
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              1 // horizontal drift is 2. If we had traction wheels, it would have been 8
);


/* Siona's Tuning
lemlib::ControllerSettings linearController (9, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              7, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);
*/


//Luke's Tuning
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              0.25, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              32 // maximum acceleration (slew)
);




lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                              -.1, // integral gain (kI)
                                              12, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              50, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);


// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);


// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);


// create the chassis
lemlib::Chassis theChassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

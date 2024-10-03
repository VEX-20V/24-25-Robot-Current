#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#include "lemlib/chassis/trackingWheel.hpp"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

//Intake 
pros::Motor intake(-4); // reverse the direction 

//Piston mogo mech 
pros::adi::Pneumatics mogoMech('A', true);

pros::adi::Pneumatics hang('B', true);

// Inertial Sensor on port 10
pros::Imu imu(10);

pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue); // left motor group
pros::MotorGroup rightMotors({1, 2, 3}, pros::MotorGearset::blue); // right motor group - all reversed. 

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(14);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(16);
// horizontal tracking wheel. 
lemlib::TrackingWheel horizontal(&horizontalEnc, 2, -0.63610);
// vertical tracking wheel. 
lemlib::TrackingWheel vertical(&verticalEnc, 2, -.3285);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              5.8, // 10 inch track width (from L-->center)
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              1 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
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
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

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
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

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


// path file name is "LukeTest.txt".
// "." is replaced with "_" to overcome c++ limitations
ASSET(LukeTest2_txt);

void autonomous() {
    // set chassis pose
    chassis.setPose(0, 0, 0);
    // lookahead distance: 15 inches
    // timeout: 2000 ms
    chassis.follow(LukeTest2_txt, 15, 2000);
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    pros::Controller master (pros::E_CONTROLLER_MASTER);
    // loop to continuously update motors
    while (true) {

        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);


        //intake controlling
        int speed = 127;
        if(master.get_digital(pros:: E_CONTROLLER_DIGITAL_R1)) {
        intake.move(speed);
        } else if (master.get_digital(pros :: E_CONTROLLER_DIGITAL_R2)) {
        intake.move(-speed);
        } else {
        intake.move(0);
        }

        //Mogo Mech Controlling
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            mogoMech.set_value(false);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            mogoMech.set_value(true);
        }

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            hang.set_value(false);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            hang.set_value(true);
        }

        // delay to save resources
        pros::delay(10);
    }
}

//organization goals are on "Code Day 4: Creating CodeV2" of notebook. 

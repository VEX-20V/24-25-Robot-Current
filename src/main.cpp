#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/rtos.hpp"
//#include "setUp.cpp"

pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue); // left motor group
pros::MotorGroup rightMotors({1, 2, 3}, pros::MotorGearset::blue); // right motor group - all reversed. 

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

//Intake 
pros::Motor intake(-15); // reverse the direction 

//Piston mogo mech 
pros::adi::Pneumatics mogoMech('A', true);
pros::adi::Pneumatics hang('B', false);


// Inertial Sensor on port 10
pros::Imu imu(10);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(17);
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

lemlib::ControllerSettings linearController (100, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
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


/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */


void autonIntake(int seconds)
{
    int miliSeconds = seconds * 1000;
    intake.move(127);
    pros::delay(miliSeconds);
    intake.move(0);
}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(BasicPathPt1_txt);
ASSET(BasicPathPt2_txt);


void autonomous() {

    // sets position / origin (what every other position will now be based on)
    chassis.setPose(-59.282, 37.344, 245);

    //moves to mogo
    chassis.moveToPose(-30.776, -26.978, 245, 4000); //motion 1 of 3
    mogoMech.set_value(false); //clamps mogo
    autonIntake(2); //scores preload

    //moves to ring
    chassis.moveToPose(-43.262, -44.411, 225, 4000); //motion 2 of 3
    autonIntake(3); //intakes and scores ring
    pros::delay(1000);
    mogoMech.set_value(false);//releases mogo

    //touches bar
    chassis.moveToPose(-16.642, -12.844, 225, 4000); //motion 3 of 3

    /*
    chassis.moveTo(0, 0, 5000);
    chassis.moveTo(-2.652, -30.216, 5000);
    chassis.moveTo(-13.175, -11.532, 5000);
    chassis.moveTo(4.184, -48.999, 5000);

    chassis.moveTo(-59.282, -37.344, 5000);
    chassis.moveTo(-30.776, -26.978, 5000);
    chassis.moveTo(-43.262, -44.411, 5000);
    chassis.moveTo(-16.642, -12.844, 5000);
    */

    /*
    // set chassis pose
    autonIntake(1);
    chassis.moveToPose(0, 5, 0, 5000);
    chassis.moveToPose(0, 0, 0, 5000);//moves back. 
    //OR does THIS move it back?
    //chassis.setPose(0, -5, 0);
    */

    /*
    mogoMech.set_value(false);//clamps mogo
    pros::delay(1000);
    mogoMech.set_value(false);//releases mogo
    */


    /*    
    chassis.follow(BasicPathPt1_txt, 15, 2000, true);
    pros::delay(1000);
    chassis.follow(BasicPathPt2_txt, 15, 2000, true);
    */




    
    //chassis.moveToPose(0, 0, 90, 5000);
    //chassis.moveToPoint(40.555, 0.517, 5000);
    

    // create a timer that will wait for 1 second
    // check if the timer is done
        

    //chassis.setPose(0, 0, 0);//does this bring it back or make it not move?
    //basically, is it incremental or absolute coordinates? Hopefully absolute plz. 

    // lookahead distance: 15 inches
    // timeout: 2000 ms
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
            mogoMech.set_value(false);//clamps mogo
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            mogoMech.set_value(true);//releases mogo
        }

        //Hang Controlling
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            hang.set_value(false);//clamps hang ???
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            hang.set_value(true);//releases hang ???
        }


        // delay to save resources
        pros::delay(10);
    }
}

//organization goals are on "Code Day 4: Creating CodeV2" of notebook. 

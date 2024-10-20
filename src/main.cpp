#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/rtos.hpp"
#include "motion.hpp"
#include "setup.hpp"


// pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue); // left motor group
// pros::MotorGroup rightMotors({1, 2, 3}, pros::MotorGearset::blue); // right motor group - all reversed.


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

void red_lights() {
    led1.set_all(0xFF0000);
}


void blue_lights() {
    led1.set_all(0x0000FF);
}

//*FOR BUTTONS*
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    theChassis.calibrate(); // calibrate sensors
    pros::lcd::register_btn0_cb(on_center_button);//*FOR BUTTONS*


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
            //pros::lcd::print(0, "X: %f", theChassis.getPose().x); // x
            // pros::lcd::print(1, "Y: %f", theChassis.getPose().y); // y
            // pros::lcd::print(2, "Theta: %f", theChassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", theChassis.getPose());
            // delay to save resources
            pros::delay(50);

            //set robot lights to blue on center button
            pros::lcd::register_btn0_cb(red_lights);
            pros::lcd::register_btn1_cb(blue_lights);

            // //set robot lights to red on center button
            
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

void autonomous()
{
    
    mogoMech.set_value(false); // start w/ MOGO released
    hang.set_value(false);//start w/ HANG released
    RED_Neg_RingAndBar(theChassis, mogoMech, intake);


    //TurnTest(theChassis);
    // //StraitMOGOTest();
    //void BLUE_LeaveStart();
    //TurnTest();
    //autonPath1();
    //TouchBarAuton();
    //BLUE_LeaveStart();

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
        theChassis.arcade(leftY, rightX);

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
            mogoMech.set_value(true);//clamps mogo
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            mogoMech.set_value(false);//releases mogo
        }


        //Hang Controlling
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            hang.set_value(false);//releases hang
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            hang.set_value(true);//clamps hang
        }

        // delay to save resources
        pros::delay(10);
    }
}


//organization goals are on "Code Day 4: Creating CodeV2" of notebook.
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"
#include "lemlib/chassis/chassis.hpp"
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
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


void red_lights() {
    led1.set_all(0xFF0000);
}


void blue_lights() {
    led1.set_all(0x0000FF);
}

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

            //set robot lights to blue on center button
            pros::lcd::register_btn0_cb(red_lights);
            pros::lcd::register_btn1_cb(blue_lights);

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




//auton helper functions
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


//auton Path functions
void autonPath1()
{
    mogoMech.set_value(false); //releases mogo


    // sets position / origin (what every other position will now be based on)
    chassis.setPose(-47.469, -37.219, 235);


    //moves to mogo
    chassis.moveToPose(-29.758, -26.296, 235, 4000, {false}); //motion 1 of 3


    mogoMech.set_value(true); //clamps mogo
    autonIntake(2); //scores preload


    chassis.turnToHeading(165, 4000);


    //moves to ring
    chassis.moveToPose(-23.606, -47.094, 165, 4000, {true}); //motion 2 of 3
    autonIntake(3); //intakes and scores ring
    pros::delay(1000);
    mogoMech.set_value(false);//releases mogo


    chassis.turnToHeading(205, 4000);




    //touches bar
    chassis.moveToPose(-9.868, -18.289, 205, 4000, {false}); //motion 3 of 3
}




//testing*************************************************************
void TestMogo()
{
    mogoMech.set_value(false); //releases mogo
    pros::delay(1000);
    mogoMech.set_value(true); //clamp mogo


}


void TurnTest()
{
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 4000);


}


void StraitMOGOTest()
{
    pros::lcd::print(5, "before travelling");
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(0, -24, 0, 4000, {false});
    pros::lcd::print(6, "traveled 24 inches");
    pros::delay(1000);
    mogoMech.set_value(true); //clamps mogo
    chassis.moveToPose(0, 0, 0, 1000);


}//testing*************************************************************

void AutonSkills()
{
    chassis.setPose(-65.405, -35.079, 235);
    chassis.moveToPose(-48.873, -24.2, 235, 3000, {false});
    pros::delay(1000);
    mogoMech.set_value(true); //clamps mogo
    chassis.turnToHeading(25, 3000);
    chassis.moveToPose(-48.873, -24, 25, 5000);

}

void TouchBarAuton()
{
    chassis.setPose(-48.052, -32.128, 235);

    //moves to mogo
    chassis.moveToPose(-15.541, -2.843, 235, 8000, {false});
}

void BLUE_LeaveStart()
{
    
    chassis.setPose(63.493, -50, 270);

    //moves off start facing forward
    chassis.moveToPose(41.02, -50, 270, 8000, {true});

}

void RED_LeaveStart()
{
    
    chassis.setPose(-63.493, 90, 270);

    //moves off start facing forward
    chassis.moveToPose(-41.02, 90, 270, 8000, {false});

}


void RED_RingAndBar()
{
    
    //start backwards
    chassis.setPose(-47.469, 37.219, 305);

    chassis.moveToPose(-60.07, 45.614, 305, 1000, {true});


    chassis.moveToPose(-29.758, 26.296, 305, 6000, {false});
    pros::delay(2000);


    mogoMech.set_value(true); //clamps mogo
    pros::delay(2500);
    autonIntake(4); //scores ring- PRELOAD

    chassis.turnToHeading(215, 3000); 
    mogoMech.set_value(false); //releases mogo

    chassis.turnToHeading(305, 3000);

    chassis.moveToPose(-6.664, 10.901, 305, 5000, {false});


}


void autonomous()
{
    // mogoMech.set_value(false); // start w/ MOGO released
    // hang.set_value(false);//start w/ HANG released
    // //StraitMOGOTest();

    // RED_RingAndBar();

    //Going forward and straight
    chassis.setPose(0, 0, 0);
    chassis.moveToPose(0, 24, 0, 10000, {true});   

    //void BLUE_LeaveStart();
    /*
    
    */
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
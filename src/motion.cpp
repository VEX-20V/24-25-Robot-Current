// #include "lemlib/api.hpp" // IWYU pragma: keep
#include "motion.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.hpp"
// #include "pros/llemu.hpp"
// #include "setUp.cpp"


//auton helper functions

void DriveTest(lemlib::Chassis& chassis)
{
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 24, 10000);
    chassis.turnToHeading(90, 4000);
}

void autonIntake(pros::Motor intake, int seconds)
{
    int miliSeconds = seconds * 1000;
    intake.move(127);
    pros::delay(miliSeconds);
    intake.move(0);
}

// void WallStake(pros::Motor wallStake, int seconds)
// {
//     int miliSeconds = seconds * 1000;
//     wallStake.move(127);
//     pros::delay(miliSeconds);
//     wallStake.move(0);
// }

void autonWallStake(pros::Motor wallStake)
{
    wallStake.move(127); //127
    pros::delay(700);
    wallStake.move(6);
    // wallStake.brake();
    pros::delay(1000);
    wallStake.move(-40);
    pros::delay(200);
    wallStake.move(0);
    // wallStake.move(0);
}


//*****************************************Ring and Bar Queue Autons*******************************************************************************
//Red Team, - Corner
void RED_Neg_RingAndBar(lemlib::Chassis& chassis, pros::adi::Pneumatics mogoMech, pros::Motor intake)
{
        //start backwards
    chassis.setPose(-54.205, 43.013, 302);

    chassis.moveToPose(-27.364, 25.561, 302, 6000, {.forwards=false, .maxSpeed = 127, .minSpeed = 40});
    pros::delay(2000);


    mogoMech.set_value(true); //clamps mogo
    pros::delay(2000);
    autonIntake(intake, 2); //scores ring- PRELOAD

    chassis.turnToHeading(25, 2000, {.maxSpeed = 127, .minSpeed = 40});
    pros::delay(400);

    chassis.setPose(0, 0, 0);

    chassis.moveToPose(0, 24, 0, 6000, {.forwards=true, .maxSpeed = 127, .minSpeed = 40});
    autonIntake(intake, 5); //scores EXTRA ring- PRELOAD
    mogoMech.set_value(false); //releases mogo
    
    //touch bar
    // chassis.turnToHeading(345, 2000);
    // chassis.moveToPose(-12.899, 8.69, 345, 6000, {.forwards=false, .maxSpeed = 127, .minSpeed = 100});

}

//Red Team, + Corner
void RED_Pos_RingAndBar(lemlib::Chassis& chassis, pros::adi::Pneumatics mogoMech, pros::Motor intake)
{
    //start backwards
    chassis.setPose(-54.205, -43.013, 238);

    chassis.moveToPose(-27.364, -25.561, 238, 6000, {.forwards=false, .maxSpeed = 127, .minSpeed = 40});
    pros::delay(2000);


    mogoMech.set_value(true); //clamps mogo
    pros::delay(2000);
    autonIntake(intake, 2); //scores ring- PRELOAD


    // pros::delay(1000);
    mogoMech.set_value(false); //releases mogo

    chassis.moveToPose(-6.664, -10.901, 238, 5000, {.forwards=false, .maxSpeed = 127, .minSpeed = 40});
}

void BLUE_Pos_RingAndBar(lemlib::Chassis& chassis, pros::adi::Pneumatics mogoMech, pros::Motor intake)
{
    //start backwards
    chassis.setPose(54.205, -43.013, 122);
    
    chassis.moveToPose(27.364, -25.561, 122, 6000, {.forwards=false, .maxSpeed = 127, .minSpeed = 50});
    pros::delay(2000);


    mogoMech.set_value(true); //clamps mogo
    pros::delay(2000);
    autonIntake(intake, 2); //scores ring- PRELOAD


    // pros::delay(1000);
    mogoMech.set_value(false); //releases mogo

    chassis.moveToPose(6.664, -10.901, 122, 5000, {.forwards=false, .maxSpeed = 127, .minSpeed = 50});

}


//*******************************************SKILLS AUTONS**************************************************** */

// void SKILLS_OneMogo(lemlib::Chassis& chassis, pros::adi::Pneumatics mogoMech, pros::Motor intake)
// {
//     //start backwards
//     chassis.setPose(-60.894, -33.339, 235);

//     //go to mogo
//     chassis.moveToPose(-44.49, -21.37, 235, 6000, {.forwards=false, .maxSpeed = 127, .minSpeed = 100});
//     pros::delay(2000);


//     mogoMech.set_value(true); //clamps mogo
//     pros::delay(2000);
//     autonIntake(intake, 2); //scores ring- PRELOAD


//     chassis.turnToHeading(30, 3000 );

//     //score mogo in corner
//     chassis.moveToPose(-66.079, -66.462, 30, 5000, {.forwards=false, .maxSpeed = 127, .minSpeed = 100});
//     mogoMech.set_value(false); //releases mogo

// }


// // get a path used for pure pursuit
// // this needs to be put outside a function
// ASSET(BasicPathPt1_txt);
// ASSET(BasicPathPt2_txt);


// //auton Path functions
// void autonPath1()
// {
//     mogoMech.set_value(false); //releases mogo


//     // sets position / origin (what every other position will now be based on)
//     theChassis.setPose(-47.469, -37.219, 235);


//     //moves to mogo
//     theChassis.moveToPose(-29.758, -26.296, 235, 4000, {false}); //motion 1 of 3


//     mogoMech.set_value(true); //clamps mogo
//     autonIntake(2); //scores preload


//     theChassis.turnToHeading(165, 4000);


//     //moves to ring
//     theChassis.moveToPose(-23.606, -47.094, 165, 4000, {true}); //motion 2 of 3
//     autonIntake(3); //intakes and scores ring
//     pros::delay(1000);
//     mogoMech.set_value(false);//releases mogo


//     theChassis.turnToHeading(205, 4000);




//     //touches bar
//     theChassis.moveToPose(-9.868, -18.289, 205, 4000, {false}); //motion 3 of 3
// }




// //testing*************************************************************
// void TestMogo()
// {
//     mogoMech.set_value(false); //releases mogo
//     pros::delay(1000);
//     mogoMech.set_value(true); //clamp mogo


// }

// void StraitMOGOTest()
// {
//     pros::lcd::print(5, "before travelling");
//     theChassis.setPose(0, 0, 0);
//     theChassis.moveToPose(0, -24, 0, 4000, {false});
//     pros::lcd::print(6, "traveled 24 inches");
//     pros::delay(1000);
//     mogoMech.set_value(true); //clamps mogo
//     theChassis.moveToPose(0, 0, 0, 1000);


// }//testing*************************************************************

// void AutonSkills()
// {
//     theChassis.setPose(-65.405, -35.079, 235);
//     theChassis.moveToPose(-48.873, -24.2, 235, 3000, {false});
//     pros::delay(1000);
//     mogoMech.set_value(true); //clamps mogo
//     theChassis.turnToHeading(25, 3000);
//     theChassis.moveToPose(-48.873, -24, 25, 5000);

// }

// void TouchBarAuton()
// {
//     theChassis.setPose(-48.052, -32.128, 235);

//     //moves to mogo
//     theChassis.moveToPose(-15.541, -2.843, 235, 8000, {false});
// }

// void BLUE_LeaveStart()
// {
    
//     theChassis.setPose(63.493, -50, 270);

//     //moves off start facing forward
//     theChassis.moveToPose(41.02, -50, 270, 8000, {true});

// }

// void RED_LeaveStart()
// {
    
//     theChassis.setPose(-63.493, 90, 270);

//     //moves off start facing forward
//     theChassis.moveToPose(-41.02, 90, 270, 8000, {false});

// }


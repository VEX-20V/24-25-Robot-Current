// #include "lemlib/api.hpp" // IWYU pragma: keep
#include "motion.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.hpp"
// #include "pros/llemu.hpp"
// #include "setUp.cpp"


//auton helper functions

void TurnTest(lemlib::Chassis& chassis)
{
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 4000);
}

void autonIntake(pros::Motor intake, int seconds)
{
    int miliSeconds = seconds * 1000;
    intake.move(127);
    pros::delay(miliSeconds);
    intake.move(0);
}

//*****************************************Ring and Bar Queue Autons*******************************************************************************
//Red Team, + Corner
void RED_Neg_RingAndBar(lemlib::Chassis& chassis, pros::adi::Pneumatics mogoMech, pros::Motor intake)
{
    
    //start backwards
    chassis.setPose(-47.469, 37.219, 305);

    chassis.moveToPose(-29.758, 26.296, 305, 6000, {.forwards=false, .maxSpeed = 120, .minSpeed = 80});
    pros::delay(2000);


    mogoMech.set_value(true); //clamps mogo
    pros::delay(2500);
    autonIntake(intake, 4); //scores ring- PRELOAD

    chassis.turnToHeading(215, 3000); 
    mogoMech.set_value(false); //releases mogo

    chassis.turnToHeading(305, 3000);

    chassis.moveToPose(-6.664, 10.901, 305, 5000, {false});
}

//Red Team, - Corner
void RED_Pos_RingAndBar(lemlib::Chassis& chassis, pros::adi::Pneumatics mogoMech, pros::Motor intake)
{
    
    //start backwards
    chassis.setPose(-47.469, -37.219, 235);

    chassis.moveToPose(-29.758, -26.296, 235, 6000, {.forwards=false, .maxSpeed = 127, .minSpeed = 100});
    pros::delay(2000);


    mogoMech.set_value(true); //clamps mogo
    pros::delay(2500);
    autonIntake(intake, 4); //scores ring- PRELOAD

    chassis.turnToHeading(325, 3000); 
    mogoMech.set_value(false); //releases mogo

    chassis.turnToHeading(235, 3000);

    chassis.moveToPose(-6.664, -10.901, 235, 5000, {false});
}



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


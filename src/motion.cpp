// #include "lemlib/api.hpp" // IWYU pragma: keep
#include "motion.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.h"
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
    wallStake.move(115); //127
    pros::delay(1000);
    wallStake.move(6);
    pros::delay(300);
    wallStake.move(-50);
    pros::delay(200);
    wallStake.move(0);
    // wallStake.move(0);
}

// void helperWallStake(pros::Motor wallStake)
// {
//     wallStake.move(85); //127
//     pros::delay(800);
//     wallStake.move(6);
//     pros::delay(3000);
//     // wallStake.move(0);
// }



//*****************************************Queue Autons*******************************************************************************

void LeaveStartBackwards(lemlib::Chassis& chassis)
{
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, -12, 10000, {.forwards=false, .maxSpeed = 127, .minSpeed = 50});
}

void LeaveStartForwards(lemlib::Chassis& chassis)
{
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 12, 10000, {.forwards=true, .maxSpeed = 127, .minSpeed = 50});

}

//Red Team, + Corner
//FOR COMP #3-Granada!!!
void NUE_OneRing(lemlib::Chassis& chassis, pros::adi::Pneumatics mogoMech, pros::Motor intake)
{
    //start backwards
    chassis.setPose(-54.205, -43.013, 238);

    chassis.moveToPose(-27.364, -25.561, 238, 6000, {.forwards=false, .maxSpeed = 127, .minSpeed = 40});
    pros::delay(2000);

    mogoMech.set_value(true); //clamps mogo
    pros::delay(2000);
    autonIntake(intake, 2); //scores ring- PRELOAD
}

//misleading. Should be 2rings but oh wulp. 
void RED_Pos_and_BLUE_Neg_2Rings(lemlib::Chassis& chassis, pros::adi::Pneumatics mogoMech, pros::Motor intake)
{
    //start backwards
    chassis.setPose(-54.205, -43.013, 238);

    chassis.moveToPose(-27.364, -25.561, 238, 6000, {.forwards=false, .maxSpeed = 127, .minSpeed = 40});
    pros::delay(2000);


    mogoMech.set_value(true); //clamps mogo
    pros::delay(2000);
    autonIntake(intake, 2); //scores ring- PRELOAD

    chassis.turnToHeading(170, 3000 );
    pros::delay(2000);
    chassis.moveToPoint(-21.534, -54.984, 4000,{.forwards=true, .maxSpeed = 127, .minSpeed = 50});
    autonIntake(intake, 4); //scores 2nd ring

    //touch bar
    chassis.turnToHeading(185, 3000 );
    chassis.moveToPose(-15.249, 0, 185, 6000, {.forwards=false, .maxSpeed = 127, .minSpeed = 60});

    // chassis.moveToPose(-23.174, -46.239, 170, 6000, {.forwards=true, .maxSpeed = 127, .minSpeed = 40});
}


//now it has been tested. 
void RED_Neg_and_BLUE_Pos_2Rings(lemlib::Chassis& chassis, pros::adi::Pneumatics mogoMech, pros::Motor intake, pros::Motor wallStake)
{
    //start backwards
    chassis.setPose(54.205, -43.013, 122);

    chassis.moveToPose(27.364, -25.561, 122, 6000, {.forwards=false, .maxSpeed = 127, .minSpeed = 45});
    pros::delay(2000);

    mogoMech.set_value(true); //clamps mogo
    pros::delay(1000);
    autonIntake(intake, 2); //scores ring- PRELOAD

    chassis.turnToHeading(185, 3000 );
    // pros::delay(800);
    chassis.moveToPoint(23.83, -45.146, 4000, {.forwards=true, .maxSpeed = 127, .minSpeed = 70});
    autonIntake(intake, 4); //scores 2nd ring

    wallStake.move(115); //127
    pros::delay(1000);
    wallStake.move(6);
    
    //touch bar
    chassis.turnToHeading(187, 1000, {.maxSpeed = 20, .minSpeed = 5});
    chassis.moveToPose(16, 0, 187, 6000, {.forwards=false, .maxSpeed = 127, .minSpeed = 60});
}

//*******************************************SKILLS AUTONS**************************************************** */

//YES 8pt AUTO WORKED!!!!!!!!!!!
void SKILLS_OneMogo(lemlib::Chassis& chassis, pros::adi::Pneumatics mogoMech, pros::adi::Pneumatics square, pros::Motor intake)
{
    //start backwards
    chassis.setPose(-56.049, -27.839, 238);

    //go to mogo
    chassis.moveToPose(-44.49, -21.37, 238, 6000, {.forwards=false, .maxSpeed = 127, .minSpeed = 40});
    pros::delay(500);

    mogoMech.set_value(true); //clamps mogo2
    autonIntake(intake, 2); //scores ring- PRELOAD

    pros::delay(200);

    chassis.turnToHeading(30, 3000 );

    pros::delay(500); //delete delay later?

    //score mogo in corner
    chassis.moveToPose(-84.552, -95.683, 30, 10000, {.forwards=false, .maxSpeed = 127, .minSpeed = 100});
    pros::delay(200);
    mogoMech.set_value(false); //releases mogo
}



//Should do 13pts, might do 18pts
void SKILLS_TwoMogos(lemlib::Chassis& chassis, pros::adi::Pneumatics mogoMech, pros::adi::Pneumatics square, pros::Motor intake)
{
    //start backwards
    chassis.setPose(-56.049, -27.839, 238);

    //go to mogo
    chassis.moveToPose(-44.49, -21.37, 238, 6000, {.forwards=false, .maxSpeed = 127, .minSpeed = 40});
    pros::delay(1000);

    mogoMech.set_value(true); //clamps mogo
    autonIntake(intake, 2); //scores ring- PRELOAD

    pros::delay(200);

    chassis.turnToHeading(30, 3000 );

    pros::delay(500); //delete delay later?

    //score mogo in corner
    chassis.moveToPose(-84.552, -95.683, 30, 5000, {.forwards=false, .maxSpeed = 127, .minSpeed = 100});
    pros::delay(200);
    mogoMech.set_value(false); //releases mogo

    chassis.setPose(-56.049, -27.839, 30);

    //move to mogo
    chassis.turnToHeading(205, 3000 );
    pros::delay(1000);
    chassis.moveToPose(-46, -11.806, 200, 10000, {.forwards=false, .maxSpeed = 127, .minSpeed = 60});
    chassis.moveToPose(-46, 35, 180, 10000, {.forwards=false, .maxSpeed = 127, .minSpeed = 40});
    pros::delay(2000);
    mogoMech.set_value(true); //clamps mogo2

    
    //move to corner
    chassis.turnToHeading(160, 3000 );
    chassis.moveToPose(-70.552, 75.683, 160, 10000, {.forwards=false, .maxSpeed = 127, .minSpeed = 60});
    
    mogoMech.set_value(false); //releases mogo

    chassis.setPose(-66.041, -66.039, 150);
    chassis.turnToHeading(90, 3000 );


    //move to 3rd mogo
    chassis.moveToPose(38, 64.166, 90, 10000, {.forwards=false, .maxSpeed = 127, .minSpeed = 60});
    pros::delay(1000);
    chassis.turnToHeading(170, 3000 );
    pros::delay(1000);
    chassis.moveToPose(67.008, -67.166, 170, 10000, {.forwards=false, .maxSpeed = 127, .minSpeed = 60});



}
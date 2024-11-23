#ifndef MOTION_HPP
#define MOTION_HPP

#include "lemlib/chassis/chassis.hpp"

void DriveTest(lemlib::Chassis& chassis);
void autonIntake(pros::Motor intake, int seconds);
void autonWallStake(pros::Motor wallStake);

//Queue
void LeaveStartBackwards(lemlib::Chassis& chassis);
void LeaveStartForwards(lemlib::Chassis& chassis);

void NUE_OneRing(lemlib::Chassis& chassis, pros::adi::Pneumatics mogoMech, pros::Motor intake);
void RED_Pos_and_BLUE_Neg_2Rings(lemlib::Chassis& chassis, pros::adi::Pneumatics mogoMech, pros::Motor intake);
void RED_Neg_and_BLUE_Pos_2Rings(lemlib::Chassis& chassis, pros::adi::Pneumatics mogoMech, pros::Motor intake);

//Finals


//Skills
void SKILLS_OneMogo(lemlib::Chassis& chassis, pros::adi::Pneumatics mogoMech, pros::adi::Pneumatics square, pros::Motor intake);


//nuetral One Ring scores the preload and keeps holding onto mogo.
//RED_Pos_2Rings



//RED_Neg_and_BLUE_Pos_2Rings
//RED_Pos_and_BLUE_Neg_2Rings

#endif
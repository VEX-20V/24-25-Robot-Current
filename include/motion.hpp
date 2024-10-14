#ifndef MOTION_HPP
#define MOTION_HPP

#include "lemlib/chassis/chassis.hpp"

void TurnTest(lemlib::Chassis& chassis);
void autonIntake(pros::Motor intake, int seconds);
void RED_Neg_RingAndBar(lemlib::Chassis& chassis, pros::adi::Pneumatics mogoMech, pros::Motor intake);
void RED_Pos_RingAndBar(lemlib::Chassis& chassis, pros::adi::Pneumatics mogoMech, pros::Motor intake);

// void autonPath1();
// void TestMogo();
// void StraitMOGOTest();
// void AutonSkills();
// void TouchBarAuton();
// void BLUE_LeaveStart();
// void RED_LeaveStart();

#endif
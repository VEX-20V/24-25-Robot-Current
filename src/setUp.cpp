#include "setup.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.hpp"

void SetUp()
{
    pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue); // left motor group
    pros::MotorGroup rightMotors({1, 2, 3}, pros::MotorGearset::blue); // right motor group - all reversed.
}

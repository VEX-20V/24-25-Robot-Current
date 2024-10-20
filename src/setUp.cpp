#include "setup.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/rtos.hpp"

pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue); // left motor group
pros::MotorGroup rightMotors({1, 2, 3}, pros::MotorGearset::blue); // right motor group - all reversed.

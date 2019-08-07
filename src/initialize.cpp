#include "main.h"


#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma clang diagnostic ignored "-Wunused-variable"

void initialize() {
    pros::lcd::initialize();

    pros::Motor initializer_chassis_lf(5, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
    pros::Motor initializer_chassis_lb(10, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
    pros::Motor initializer_chassis_rf(1, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
    pros::Motor initializer_chassis_rb(6, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
    pros::Motor initializer_arm_left(15, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
    pros::Motor initializer_arm_right(11, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
    pros::Motor initializer_collector_left(20, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
    pros::Motor initializer_collector_right(16, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);

}

#pragma clang diagnostic pop

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

#include "main.h"

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
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");

    pros::lcd::register_btn1_cb(on_center_button);


    pros::Motor initializer_chassis_lf(5, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
    pros::Motor initializer_chassis_lb(10, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
    pros::Motor initializer_chassis_rf(1, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
    pros::Motor initializer_chassis_rb(6, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
    pros::Motor initializer_arm_left(15, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
    pros::Motor initializer_arm_right(11, MOTOR_GEARSET_18, false, MOTOR_ENCODER_DEGREES);
    pros::Motor initializer_claw_left(20, MOTOR_GEARSET_18, true, MOTOR_ENCODER_DEGREES);
    pros::Motor initializer_claw_right(16, MOTOR_GEARSET_36, false, MOTOR_ENCODER_DEGREES);

}

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

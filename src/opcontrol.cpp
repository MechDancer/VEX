#include "main.h"
#include "robot.hpp"
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    
    Chassis chassis;
    Arm arm;
    Claw claw;

    
    
    while (true) {
        pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
                         (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                         (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
        int a = master.get_analog(ANALOG_LEFT_Y);
        int b = master.get_analog(ANALOG_RIGHT_X);
        
        chassis.move(a,b);
        
        if (master.get_digital(DIGITAL_L2)) {
            //up
        } else if (master.get_digital(DIGITAL_L1)) {
            //down
        } else {
            //stop
        }

        if (master.get_digital(DIGITAL_R2)) {
            claw.clam();
        } else if (master.get_digital(DIGITAL_R1)) {
            claw.release();
        } else {
            claw.stop();
        }

        pros::delay(20);
    }
}

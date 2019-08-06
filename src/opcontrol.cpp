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
    
    chassis_t   chassis;
    arm_t       arm;
    collector_t collector;
    
    auto position_left_last         = .0,
         position_right_last        = .0;
    
    odometry_t<>     odometry{};
    chassis_config_t chassis_config = {
        37.5,
        5.0,
        5.0
    };
    
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wmissing-noreturn"
    while (true) {
        pros::lcd::print(1, "left %d, right %d", chassis.left_position(), chassis.right_position());
        pros::lcd::print(2, "x %f, y %f, w %f", odometry.x, odometry.y, odometry.theta);
        pros::lcd::print(3, "s %f, a %f", odometry.s, odometry.a);
        pros::lcd::print(4, "arm %d", arm.current_position());
        
        auto a                      = master.get_analog(ANALOG_LEFT_Y),
             b                      = master.get_analog(ANALOG_RIGHT_X);
        auto position_left_current  = chassis.left_position(),
             position_right_current = chassis.right_position();
        
        chassis.move(a, b);
        odometry += wheels_to_odometry(position_left_current - position_left_last,
                                       position_right_current - position_right_last,
                                       chassis_config);
        pros::lcd::print(3, "d_left %f, d_right %f",
                         position_left_current - position_left_last,
                         position_right_current - position_right_last);
        
        if (master.get_digital(DIGITAL_L2)) {
            //up
        } else if (master.get_digital(DIGITAL_L1)) {
            //down
        } else {
            //stop
        }
        
        if (master.get_digital(DIGITAL_R2)) {
            collector.collect();
        } else if (master.get_digital(DIGITAL_R1)) {
            collector.spit();
        } else {
            collector.stop();
        }
        
        position_left_last  = position_left_current;
        position_right_last = position_right_current;
        pros::delay(20);
    }
    #pragma clang diagnostic pop
}

#include "main.h"
#include "robot.hpp"


void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    chassis_t chassis;
    arm_t arm;
    collector_t collector;

    auto position_left_last = .0,
            position_right_last = .0;

    odometry_t<> odometry{};
    chassis_config_t chassis_config = {
            .375,
            .05,
            .05
    };

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"
    while (true) {
        pros::lcd::print(0, "left current %d, right current%d",
                         chassis.left_position(), chassis.right_position());
        pros::lcd::print(1, "x %f, y %f, w %f", odometry.x, odometry.y, odometry.theta);
        pros::lcd::print(2, "s %f, a %f", odometry.s, odometry.a);
        pros::lcd::print(3, "arm current %d, arm target %d, arm lock %d", arm.current_position(), arm.target_position,
                         arm.triggered_lock);

        auto a = master.get_analog(ANALOG_LEFT_Y),
                b = master.get_analog(ANALOG_RIGHT_X);
        auto position_left_current = chassis.left_position(),
                position_right_current = chassis.right_position();

        chassis.move(a, b);

        odometry += wheels_to_odometry(position_left_current - position_left_last,
                                       position_right_current - position_right_last,
                                       chassis_config);
        master.print(0, 5, "d_left %f, d_right %f",
                     position_left_current - position_left_last,
                     position_right_current - position_right_last);

        if (master.get_digital(DIGITAL_L2))
            arm.lift();
        else if (master.get_digital(DIGITAL_L1))
            arm.down();
        else
            arm.stop_and_lock();


        if (master.get_digital(DIGITAL_R2)) {
            collector.collect();
        } else if (master.get_digital(DIGITAL_R1)) {
            collector.spit();
        } else {
            collector.stop();
        }

        position_left_last = position_left_current;
        position_right_last = position_right_current;

        pros::delay(20);
    }
#pragma clang diagnostic pop
}

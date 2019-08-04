#include "main.h"
#include <functional>
#include <vector>
#include <algorithm>

template<class _device_t, size_t count>
struct device_bundle {
    using device_t=_device_t;
    device_t devices[count];

    void for_each(const std::function<void(device_t &)> block) {
        for (size_t i = 0; i < count; ++i) {
            block(devices[i]);
        }
    }

    template<class result_t>
    std::vector<result_t> read(const std::function<result_t(const device_t &)> &block) {
        std::vector<result_t> result(count);
        std::transform(devices, devices + count, result.begin(), block());
        return result;
    }
};

#define FOREACH(GROUP, OPERATION) \
(GROUP).for_each([=](typename decltype(GROUP)::device_t &it) { OPERATION })

template<size_t count>
using motor_bundle = device_bundle<pros::Motor, count>;

class Chassis {

private:
    motor_bundle<2> left{pros::Motor(5), pros::Motor(10)};
    motor_bundle<2> right{pros::Motor(1), pros::Motor(6)};

public:
    Chassis() {
        FOREACH(left,
                it.set_brake_mode(MOTOR_BRAKE_HOLD);
                        it.set_gearing(MOTOR_GEARSET_18);
        );
        FOREACH(right,
                it.set_brake_mode(MOTOR_BRAKE_HOLD);
                        it.set_gearing(MOTOR_GEARSET_18);
                        it.set_reversed(true);
        );
    }

    void move(const int32_t forward, const int32_t turn) {
        FOREACH(left, it.move(forward + turn););
        FOREACH(right, it.move(forward - turn););
    }

    void stop() {
        move(0, 0);
    }

};

class Arm {
private:
    motor_bundle<2> motors{pros::Motor(15, true), pros::Motor(11)};

public:

    Arm() {
        FOREACH(motors, it.set_gearing(MOTOR_GEARSET_18););
    }

    //TODO
    //Position close loop

};

class Claw {
private:
    motor_bundle<2> motors{pros::Motor(20, true), pros::Motor(16)};


public:
    Claw() {
        FOREACH(motors, it.set_gearing(MOTOR_GEARSET_36););
    }

    void clam() {
        FOREACH(motors, it.move(127););
    }

    void release() {
        FOREACH(motors, it.move(-127););
    }

    void stop() {
        FOREACH(motors, it.move(0););
    }


};
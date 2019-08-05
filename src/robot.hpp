#ifndef VEX_ROBOT_HPP
#define VEX_ROBOT_HPP

#include "main.h"
#include <functional>
#include <vector>
#include <algorithm>
#include <numeric>

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
(GROUP).for_each([=](typename decltype(GROUP)::device_t &it)  OPERATION )

#define MAP(TYPE, GROUP, OPERATION) \
(GROUP).read<TYPE>([=](const typename decltype(GROUP)::device_t &it) OPERATION)

template<size_t count>
using motor_bundle = device_bundle<pros::Motor, count>;

template<typename ... fold_t>
auto sum(fold_t ... ele) {
    return (ele +...);
}

class Chassis {

private:
    motor_bundle<2> left{pros::Motor(5), pros::Motor(10)};
    motor_bundle<2> right{pros::Motor(1), pros::Motor(6)};

public:
    Chassis() {
        FOREACH(left,
                {
                    it.set_brake_mode(MOTOR_BRAKE_HOLD);
                }
        );
        FOREACH(right,
                {
                    it.set_brake_mode(MOTOR_BRAKE_HOLD);
                }
        );
    }

    void move(const int32_t forward, const int32_t turn) {
        FOREACH(left, { it.move(forward + turn); });
        FOREACH(right, { it.move(forward - turn); });
    }

    void stop() {
        move(0, 0);
    }

};

class Arm {
private:
    motor_bundle<2> motors{pros::Motor(15), pros::Motor(11)};

public:

    Arm() = default;
    //TODO
    //Position close loop

    int32_t current_position() {
        auto v = MAP(int32_t, motors, { return it.get_position(); });
        return std::accumulate(v.begin(), v.end(), 0);
    }

};

class Claw {
private:
    motor_bundle<2> motors{pros::Motor(20), pros::Motor(16)};


public:
    Claw() = default;

    void clam() {
        FOREACH(motors, { it.move(127); });
    }

    void release() {
        FOREACH(motors, { it.move(-127); });
    }

    void stop() {
        FOREACH(motors, { it.move(0); });
    }


};

#endif
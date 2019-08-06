#ifndef VEX_ROBOT_HPP
#define VEX_ROBOT_HPP

#include "main.h"
#include <functional>
#include <vector>
#include <algorithm>
#include <numeric>

template<class _device_t, size_t count>
struct device_bundle_t {
    using device_t=_device_t;
    device_t devices[count];
    
    void for_each(const std::function<void(device_t &)> &block) {
        for (size_t i = 0; i < count; ++i)
            block(devices[i]);
    }
    
    template<class result_t>
    std::vector<result_t> read(const std::function<result_t(const device_t &)> &block) const {
        std::vector<result_t> result(count);
        std::transform(devices, devices + count, result.begin(), block);
        return result;
    }
    
    template<class result_t>
    result_t average(const std::function<result_t(const device_t &)> &block) const {
        return std::accumulate(devices, devices + count, result_t{}, block) / count;
    }
};

#define FOREACH(GROUP, OPERATION) \
(GROUP).for_each([=](typename decltype(GROUP)::device_t &it)  OPERATION )

#define MAP(TYPE, GROUP, OPERATION) \
(GROUP).read<TYPE>([=](const typename decltype(GROUP)::device_t &it) OPERATION)

#define AVERAGE(TYPE, GROUP, OPERATION) \
(GROUP).average<TYPE>([=](const typename decltype(GROUP)::device_t &it) OPERATION)

template<size_t count>
using motor_bundle = device_bundle_t<pros::Motor, count>;

template<typename ... fold_t>
auto sum(fold_t ... ele) {
    return (ele +...);
}

/**
 * 底盘模型
 */
struct chassis_config_t {
    double width,        // 轮距
           left_radius,  // 左轮半径
           right_radius; // 右轮半径
};

enum class odometry_type : uint8_t { state, delta };

template<odometry_type type = odometry_type::state>
struct odometry_t;

/** 里程计增量 */
template<>
struct odometry_t<odometry_type::delta> {
    double s, a, x, y, theta;
};

/** 里程计状态 */
template<>
struct odometry_t<odometry_type::state> {
    double s, a, x, y, theta;
    
    odometry_t &operator+=(
        const odometry_t<odometry_type::delta> &delta
    ) {
        s += delta.s;
        a += delta.a;
        
        const auto sin = std::sin(theta),
                   cos = std::cos(theta);
        theta += delta.theta;
        
        x += delta.x * cos - delta.y * sin;
        y += delta.x * sin + delta.y * cos;
        return *this;
    }
    
    odometry_t &operator-=(
        const odometry_t<odometry_type::delta> &delta
    ) {
        s -= delta.s;
        a -= delta.a;
        
        theta -= delta.theta;
        const auto sin = std::sin(theta),
                   cos = std::cos(theta);
        
        x -= delta.x * cos - delta.y * sin;
        y -= delta.x * sin + delta.y * cos;
        return *this;
    }
    
    odometry_t operator+(
        const odometry_t<odometry_type::delta> &delta
    ) const {
        auto temp = *this;
        return temp += delta;
    }
    
    odometry_t operator-(
        const odometry_t<odometry_type::delta> &delta
    ) const {
        auto temp = *this;
        return temp -= delta;
    }
    
    odometry_t<odometry_type::delta> operator-(
        const odometry_t<odometry_type::state> &mark
    ) const {
        const auto sin = std::sin(-mark.theta),
                   cos = std::cos(-mark.theta),
                   dx  = x - mark.x,
                   dy  = y - mark.y;
        
        return {s - mark.s,
                a - mark.a,
                dx * cos - dy * sin,
                dx * sin + dy * cos,
                theta - mark.theta};
    }
};

class chassis_t {
    motor_bundle<2>
        left{pros::Motor(5), pros::Motor(10)},
        right{pros::Motor(1), pros::Motor(6)};

public:
    chassis_t() {
        FOREACH(left, {
            it.set_brake_mode(MOTOR_BRAKE_HOLD);
        });
        FOREACH(right, {
            it.set_brake_mode(MOTOR_BRAKE_HOLD);
        });
    }
    
    double left_position() {
        return AVERAGE(double, left, { return it.get_position(); });
    }
    
    double right_position() {
        return AVERAGE(double, right, { return it.get_position(); });
    }
    
    void move(const int32_t forward, const int32_t turn) {
        FOREACH(left, { it.move(forward + turn); });
        FOREACH(right, { it.move(forward - turn); });
    }
};

/**
 * 推算里程增量
 * @param left   左轮转角增量
 * @param right  右轮转角增量
 * @param config 底盘结构参数
 * @return 里程增量
 */
odometry_t<odometry_type::delta>
wheels_to_odometry(
    double left,
    double right,
    const chassis_config_t &config) {
    const auto l = config.left_radius * left,
               r = config.right_radius * right,
               s = (r + l) / 2,
               a = (r - l) / config.width;
    double     x, y;
    if (std::abs(a) < std::numeric_limits<double>::epsilon()) {
        x = s;
        y = 0;
    } else {
        auto _r = s / a;
        x = _r * std::sin(a);
        y = _r * (1 - std::cos(a));
    }
    return {std::abs(s), std::abs(a), x, y, a};
}

class arm_t {
    motor_bundle<2> motors{pros::Motor(15), pros::Motor(11)};

public:
    arm_t() = default;
    //TODO
    //Position close loop
    
    int32_t current_position() {
        return AVERAGE(double, motors, { return it.get_position(); });
    }
};

class collector_t {
    motor_bundle<2> motors{pros::Motor(20), pros::Motor(16)};

public:
    void collect() { FOREACH(motors, { it.move(127); }); }
    
    void spit() { FOREACH(motors, { it.move(-127); }); }
    
    void stop() { FOREACH(motors, { it.move(0); }); }
};

#endif

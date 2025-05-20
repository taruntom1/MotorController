#pragma once

#include <stdint.h>
#include <stdfloat>
#include <vector>
#include <cstring>
#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using timestamp_t = uint64_t;
using encoder_ticks_t = int;
using encoder_tickrate_t = float;
using angle_t = float;
using angularvelocity_t = float;
using pwmvalue_t = float;
using pin_connection_t = uint8_t;
using frequency_t = uint16_t;

// Internal little-endian helpers
namespace detail
{
    template <typename T>
    inline void appendLE(std::vector<uint8_t> &buf, const T &val)
    {
        const uint8_t *p = reinterpret_cast<const uint8_t *>(&val);
        buf.insert(buf.end(), p, p + sizeof(T));
    }
    template <typename T>
    inline T readLE(const std::vector<uint8_t> &buf, size_t &offset)
    {
        T val;
        memcpy(&val, buf.data() + offset, sizeof(T));
        offset += sizeof(T);
        return val;
    }
}

struct odo_broadcast_flags_t
{
    bool angle;
    bool speed;
    bool pwm_value;

    constexpr static size_t size = sizeof(angle) + sizeof(speed) +
                                   sizeof(pwm_value);

    std::vector<uint8_t> to_bytes() const
    {
        std::vector<uint8_t> buf;
        buf.reserve(size);
        detail::appendLE(buf, angle);
        detail::appendLE(buf, speed);
        detail::appendLE(buf, pwm_value);
        return buf;
    }
    void from_bytes(const std::vector<uint8_t> &buf, size_t &offset)
    {
        angle = detail::readLE<bool>(buf, offset);
        speed = detail::readLE<bool>(buf, offset);
        pwm_value = detail::readLE<bool>(buf, offset);
    }

    bool operator==(const odo_broadcast_flags_t &other)
    {
        return (angle == other.angle) && (speed == other.speed) &&
               (pwm_value == other.pwm_value);
    }
    bool operator!=(const odo_broadcast_flags_t &other)
    {
        return !operator==(other);
    }

    odo_broadcast_flags_t &operator|=(const odo_broadcast_flags_t &rhs)
    {
        angle |= rhs.angle;
        speed |= rhs.speed;
        pwm_value |= rhs.pwm_value;
        return *this;
    }
};

struct pid_constants_t
{
    float p;
    float i;
    float d;

    constexpr static size_t size = sizeof(p) + sizeof(i) + sizeof(d);

    std::vector<uint8_t> to_bytes() const
    {
        std::vector<uint8_t> buf;
        buf.reserve(size);
        detail::appendLE(buf, p);
        detail::appendLE(buf, i);
        detail::appendLE(buf, d);
        return buf;
    }
    void from_bytes(const std::vector<uint8_t> &buf, size_t &offset)
    {
        p = detail::readLE<float>(buf, offset);
        i = detail::readLE<float>(buf, offset);
        d = detail::readLE<float>(buf, offset);
    }
};

struct limits_pwm_t
{
    int8_t min = 0;
    int8_t max = 0;

    constexpr static size_t size = sizeof(min) + sizeof(max);

    std::vector<uint8_t> to_bytes() const
    {
        std::vector<uint8_t> buf;
        buf.reserve(size);
        detail::appendLE(buf, min);
        detail::appendLE(buf, max);
        return buf;
    }
    void from_bytes(const std::vector<uint8_t> &buf, size_t &offset)
    {
        min = detail::readLE<int8_t>(buf, offset);
        max = detail::readLE<int8_t>(buf, offset);
    }
};

struct connections_wheel_t
{
    pin_connection_t dir;
    pin_connection_t pwm;
    pin_connection_t enc_a;
    pin_connection_t enc_b;

    constexpr static size_t size = sizeof(pin_connection_t) * 4;

    std::vector<uint8_t> to_bytes() const
    {
        std::vector<uint8_t> buf;
        buf.reserve(size);
        detail::appendLE(buf, dir);
        detail::appendLE(buf, pwm);
        detail::appendLE(buf, enc_a);
        detail::appendLE(buf, enc_b);
        return buf;
    }
    void from_bytes(const std::vector<uint8_t> &buf, size_t &offset)
    {
        dir = detail::readLE<pin_connection_t>(buf, offset);
        pwm = detail::readLE<pin_connection_t>(buf, offset);
        enc_a = detail::readLE<pin_connection_t>(buf, offset);
        enc_b = detail::readLE<pin_connection_t>(buf, offset);
    }
};

struct timestamped_angle_t
{
    timestamp_t timestamp;
    angle_t angle;

    constexpr static size_t size = sizeof(timestamp_t) + sizeof(angle_t);
    std::vector<uint8_t> to_bytes() const
    {
        std::vector<uint8_t> buf;
        buf.reserve(size);
        detail::appendLE(buf, timestamp);
        detail::appendLE(buf, angle);
        return buf;
    }
    void from_bytes(const std::vector<uint8_t> &buf, size_t &offset)
    {
        timestamp = detail::readLE<timestamp_t>(buf, offset);
        angle = detail::readLE<angle_t>(buf, offset);
    }
};

struct odometry_t
{
    angle_t angle{0};
    angularvelocity_t angular_velocity{0};
    pwmvalue_t pwm_value{0};

    constexpr static size_t size = sizeof(angle_t) + sizeof(angularvelocity_t) + sizeof(pwmvalue_t);

    std::vector<uint8_t> to_bytes() const
    {
        std::vector<uint8_t> buf;
        buf.reserve(size);
        detail::appendLE(buf, angle);
        detail::appendLE(buf, angular_velocity);
        detail::appendLE(buf, pwm_value);
        return buf;
    }

    void from_bytes(const std::vector<uint8_t> &buf, size_t &offset)
    {
        angle = detail::readLE<angle_t>(buf, offset);
        angular_velocity = detail::readLE<angularvelocity_t>(buf, offset);
        pwm_value = detail::readLE<pwmvalue_t>(buf, offset);
    }
};

struct wheel_update_frequencies_t
{
    frequency_t pwm = 50;
    frequency_t angle_pid = 50;
    frequency_t speed_pid = 50;

    constexpr static size_t size = sizeof(frequency_t) * 3;

    std::vector<uint8_t> to_bytes() const
    {
        std::vector<uint8_t> buf;
        buf.reserve(size);
        detail::appendLE(buf, pwm);
        detail::appendLE(buf, angle_pid);
        detail::appendLE(buf, speed_pid);
        return buf;
    }
    void from_bytes(const std::vector<uint8_t> &buf, size_t &offset)
    {
        pwm = detail::readLE<frequency_t>(buf, offset);
        angle_pid = detail::readLE<frequency_t>(buf, offset);
        speed_pid = detail::readLE<frequency_t>(buf, offset);
    }
};

enum class ControlMode : uint8_t
{
    OFF,
    PWM_DIRECT_CONTROL,
    POSITION_CONTROL,
    SPEED_CONTROL
};

inline std::vector<uint8_t> to_bytes(ControlMode m)
{
    return std::vector<uint8_t>{static_cast<uint8_t>(m)};
}
inline void from_bytes(ControlMode &m, const std::vector<uint8_t> &buf, size_t &offset)
{
    m = static_cast<ControlMode>(detail::readLE<uint8_t>(buf, offset));
}

struct wheel_data_t
{
    uint8_t motor_id;
    ControlMode control_mode;
    pid_constants_t anglePIDConstants;
    pid_constants_t speedPIDConstants;
    connections_wheel_t motorConnections;
    odo_broadcast_flags_t odoBroadcastStatus;
    float radians_per_tick = 1;

    constexpr static size_t size = sizeof(motor_id) + sizeof(control_mode) +
                                   pid_constants_t::size * 2 + connections_wheel_t::size +
                                   odo_broadcast_flags_t::size + sizeof(radians_per_tick);

    std::vector<uint8_t> to_bytes() const
    {
        std::vector<uint8_t> buf;
        buf.reserve(size);
        detail::appendLE(buf, motor_id);
        auto cm = ::to_bytes(control_mode);
        buf.insert(buf.end(), cm.begin(), cm.end());
        auto a = anglePIDConstants.to_bytes();
        buf.insert(buf.end(), a.begin(), a.end());
        auto s = speedPIDConstants.to_bytes();
        buf.insert(buf.end(), s.begin(), s.end());
        auto c = motorConnections.to_bytes();
        buf.insert(buf.end(), c.begin(), c.end());
        auto f = odoBroadcastStatus.to_bytes();
        buf.insert(buf.end(), f.begin(), f.end());
        detail::appendLE(buf, radians_per_tick);
        return buf;
    }

    void from_bytes(const std::vector<uint8_t> &buf, size_t &offset)
    {
        motor_id = detail::readLE<uint8_t>(buf, offset);
        ::from_bytes(control_mode, buf, offset);
        anglePIDConstants.from_bytes(buf, offset);
        speedPIDConstants.from_bytes(buf, offset);
        motorConnections.from_bytes(buf, offset);
        odoBroadcastStatus.from_bytes(buf, offset);
        radians_per_tick = detail::readLE<float>(buf, offset);
    }
};

struct update_frequencies_t
{
    frequency_t control_run_frequency;
    frequency_t odoBroadcastFrequency;

    constexpr static size_t size = sizeof(frequency_t) * 2;

    std::vector<uint8_t> to_bytes() const
    {
        std::vector<uint8_t> buf;
        buf.reserve(size);
        detail::appendLE(buf, control_run_frequency);
        detail::appendLE(buf, odoBroadcastFrequency);
        return buf;
    }
    void from_bytes(const std::vector<uint8_t> &buf, size_t &offset)
    {
        control_run_frequency = detail::readLE<frequency_t>(buf, offset);
        odoBroadcastFrequency = detail::readLE<frequency_t>(buf, offset);
    }
};

struct controller_properties_t
{
    uint8_t numMotors;
    update_frequencies_t updateFrequencies;

    constexpr static size_t size = sizeof(uint8_t) +
                                   update_frequencies_t::size;

    std::vector<uint8_t> to_bytes() const
    {
        std::vector<uint8_t> buf;
        detail::appendLE(buf, numMotors);
        auto uf = updateFrequencies.to_bytes();
        buf.insert(buf.end(), uf.begin(), uf.end());
        return buf;
    }
    void from_bytes(const std::vector<uint8_t> &buf, size_t &offset)
    {
        numMotors = detail::readLE<uint8_t>(buf, offset);
        updateFrequencies.from_bytes(buf, offset);
    }
};

struct controller_data_t
{
    std::vector<wheel_data_t> wheelData;
    controller_properties_t controllerProperties;

    std::vector<uint8_t> to_bytes() const
    {
        std::vector<uint8_t> buf;
        uint8_t n = static_cast<uint8_t>(wheelData.size());
        detail::appendLE(buf, n);
        for (auto &w : wheelData)
        {
            auto wb = w.to_bytes();
            buf.insert(buf.end(), wb.begin(), wb.end());
        }
        auto cp = controllerProperties.to_bytes();
        buf.insert(buf.end(), cp.begin(), cp.end());
        return buf;
    }
    void from_bytes(const std::vector<uint8_t> &buf, size_t &offset)
    {
        uint8_t n = detail::readLE<uint8_t>(buf, offset);
        wheelData.clear();
        wheelData.resize(n);
        for (uint8_t i = 0; i < n; ++i)
        {
            wheelData.at(i).from_bytes(buf, offset);
        }
        controllerProperties.from_bytes(buf, offset);
    }
};

enum class PIDType : uint8_t
{
    POSITION = 0,
    VELOCITY = 1
};

struct WheelTaskHandles
{
    TaskHandle_t wheel_run_task_handle;
    TaskHandle_t control_task_handle;
    TaskHandle_t odo_broadcast;
};

struct InterfaceTaskHandles
{
    TaskHandle_t Run;
};

struct TaskHandles
{
    TaskHandle_t main_task_handle;
    TaskHandle_t wheel_manager;
    InterfaceTaskHandles interface_task_handles;
    std::vector<WheelTaskHandles> wheel_task_handles;
};

/** @defgroup notifications Notifications
 *  @brief Defines notification messages for the motor controller.
 *  @{
 */

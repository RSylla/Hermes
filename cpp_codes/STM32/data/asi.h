#pragma once

#include "canopen.h"
#include <cstdint>
#include <memory> // For smart pointers

namespace asi
{

struct motor_state
{
    double wheel_rpm = 0.0;
    double battery_voltage = 0.0;
    double controller_temperature = 0.0;
    uint32_t last_time_received = 0U;
    int32_t ticks = 0;         // Accumulated tick count (int32_t)
    uint16_t last_ticks = 0U;  // Last tick count received from the motor (uint16_t)
    bool first_tick_received = false;
};

struct emergency_state
{
    uint16_t motor_id = 0U;
    uint8_t error_register = 0U;
    uint8_t faults_2 = 0U;
    uint16_t faults_1 = 0U;
    uint16_t warnings = 0U;
    bool motor_temp_warning = false;
};

// Use smart pointers for motor states
extern std::unique_ptr<motor_state> left_motor;
extern std::unique_ptr<motor_state> right_motor;
extern emergency_state error;
extern bool rx_timeout;

inline constexpr uint32_t rx_timeout_value = 10000U;

// Function declarations
void receive_frame(const canopen::can_frame& frame);
void update_motor_commands(double throttle_left, double throttle_right, double regen_left, double regen_right, bool reverse_left, bool reverse_right);
void enter_operational_mode();

} // namespace asi

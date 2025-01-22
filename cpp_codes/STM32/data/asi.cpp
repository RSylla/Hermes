// asi.cpp

#include "asi.h"
#include <Arduino.h> // For 'millis()'
#include <cstring>   // For 'memcpy'

namespace asi
{

std::unique_ptr<motor_state> left_motor = std::make_unique<motor_state>();
std::unique_ptr<motor_state> right_motor = std::make_unique<motor_state>();
emergency_state error{};
bool rx_timeout = false;

namespace
{

canopen::node_id get_node_id(const canopen::can_frame& frame)
{
    return static_cast<canopen::node_id>(frame.id & 0x7FU);
}

void read_pdo_tx_1(const canopen::can_frame& frame, std::unique_ptr<motor_state>& motor)
{
    motor->last_time_received = millis();

    canopen::pdo_tx_1 read_val{};
    memcpy(&read_val, frame.data, sizeof(canopen::pdo_tx_1));

    motor->wheel_rpm = static_cast<double>(read_val.speed_rpm) / 16.0;
    motor->wheel_rpm = (std::abs(motor->wheel_rpm) > 1500.0) ? 0.0 : motor->wheel_rpm;

    uint16_t current_ticks = read_val.ticks;

    if (motor->first_tick_received)
    {
        uint16_t last_ticks = motor->last_ticks;
        int16_t tick_difference = static_cast<int16_t>(current_ticks - last_ticks);

        int direction = (read_val.speed_rpm >= 0) ? 1 : -1;
        motor->ticks += direction * static_cast<int32_t>(tick_difference);
    }
    else
    {
        motor->ticks = 0;
        motor->first_tick_received = true;
    }

    motor->last_ticks = current_ticks;

    motor->battery_voltage = static_cast<double>(read_val.battery_voltage) / 32.0;
    motor->controller_temperature = static_cast<double>(read_val.controller_temperature);
}


void read_emergency(const canopen::can_frame& frame)
{
    canopen::emergency_state read_val{};
    memcpy(&read_val, frame.data, sizeof(canopen::emergency_state));

    error.motor_id = static_cast<uint16_t>(get_node_id(frame));
    error.error_register = read_val.error_register;
    error.faults_2 = read_val.faults_2;
    error.faults_1 = read_val.faults_1;
    error.warnings = read_val.warnings;
}

bool check_estops()
{
    return rx_timeout;
}

std::pair<canopen::pdo_rx_1, canopen::pdo_rx_1> create_pdo_rx_1(
    uint16_t throttle_voltage_left, uint16_t throttle_voltage_right,
    uint16_t regen_voltage_left, uint16_t regen_voltage_right,
    bool reverse_left, bool reverse_right)
{
    static uint8_t rolling_counter = 0U;

    canopen::pdo_rx_1 pdo_left{};
    canopen::pdo_rx_1 pdo_right{};

    constexpr uint16_t regen_threshold = 10U;

    // Left motor PDO
    pdo_left.throttle_voltage = throttle_voltage_left;
    pdo_left.regen_voltage = regen_voltage_left;
    pdo_left.remote_commands = canopen::remote_digital_commands{
        .cut_out = false,
        .headlight = false,
        .runlight = false,
        .brakelight = false,
        .chrgdisable = true,
        .alt_speed = false,
        .alt_pwr = false,
        .regen1 = (regen_voltage_left > regen_threshold),
        .regen2 = (regen_voltage_left > regen_threshold),
        .HDQ = false,
        .disable_analog_regen = false,
        .disable_reverse_cadence_regen = false,
        .enable_remote_braking_torque = false,
        .CAN_remote_fault = false,
        .disable_requested_function = false,
        .reverse = reverse_left
    };
    pdo_left.rolling_counter = static_cast<uint16_t>(rolling_counter) << 8;

    // Right motor PDO
    pdo_right.throttle_voltage = throttle_voltage_right;
    pdo_right.regen_voltage = regen_voltage_right;
    pdo_right.remote_commands = canopen::remote_digital_commands{
        .cut_out = false,
        .headlight = false,
        .runlight = false,
        .brakelight = false,
        .chrgdisable = true,
        .alt_speed = false,
        .alt_pwr = false,
        .regen1 = (regen_voltage_right > regen_threshold),
        .regen2 = (regen_voltage_right > regen_threshold),
        .HDQ = false,
        .disable_analog_regen = false,
        .disable_reverse_cadence_regen = false,
        .enable_remote_braking_torque = false,
        .CAN_remote_fault = false,
        .disable_requested_function = false,
        .reverse = reverse_right
    };
    pdo_right.rolling_counter = static_cast<uint16_t>(rolling_counter) << 8;

    ++rolling_counter;

    return {pdo_left, pdo_right};
}

int16_t convert_throttle(double throttle)
{
    // Convert throttle value from 0.0 - 1.0 to -4096 to +4096 (assuming 12-bit resolution for bidirectional)
    return static_cast<int16_t>(throttle * 4096.0 * 5.0); // Adjust scaling as needed
}

int16_t convert_regen(double regen)
{
    // Convert regen value from 0.0 - 1.0 to 0 - 4096
    return static_cast<int16_t>(regen * 4096.0);
}

} // unnamed namespace

void receive_frame(const canopen::can_frame& frame)
{
    const uint32_t function_id_mask = frame.id & 0x780U;
    const canopen::node_id node = get_node_id(frame);

    switch (function_id_mask)
    {
        case static_cast<uint32_t>(canopen::function_id::pdo_tx_1):
            if (node == canopen::node_id::left_motor)
            {
                read_pdo_tx_1(frame, left_motor);
            }
            else if (node == canopen::node_id::right_motor)
            {
                read_pdo_tx_1(frame, right_motor);
            }
            break;
        case static_cast<uint32_t>(canopen::function_id::emergency):
            read_emergency(frame);
            break;
        default:
            // Handle other cases if necessary
            break;
    }
}

void update_motor_commands(double throttle_left, double throttle_right, double regen_left, double regen_right, bool reverse_left, bool reverse_right)
{
    if (check_estops())
    {
        throttle_left = 0.0;
        throttle_right = 0.0;
        regen_left = 1.0;
        regen_right = 1.0;
    }

    auto [pdo_left, pdo_right] = create_pdo_rx_1(
        convert_throttle(throttle_left), convert_throttle(throttle_right),
        convert_regen(regen_left), convert_regen(regen_right),
        reverse_left, reverse_right
    );

    const canopen::can_frame left_frame_rpdo1 = canopen::pdo1_to_can_frame(canopen::node_id::left_motor, pdo_left);
    const canopen::can_frame right_frame_rpdo1 = canopen::pdo1_to_can_frame(canopen::node_id::right_motor, pdo_right);

    canopen::send_pdo_frame(left_frame_rpdo1);
    canopen::send_pdo_frame(right_frame_rpdo1);

    constexpr double warning_temp = 100.0;
    error.motor_temp_warning = (left_motor->controller_temperature >= warning_temp || right_motor->controller_temperature >= warning_temp);
}

void enter_operational_mode()
{
    canopen::send_nmt_command(canopen::node_id::left_motor, canopen::nmt::commands::enter_op_state);
    canopen::send_nmt_command(canopen::node_id::right_motor, canopen::nmt::commands::enter_op_state);
}

} // namespace asi

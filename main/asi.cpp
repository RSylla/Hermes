#include "asi.h"

namespace asi {

motor_state left_motor;
motor_state right_motor;
emergency_state error;
bool rx_timeout{};

inline canopen::node_id get_node_id(canopen::can_frame frame) {
    return static_cast<canopen::node_id>(frame.id & 0b01111111);
}

void read_pdo_tx_1(canopen::can_frame frame, motor_state* motor) {
    uint32_t current_time = millis();
    motor->last_time_received = current_time;

    canopen::pdo_tx_1 read_val{};
    std::copy(frame.data, frame.data + 8, reinterpret_cast<uint8_t *>(&read_val));

    switch (get_node_id(frame)) {
        case canopen::node_id::left_motor:
            // Process left motor data
            break;
        case canopen::node_id::right_motor:
            // Process right motor data
            break;
        default:
            return;
    }

    if (read_val.speed_rpm >= -32768 && read_val.speed_rpm <= 32767 &&
        read_val.battery_voltage >= 0 && read_val.battery_voltage <= 65535 &&
        read_val.controller_temperature >= 0 && read_val.controller_temperature <= 65535) {
        motor->wheel_rpm = read_val.speed_rpm / 16.0;
        motor->wheel_rpm = (motor->wheel_rpm > 1500.0) ? 0.0 : motor->wheel_rpm;
        
        // Check if it's the first tick received
        if (motor->first_tick_received) {
            // Increment the ticks from the previous value
            motor->ticks += read_val.ticks - motor->last_ticks;
        } else {
            // Set the first tick value to 0 and mark it as received
            motor->ticks = 0;
            motor->first_tick_received = true;
        }
        
        // Store the current tick value for the next iteration
        motor->last_ticks = read_val.ticks;
        
        motor->battery_voltage = read_val.battery_voltage / 32.0;
        motor->controller_temperature = read_val.controller_temperature;
    } else {
        // Invalid data received, reset the motor state
        motor->ticks = 0.0;
        motor->wheel_rpm = 0.0;
        motor->battery_voltage = 0.0;
        motor->first_tick_received = false;
    }
}

void read_emergency(canopen::can_frame frame) {
    canopen::emergency_state read_val{};
    std::copy(frame.data, frame.data + 8, reinterpret_cast<uint8_t *>(&read_val));

    error.motor_id = static_cast<uint16_t>(get_node_id(frame));
    error.error_register = read_val.error_register;
    error.faults_2 = read_val.faults_2;
    error.faults_1 = read_val.faults_1;
    error.warnings = read_val.warnings;
}

void receive_frame(canopen::can_frame frame) {
    switch (frame.id & 0b11110000000) {
        case static_cast<uint32_t>(canopen::function_id::nmt_node_control):
            break;
        case static_cast<uint32_t>(canopen::function_id::emergency):
            read_emergency(frame);
            break;
        case static_cast<uint32_t>(canopen::function_id::timestamp):
            break;
        case static_cast<uint32_t>(canopen::function_id::pdo_tx_1):
            if (get_node_id(frame) == canopen::node_id::left_motor) {
                read_pdo_tx_1(frame, &left_motor);
            } else if (get_node_id(frame) == canopen::node_id::right_motor) {
                read_pdo_tx_1(frame, &right_motor);
            }
            break;
        case static_cast<uint32_t>(canopen::function_id::pdo_tx_2):
            break;
        case static_cast<uint32_t>(canopen::function_id::pdo_tx_3):
            break;
        case static_cast<uint32_t>(canopen::function_id::pdo_tx_4):
            break;
        case static_cast<uint32_t>(canopen::function_id::sdo_tx):
            if (get_node_id(frame) == canopen::node_id::left_motor) {
                read_pdo_tx_1(frame, &left_motor);
            } else if (get_node_id(frame) == canopen::node_id::right_motor) {
                read_pdo_tx_1(frame, &right_motor);
            }
            break;
        case static_cast<uint32_t>(canopen::function_id::heartbeat):
            break;
        default:
            break;
    }
}

bool check_estops() {
    return rx_timeout;
}

std::pair<canopen::pdo_rx_1, canopen::pdo_rx_1> create_pdo_rx_1(uint16_t throttle_voltage_left, uint16_t throttle_voltage_right,
                                                                uint16_t regen_voltage_left, uint16_t regen_voltage_right,
                                                                bool reverse_left, bool reverse_right) {
    static uint8_t rolling_counter{ 0 };
    canopen::pdo_rx_1 pdo_left, pdo_right;

    pdo_left.throttle_voltage = throttle_voltage_left;
    pdo_left.regen_voltage = regen_voltage_left;
    constexpr uint16_t regen_threshold{ 10 };
    bool regen1_left{ regen_voltage_left > regen_threshold };
    bool regen2_left{ regen_voltage_left > regen_threshold };
    pdo_left.remote_commands = canopen::remote_digital_commands{ .regen1{ regen1_left }, .regen2{ regen2_left }, .reverse{ reverse_left } };
    pdo_left.rolling_counter = static_cast<uint16_t>(rolling_counter) << 8;

    pdo_right.throttle_voltage = throttle_voltage_right;
    pdo_right.regen_voltage = regen_voltage_right;
    bool regen1_right{ regen_voltage_right > regen_threshold };
    bool regen2_right{ regen_voltage_right > regen_threshold };
    pdo_right.remote_commands = canopen::remote_digital_commands{ .regen1{ regen1_right }, .regen2{ regen2_right }, .reverse{ reverse_right } };
    pdo_right.rolling_counter = static_cast<uint16_t>(rolling_counter) << 8;

    rolling_counter++;

    return std::make_pair(pdo_left, pdo_right);
}

void send_control_commands(double throttle_left, double throttle_right, double regen_left, double regen_right, bool reverse_left, bool reverse_right) {
    if (check_estops()) {
        throttle_left = 0;
        throttle_right = 0;
        regen_left = 1.0;
        regen_right = 1.0;
    }

    auto [pdo_left, pdo_right] = create_pdo_rx_1(convert_throttle(throttle_left), convert_throttle(throttle_right),
                                                convert_regen(regen_left), convert_regen(regen_right),
                                                reverse_left, reverse_right);

    canopen::can_frame left_frame_rpdo1 = canopen::pdo1_to_can_frame(canopen::node_id::left_motor, pdo_left);
    canopen::can_frame right_frame_rpdo1 = canopen::pdo1_to_can_frame(canopen::node_id::right_motor, pdo_right);

    canopen::send_pdo_frame(left_frame_rpdo1);
    canopen::send_pdo_frame(right_frame_rpdo1);

    double warning_temp = 100.0;
    error.motor_temp_warning = (left_motor.motor_temperature >= warning_temp || right_motor.motor_temperature >= warning_temp);
}

void update_motor_commands(double throttle_left, double throttle_right, double regen_left, double regen_right, bool reverse_left, bool reverse_right) {
    send_control_commands(throttle_left, throttle_right, regen_left, regen_right, reverse_left, reverse_right);
}

void enter_operational_mode() {
    canopen::send_nmt_command(canopen::node_id::left_motor, canopen::nmt::commands::enter_op_state);
    canopen::send_nmt_command(canopen::node_id::right_motor, canopen::nmt::commands::enter_op_state);
}

inline uint16_t convert_throttle(double throttle) {
    // Convert throttle value from 0.0 - 1.0 to 0 - 4096
    return static_cast<uint16_t>(throttle * 4096 * 5);
}

inline uint16_t convert_regen(double regen) {
    // Convert regen value from 0.0 - 1.0 to 0 - 4096
    return static_cast<uint16_t>(regen * 4096);
}

} // namespace asi

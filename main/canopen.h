#ifndef CANOPEN_H
#define CANOPEN_H

#include <Arduino.h>
#include "STM32_CAN.h"

namespace asi {
namespace canopen {

enum class node_id {
    right_motor = 42,
    left_motor = 43,
};

struct can_frame {
    can_frame() {}
    can_frame(uint32_t id, uint8_t dlc, uint8_t databuf[8])
        : id(id), dlc(dlc) {
        std::copy(databuf, databuf + 8, data);
    }
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
};

enum class function_id {
    nmt_node_control = 0x000,
    global_failsafe_cmd = 0x001,
    emergency = 0x080,
    timestamp = 0x100,
    pdo_tx_1 = 0x180,
    pdo_rx_1 = 0x200,
    pdo_tx_2 = 0x280,
    pdo_rx_2 = 0x300,
    pdo_tx_3 = 0x380,
    pdo_rx_3 = 0x400,
    pdo_tx_4 = 0x480,
    pdo_rx_4 = 0x500,
    sdo_tx = 0x580,
    sdo_rx = 0x600,
    heartbeat = 0x700,
};

namespace nmt {
enum class commands {
    enter_op_state = 0x01,
    enter_stopped_state = 0x02,
    enter_pre_op_state = 0x80,
    reset_node = 0x81,
    reset_communication = 0x82,
};
}

namespace sdo_write {
enum class command {
    write_4_bytes = 0x23,
    write_3_bytes = 0x27,
    write_2_bytes = 0x2b,
    write_1_bytes = 0x2f,
};
}

struct [[gnu::packed]] remote_digital_commands {
    bool cut_out : 1;
    bool headlight : 1;
    bool runlight : 1;
    bool brakelight : 1;
    bool chrgdisable : 1 { true };
    bool alt_speed : 1;
    bool alt_pwr : 1;
    bool regen1 : 1;
    bool regen2 : 1;
    bool HDQ : 1;
    bool disable_analog_regen : 1;
    bool disable_reverse_cadence_regen : 1;
    bool enable_remote_braking_torque : 1;
    bool CAN_remote_fault : 1;
    bool disable_requested_function : 1;
    bool reverse : 1;
};

struct pdo_rx_1 {
    uint16_t throttle_voltage;
    remote_digital_commands remote_commands;
    uint16_t regen_voltage;
    uint16_t rolling_counter;
    bool reverse;
};

struct pdo_tx_1 {
    int16_t speed_rpm;
    uint16_t battery_voltage;
    uint32_t ticks;
    uint16_t controller_temperature;
};

struct emergency_state {
    uint16_t motor_id;
    uint8_t error_register;
    uint8_t faults_2;
    uint16_t faults_1;
    uint16_t warnings;
};

can_frame pdo1_to_can_frame(node_id node_id, pdo_rx_1 pdo_rx_1);
can_frame write_sdo_rx(node_id node_id, uint16_t index, uint8_t sub_index, uint32_t data);
void send_pdo_frame(can_frame frame);
void send_nmt_command(node_id node_id, nmt::commands cmd);

} // namespace canopen
} // namespace asi

#endif // CANOPEN_H

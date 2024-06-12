#ifndef ASI_H
#define ASI_H

#include "canopen.h"

namespace asi {

struct motor_state {
    double wheel_rpm;
    double battery_voltage;
    double controller_temperature;
    double motor_temperature;
    uint32_t last_time_received;
    uint32_t ticks;
    bool first_tick_received;
    uint32_t last_ticks;
};

struct emergency_state {
    uint16_t motor_id;
    uint8_t error_register;
    uint8_t faults_2;
    uint16_t faults_1;
    uint16_t warnings;
    bool motor_temp_warning;
};

extern motor_state left_motor;
extern motor_state right_motor;
extern emergency_state error;
extern bool rx_timeout;
constexpr uint32_t rx_timeout_value{10000};

inline canopen::node_id get_node_id(canopen::can_frame frame);

void read_pdo_tx_1(canopen::can_frame frame, motor_state* motor);
void read_emergency(canopen::can_frame frame);
void receive_frame(canopen::can_frame frame);
bool check_estops();
std::pair<canopen::pdo_rx_1, canopen::pdo_rx_1> create_pdo_rx_1(uint16_t throttle_voltage_left, uint16_t throttle_voltage_right,
                                                                uint16_t regen_voltage_left, uint16_t regen_voltage_right,
                                                                bool reverse_left, bool reverse_right);
void send_control_commands(double throttle_left, double throttle_right, double regen_left, double regen_right, bool reverse_left, bool reverse_right);
void update_motor_commands(double throttle_left, double throttle_right, double regen_left, double regen_right, bool reverse_left, bool reverse_right);
void enter_operational_mode();

inline uint16_t convert_throttle(double throttle);
inline uint16_t convert_regen(double regen);

} // namespace asi

#endif // ASI_H

#pragma once

#include <cstdint>

namespace asi
{
namespace canopen
{

enum class node_id : uint8_t
{
	right_motor = 42U,
	left_motor = 43U,
};

enum class function_id : uint32_t
{
	nmt_node_control     = 0x000U,
	global_failsafe_cmd  = 0x001U,
	emergency            = 0x080U,
	timestamp            = 0x100U,
	pdo_tx_1             = 0x180U,
	pdo_rx_1             = 0x200U,
	pdo_tx_2             = 0x280U,
	pdo_rx_2             = 0x300U,
	pdo_tx_3             = 0x380U,
	pdo_rx_3             = 0x400U,
	pdo_tx_4             = 0x480U,
	pdo_rx_4             = 0x500U,
	sdo_tx               = 0x580U,
	sdo_rx               = 0x600U,
	heartbeat            = 0x700U,
};

namespace nmt
{
enum class commands : uint8_t
{
	enter_op_state           = 0x01U,
	enter_stopped_state      = 0x02U,
	enter_pre_op_state       = 0x80U,
	reset_node               = 0x81U,
	reset_communication      = 0x82U,
};
}

// SDO write commands
namespace sdo_write
{
enum class command : uint8_t
{
	write_4_bytes = 0x23U,
	write_3_bytes = 0x27U,
	write_2_bytes = 0x2BU,
	write_1_byte  = 0x2FU,
};
}

// CAN frame structure
struct can_frame
{
	uint32_t id = 0U;
	uint8_t dlc = 0U;
	uint8_t data[8]{};
};

// Remote digital commands structure
struct __attribute__((packed)) remote_digital_commands
{
	uint16_t cut_out                       : 1;
	uint16_t headlight                     : 1;
	uint16_t runlight                      : 1;
	uint16_t brakelight                    : 1;
	uint16_t chrgdisable                   : 1;
	uint16_t alt_speed                     : 1;
	uint16_t alt_pwr                       : 1;
	uint16_t regen1                        : 1;
	uint16_t regen2                        : 1;
	uint16_t HDQ                           : 1;
	uint16_t disable_analog_regen          : 1;
	uint16_t disable_reverse_cadence_regen : 1;
	uint16_t enable_remote_braking_torque  : 1;
	uint16_t CAN_remote_fault              : 1;
	uint16_t disable_requested_function    : 1;
	uint16_t reverse                       : 1;
};

static_assert(sizeof(remote_digital_commands) == 2, "remote_digital_commands size must be 2 bytes");

struct __attribute__((packed)) pdo_rx_1
{
	uint16_t throttle_voltage = 0U;
	remote_digital_commands remote_commands{};
	uint16_t regen_voltage = 0U;
	uint16_t rolling_counter = 0U;
};

static_assert(sizeof(pdo_rx_1) == 8, "pdo_rx_1 size must be 8 bytes");

// PDO TX 1 structure
struct __attribute__((packed)) pdo_tx_1
{
	int16_t speed_rpm = 0;
	uint16_t battery_voltage = 0U;
	uint32_t ticks = 0U;
	uint16_t controller_temperature = 0U;
};

static_assert(sizeof(pdo_tx_1) == 10, "pdo_tx_1 size must be 10 bytes");

struct __attribute__((packed)) emergency_state
{
	uint16_t motor_id = 0U;
	uint8_t error_register = 0U;
	uint8_t faults_2 = 0U;
	uint16_t faults_1 = 0U;
	uint16_t warnings = 0U;
};

// Function declarations
can_frame pdo1_to_can_frame(node_id node, const pdo_rx_1& pdo);
can_frame write_sdo_rx(node_id node, uint16_t index, uint8_t sub_index, uint32_t data);
void send_pdo_frame(const can_frame& frame);
void send_nmt_command(node_id node, nmt::commands cmd);

} 
} // namespace canopen

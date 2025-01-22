#include "canopen.h"
#include "STM32_CAN.h"
#include <cstring>

// Ensure HAL_CAN_MODULE_ENABLED is defined correctly
#if !defined(HAL_CAN_MODULE_ENABLED)
#define HAL_CAN_MODULE_ENABLED
#endif

// Declare the global CAN instance defined in hermes_main.ino
extern STM32_CAN Can;

namespace asi
{
namespace canopen
{

// Function to convert PDO_RX1 to CAN frame
can_frame pdo1_to_can_frame(node_id node, const pdo_rx_1& pdo)
{
	can_frame frame{};
	frame.id = static_cast<uint32_t>(function_id::pdo_rx_1) | static_cast<uint32_t>(node);
	frame.dlc = sizeof(pdo_rx_1);

	memcpy(frame.data, &pdo, sizeof(pdo_rx_1));
	return frame;
}

// Function to create SDO RX CAN frame
can_frame write_sdo_rx(node_id node, uint16_t index, uint8_t sub_index, uint32_t data)
{
	can_frame frame{};
	frame.id = static_cast<uint32_t>(function_id::sdo_rx) | static_cast<uint32_t>(node);
	frame.dlc = 8U;

	frame.data[0] = static_cast<uint8_t>(sdo_write::command::write_4_bytes);
	frame.data[1] = static_cast<uint8_t>(index & 0xFFU);
	frame.data[2] = static_cast<uint8_t>((index >> 8) & 0xFFU);
	frame.data[3] = sub_index;
	frame.data[4] = static_cast<uint8_t>(data & 0xFFU);
	frame.data[5] = static_cast<uint8_t>((data >> 8) & 0xFFU);
	frame.data[6] = static_cast<uint8_t>((data >> 16) & 0xFFU);
	frame.data[7] = static_cast<uint8_t>((data >> 24) & 0xFFU);

	return frame;
}

// Function to send PDO frame via CAN bus
void send_pdo_frame(const can_frame& frame)
{
	CAN_message_t can_msg{};
	can_msg.id = frame.id;
	can_msg.len = frame.dlc;

	memcpy(can_msg.buf, frame.data, frame.dlc);
	Can.write(can_msg);
}

// Function to send NMT command via CAN bus
void send_nmt_command(node_id node, nmt::commands cmd)
{
	can_frame frame{};
	frame.id = static_cast<uint32_t>(function_id::nmt_node_control);
	frame.dlc = 2U;
	frame.data[0] = static_cast<uint8_t>(cmd);
	frame.data[1] = static_cast<uint8_t>(node);

	send_pdo_frame(frame);
}

} 
} // namespace canopen

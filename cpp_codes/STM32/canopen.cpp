#include "canopen.h"
#include "STM32_CAN.h"

extern STM32_CAN Can;

namespace asi {
namespace canopen {

can_frame pdo1_to_can_frame(node_id node_id, pdo_rx_1 pdo_rx_1) {
    can_frame frame;
    frame.id = static_cast<uint32_t>(function_id::pdo_rx_1) | static_cast<uint32_t>(node_id);
    frame.dlc = 8;
    std::copy(reinterpret_cast<uint8_t *>(&pdo_rx_1), reinterpret_cast<uint8_t *>(&pdo_rx_1) + 8, frame.data);
    // CanOpen expects 4 uint16_t words in little endian.
    return frame;
}

// This function is hardcoded to send "write_2_bytes" command since all ASI parameters were 2 bytes long
can_frame write_sdo_rx(node_id node_id, uint16_t index, uint8_t sub_index, uint32_t data) {
    can_frame frame;
    frame.id = static_cast<uint32_t>(function_id::sdo_rx) | static_cast<uint32_t>(node_id);
    frame.dlc = 8;
    frame.data[0] = static_cast<uint8_t>(sdo_write::command::write_2_bytes);
    frame.data[1] = index & 0xff;
    frame.data[2] = (index >> 8) & 0xff;
    frame.data[3] = sub_index;
    frame.data[4] = data & 0xff;
    frame.data[5] = (data >> 8) & 0xff;
    frame.data[6] = (data >> 16) & 0xff;
    frame.data[7] = (data >> 24) & 0xff;
    return frame;
}

void send_pdo_frame(can_frame frame) {
    CAN_message_t can_msg;
    can_msg.id = frame.id;
    can_msg.len = frame.dlc;
    memcpy(can_msg.buf, frame.data, frame.dlc);
    Can.write(can_msg);
}

void send_nmt_command(node_id node_id, nmt::commands cmd) {
    can_frame frame;
    frame.id = static_cast<uint32_t>(function_id::nmt_node_control);
    frame.dlc = 2;
    frame.data[0] = static_cast<uint8_t>(cmd);
    frame.data[1] = static_cast<uint16_t>(node_id);
    send_pdo_frame(frame);
}

} // namespace canopen
} // namespace asi

#include <Arduino.h>
#include "STM32_CAN.h"
#include <cstring>
#include "stm32f7xx_hal.h"
#include <micro_ros_arduino.h>
#include "ibus.h"
#include "asi.h"
#include "canopen.h"
 

// Include necessary micro-ROS headers
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16.h>
#include <geometry_msgs/msg/twist.h>

// Constants
#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

HardwareSerial Serial1(PD6, PD5);  // iBUS communication on pins PD6 (RX) and PD5 (TX)

STM32_CAN Can(CAN1, ALT);  // Use PA11/12 pins for CAN1.

// micro-ROS setup
rcl_subscription_t subscriber;
rcl_publisher_t publisher_left_ticks;
rcl_publisher_t publisher_right_ticks;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

std_msgs__msg__Int16 ticks_msg_left;
std_msgs__msg__Int16 ticks_msg_right;

geometry_msgs__msg__Twist cmd_vel_msg; // Declare the message

bool cmd_vel_received = false;

void error_loop(){
    while(1){
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(100);
    }
}

void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    double throttle_left = msg->linear.x - msg->angular.z;
    double throttle_right = msg->linear.x + msg->angular.z;

    throttle_left = constrain(throttle_left, -1.0, 1.0);
    throttle_right = constrain(throttle_right, -1.0, 1.0);

    bool reverse_left = (throttle_left < 0);
    bool reverse_right = (throttle_right < 0);

    throttle_left = abs(throttle_left);
    throttle_right = abs(throttle_right);

    double regen_left = 0;  // Example regenerative braking value
    double regen_right = 0;

    asi::update_motor_commands(throttle_left, throttle_right, regen_left, regen_right, reverse_left, reverse_right);

    cmd_vel_received = true;  // Set flag to indicate cmd_vel message has been received
}



void publish_motor_data(rcl_timer_t *timer, int64_t last_call_time) {
    ticks_msg_left.data = static_cast<int16_t>(asi::left_motor.ticks);
    ticks_msg_right.data = static_cast<int16_t>(asi::right_motor.ticks);
    RCSOFTCHECK(rcl_publish(&publisher_left_ticks, &ticks_msg_left, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_right_ticks, &ticks_msg_right, NULL));
}

void setup() {
    set_microros_transports();
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);  

    Serial.begin(115200);
    Serial1.begin(115200);  // Start Serial1 for iBUS communication
    Can.begin();

    // Initialize micro-ROS
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "motor_rpm_pub_cmd_vel_sub_node", "", &support));
;

    RCSOFTCHECK(rclc_publisher_init_default(
        &publisher_left_ticks,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
        "left_wheel_ticks"));
    RCSOFTCHECK(rclc_publisher_init_default(
        &publisher_right_ticks,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
        "right_wheel_ticks"));

    // Create subscriber for cmd_vel
    RCCHECK(rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // Create timer for motor RPM publishing
    const unsigned int timer_timeout = 100;


    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        publish_motor_data));

    // Create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

    Can.setBaudRate(1000000);  // 500KBPS

    // Send NMT command to enter operational mode
    asi::enter_operational_mode();
}

void loop() {
    uint32_t last_command_time = HAL_GetTick();
    uint32_t last_message_time = HAL_GetTick();
    bool ibus_received = false;
    bool ibus_active = true;
    bool cmd_vel_active = false;

    while (1) {
        uint32_t current_time = HAL_GetTick();

        // Check if cmd_vel is active
        if (cmd_vel_received) {
            cmd_vel_active = true;
            last_command_time = current_time;
        } else if (current_time - last_command_time > 500) {  // Timeout for cmd_vel (adjust as needed)
            cmd_vel_active = false;
        }

        // Switch between cmd_vel and iBUS based on activity
        if (cmd_vel_active) {
            if (ibus_active) {
                // Disable iBUS reading when cmd_vel is active
                ibus_active = false;
                Serial1.end();
            }

            // Handle cmd_vel inputs
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));  // Process incoming messages
            cmd_vel_received = false;  // Reset flag after processing
        } else {
            if (!ibus_active) {
                // Re-enable iBUS reading when cmd_vel is not active
                ibus_active = true;
                Serial1.begin(115200);
            }

            // Handle iBUS inputs
            if (readIBUS()) {
                ibus_received = true;

                double throttle_left = convertSteeringData(3, 0, 1000, 0) / 500.0 - 1.0;
                double direction_left = convertSteeringData(1, 0, 1000, 0) / 500.0 - 1.0;
                double throttle_right = convertThrottleData(3, 0, 1000, 0) / 500.0 - 1.0;
                double direction_right = convertThrottleData(1, 0, 1000, 0) / 500.0 - 1.0;

                throttle_left = constrain(throttle_left, -1.0, 1.0);
                throttle_right = constrain(throttle_right, -1.0, 1.0);

                bool reverse_left = (throttle_left < 0);
                bool reverse_right = (throttle_right < 0);

                throttle_left = abs(throttle_left);
                throttle_right = abs(throttle_right);

                double turnAdjustmentLeft = direction_left * 0.2;
                double turnAdjustmentRight = direction_right * 0.2;

                throttle_left += turnAdjustmentLeft;
                throttle_right -= turnAdjustmentRight;

                throttle_left = constrain(throttle_left, 0.0, 1.0);
                throttle_right = constrain(throttle_right, 0.0, 1.0);

                double regen_left = 0;
                double regen_right = 0;

                asi::update_motor_commands(throttle_left, throttle_right, regen_left, regen_right, reverse_left, reverse_right);
            }
        }

        // Check for timeout using kernel ticks
        uint32_t left_motor_last_time = asi::left_motor.last_time_received;
        uint32_t right_motor_last_time = asi::right_motor.last_time_received;
        bool left_motor_timeout = (current_time - left_motor_last_time > asi::rx_timeout_value);
        bool right_motor_timeout = (current_time - right_motor_last_time > asi::rx_timeout_value);

        if (left_motor_timeout || right_motor_timeout) {
            // Reinitialize CAN communication for the timed-out motor(s)
            if (left_motor_timeout) {
                asi::canopen::send_nmt_command(asi::canopen::node_id::left_motor, asi::canopen::nmt::commands::reset_communication);
                asi::left_motor.last_time_received = current_time;
            }
            if (right_motor_timeout) {
                asi::canopen::send_nmt_command(asi::canopen::node_id::right_motor, asi::canopen::nmt::commands::reset_communication);
                asi::right_motor.last_time_received = current_time;
            
            }
        }

        // Process received CAN frames
        CAN_message_t rx_msg;
        while (Can.read(rx_msg)) {
            asi::canopen::can_frame frame{ rx_msg.id, rx_msg.len, rx_msg.buf };
            asi::receive_frame(frame);
        }

        // Cmd vel task
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    }
}

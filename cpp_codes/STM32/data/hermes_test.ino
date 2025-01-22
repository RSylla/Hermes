// main.cpp

#include "asi.h"
#include "canopen.h"

#include <Arduino.h>
#include "STM32_CAN.h"
#include <IBusBM.h>

#include <algorithm>
#include <cmath>
#include <memory>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>      
#include <geometry_msgs/msg/twist.h>

// --------------------- Constants ---------------------
constexpr uint8_t LED_PIN = 13U;
constexpr uint8_t PRIORITY_LED_PIN = 12U;
constexpr uint32_t CMD_VEL_TIMEOUT_MS = 1000U;
constexpr uint32_t IBUS_TIMEOUT_MS = 500U;
constexpr unsigned int PUBLISH_TIMER_INTERVAL_MS = 20U;
constexpr unsigned int CONTROL_COMMAND_INTERVAL_MS = 50U;
constexpr float MAX_THROTTLE = 1.0f;
constexpr float MIN_THROTTLE = -1.0f;
constexpr float NEUTRAL_THROTTLE = 0.0f;
constexpr float DEFAULT_REGEN = 0.5f;

constexpr int16_t IBUS_NEUTRAL = 1500;
constexpr int16_t IBUS_DEADZONE = 20;

// Maximum throttle change per control loop for smooth start/stop
constexpr float MAX_THROTTLE_CHANGE = 0.1f;

// ----------------- Error Handling --------------------
void handle_rc_error(rcl_ret_t ret_code)
{
	if (ret_code != RCL_RET_OK)
	{
		rcl_reset_error();
		while (true)
		{
			digitalWrite(LED_PIN, !digitalRead(LED_PIN));
			delay(100U);
		}
	}
}

void handle_rc_soft_error(rcl_ret_t ret_code)
{
	if (ret_code != RCL_RET_OK)
	{
		rcl_reset_error();
	}
}

// ------------------ Motor Commands -------------------
struct MotorCommands
{
	float throttle_left = NEUTRAL_THROTTLE;
	float throttle_right = NEUTRAL_THROTTLE;
	float regen_left = DEFAULT_REGEN;
	float regen_right = DEFAULT_REGEN;
	bool reverse_left = false;
	bool reverse_right = false;

	void reset()
	{
		throttle_left = NEUTRAL_THROTTLE;
		throttle_right = NEUTRAL_THROTTLE;
		regen_left = DEFAULT_REGEN;
		regen_right = DEFAULT_REGEN;
		reverse_left = false;
		reverse_right = false;
	}
};

// -------------------- Globals ------------------------
std::unique_ptr<MotorCommands> cmd_vel_commands = std::make_unique<MotorCommands>();
std::unique_ptr<MotorCommands> ibus_commands = std::make_unique<MotorCommands>();

MotorCommands actual_motor_commands; // Holds the current motor commands being sent

volatile uint32_t last_cmd_vel_time = 0U;
volatile uint32_t last_ibus_time = 0U;

enum class InputSource
{
	NONE,
	CMD_VEL,
	IBUS
};

volatile InputSource active_input_source = InputSource::NONE;

// ------------------ Hardware Interfaces --------------
STM32_CAN Can(CAN1, ALT);
HardwareSerial SerialIBus(PD6, PD5);
IBusBM ibusInstance;

// ----------------- micro-ROS Setup --------------------
rcl_subscription_t cmd_vel_subscriber;
rcl_publisher_t publisher_left_ticks;
rcl_publisher_t publisher_right_ticks;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t publish_timer;
rcl_timer_t control_timer;

std_msgs__msg__Int32 ticks_msg_left;
std_msgs__msg__Int32 ticks_msg_right;
geometry_msgs__msg__Twist cmd_vel_msg;

// Time synchronization
unsigned long long time_offset = 0;

// ------------------- Function Prototypes ---------------
void cmd_vel_callback(const void* msgin);
void publish_motor_data_callback(rcl_timer_t* timer, int64_t last_call_time);
void send_control_commands_callback(rcl_timer_t* timer, int64_t last_call_time);
void process_iBus_input();
void handle_motor_timeouts();
void setup_micro_ros();
void create_publishers();
void syncTime();
struct timespec getTime();

// ----------------- ROS Callback ------------------------
void cmd_vel_callback(const void* msgin)
{
	if (msgin == nullptr)
	{
		return;
	}

	const auto* msg = static_cast<const geometry_msgs__msg__Twist*>(msgin);

	last_cmd_vel_time = millis();

	const float linear_x = static_cast<float>(msg->linear.x);
	const float angular_z = static_cast<float>(msg->angular.z);

	cmd_vel_commands->throttle_left = std::clamp(linear_x - angular_z, MIN_THROTTLE, MAX_THROTTLE);
	cmd_vel_commands->throttle_right = std::clamp(linear_x + angular_z, MIN_THROTTLE, MAX_THROTTLE);

	cmd_vel_commands->reverse_left = (cmd_vel_commands->throttle_left < 0.0f);
	cmd_vel_commands->reverse_right = (cmd_vel_commands->throttle_right < 0.0f);

	cmd_vel_commands->throttle_left = std::abs(cmd_vel_commands->throttle_left);
	cmd_vel_commands->throttle_right = std::abs(cmd_vel_commands->throttle_right);
}

// -------------- Motor Data Publishing Callback ---------
void publish_motor_data_callback(rcl_timer_t*, int64_t)
{
	ticks_msg_left.data = asi::left_motor->ticks;   // Assign int32_t ticks directly
	ticks_msg_right.data = asi::right_motor->ticks; // Assign int32_t ticks directly

	handle_rc_soft_error(rcl_publish(&publisher_left_ticks, &ticks_msg_left, nullptr));
	handle_rc_soft_error(rcl_publish(&publisher_right_ticks, &ticks_msg_right, nullptr));
}

// ------------- Control Commands Callback ---------------
void send_control_commands_callback(rcl_timer_t*, int64_t)
{
	const uint32_t current_time = millis();

	const bool cmd_vel_active = (current_time - last_cmd_vel_time) < CMD_VEL_TIMEOUT_MS;
	const bool ibus_active = (current_time - last_ibus_time) < IBUS_TIMEOUT_MS;

	InputSource new_input_source = InputSource::NONE;

	// iBus has higher priority over cmd_vel
	if (ibus_active)
	{
		new_input_source = InputSource::IBUS;
	}
	else if (cmd_vel_active)
	{
		new_input_source = InputSource::CMD_VEL;
	}

	if (new_input_source != active_input_source)
	{
		// Reset previous commands
		switch (active_input_source)
		{
			case InputSource::CMD_VEL:
				cmd_vel_commands->reset();
				break;
			case InputSource::IBUS:
				ibus_commands->reset();
				break;
			default:
				break;
		}
		active_input_source = new_input_source;

		// Update LED indicators based on active input source
		switch (active_input_source)
		{
			case InputSource::CMD_VEL:
				digitalWrite(PRIORITY_LED_PIN, LOW); // Indicate cmd_vel is active
				break;
			case InputSource::IBUS:
				digitalWrite(PRIORITY_LED_PIN, HIGH); // Indicate iBus is active
				break;
			case InputSource::NONE:
				digitalWrite(PRIORITY_LED_PIN, LOW); // No active input
				break;
		}
	}

	// Select the appropriate commands to send
	const MotorCommands* commands_to_send = nullptr;
	switch (active_input_source)
	{
		case InputSource::CMD_VEL:
			commands_to_send = cmd_vel_commands.get();
			break;
		case InputSource::IBUS:
			commands_to_send = ibus_commands.get();
			break;
		default:
			commands_to_send = nullptr;
			break;
	}

	if (commands_to_send == nullptr)
	{
		static const MotorCommands default_commands;
		commands_to_send = &default_commands;
	}

	// Adjust actual_motor_commands towards commands_to_send for smooth start/stop

	// Left motor
	{
		float& actual_throttle = actual_motor_commands.throttle_left;
		float desired_throttle = commands_to_send->throttle_left;

		bool& actual_reverse = actual_motor_commands.reverse_left;
		bool desired_reverse = commands_to_send->reverse_left;

		// Handle reverse direction
		if (actual_reverse != desired_reverse)
		{
			// Bring throttle to zero first
			if (actual_throttle > 0.0f)
			{
				actual_throttle = std::max(0.0f, actual_throttle - MAX_THROTTLE_CHANGE);
			}
			else
			{
				actual_reverse = desired_reverse; // Update reverse
			}
		}
		else
		{
			// Compute the difference
			float throttle_diff = desired_throttle - actual_throttle;

			// Limit the change
			throttle_diff = std::clamp(throttle_diff, -MAX_THROTTLE_CHANGE, MAX_THROTTLE_CHANGE);

			// Update the actual throttle
			actual_throttle += throttle_diff;
		}

		// Copy regen value
		actual_motor_commands.regen_left = commands_to_send->regen_left;
	}

	// Right motor
	{
		float& actual_throttle = actual_motor_commands.throttle_right;
		float desired_throttle = commands_to_send->throttle_right;

		bool& actual_reverse = actual_motor_commands.reverse_right;
		bool desired_reverse = commands_to_send->reverse_right;

		// Handle reverse direction
		if (actual_reverse != desired_reverse)
		{
			// Bring throttle to zero first
			if (actual_throttle > 0.0f)
			{
				actual_throttle = std::max(0.0f, actual_throttle - MAX_THROTTLE_CHANGE);
			}
			else
			{
				actual_reverse = desired_reverse; // Update reverse
			}
		}
		else
		{
			// Compute the difference
			float throttle_diff = desired_throttle - actual_throttle;

			// Limit the change
			throttle_diff = std::clamp(throttle_diff, -MAX_THROTTLE_CHANGE, MAX_THROTTLE_CHANGE);

			// Update the actual throttle
			actual_throttle += throttle_diff;
		}

		// Copy regen value
		actual_motor_commands.regen_right = commands_to_send->regen_right;
	}

	// Send the adjusted commands to the motors
	asi::update_motor_commands(
		actual_motor_commands.throttle_left,
		actual_motor_commands.throttle_right,
		actual_motor_commands.regen_left,
		actual_motor_commands.regen_right,
		actual_motor_commands.reverse_left,
		actual_motor_commands.reverse_right
	);
}

// ------------------- iBus Processing Function ---------------
void process_iBus_input()
{
	if (ibusInstance.cnt_rec > 0)
	{
		// Read iBus channels
		const int16_t raw_throttle = ibusInstance.readChannel(2) - IBUS_NEUTRAL;
		const int16_t raw_steering = ibusInstance.readChannel(0) - IBUS_NEUTRAL;

		// Check if the input is beyond the dead zone
		const bool throttle_active = std::abs(raw_throttle) > IBUS_DEADZONE;
		const bool steering_active = std::abs(raw_steering) > IBUS_DEADZONE;

		if (throttle_active || steering_active)
		{
			last_ibus_time = millis(); // Update only when input is active

			float throttle = static_cast<float>(raw_throttle) / 500.0f;
			float steering = static_cast<float>(raw_steering) / 500.0f;

			// Differential drive calculations
			ibus_commands->throttle_left = std::clamp(throttle + steering, MIN_THROTTLE, MAX_THROTTLE);
			ibus_commands->throttle_right = std::clamp(throttle - steering, MIN_THROTTLE, MAX_THROTTLE);

			ibus_commands->reverse_left = (ibus_commands->throttle_left < 0.0f);
			ibus_commands->reverse_right = (ibus_commands->throttle_right < 0.0f);

			ibus_commands->throttle_left = std::abs(ibus_commands->throttle_left);
			ibus_commands->throttle_right = std::abs(ibus_commands->throttle_right);
		}

		ibusInstance.cnt_rec = 0;
	}
}

// ------------------- Motor Timeout Handling ---------------
void handle_motor_timeouts()
{
	const uint32_t current_time = millis();
	const uint32_t left_motor_last_time = asi::left_motor->last_time_received;
	const uint32_t right_motor_last_time = asi::right_motor->last_time_received;

	const bool left_motor_timeout = (current_time - left_motor_last_time) > asi::rx_timeout_value;
	const bool right_motor_timeout = (current_time - right_motor_last_time) > asi::rx_timeout_value;

	if (active_input_source == InputSource::NONE && (left_motor_timeout || right_motor_timeout))
	{
		if (left_motor_timeout)
		{
			asi::canopen::send_nmt_command(
				asi::canopen::node_id::left_motor,
				asi::canopen::nmt::commands::reset_node);
			asi::left_motor->last_time_received = current_time;
		}
		if (right_motor_timeout)
		{
			asi::canopen::send_nmt_command(
				asi::canopen::node_id::right_motor,
				asi::canopen::nmt::commands::reset_node);
			asi::right_motor->last_time_received = current_time;
		}
	}
}

// ---------------------- Setup ---------------------------
void setup()
{
	// Initialize LEDs
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, HIGH);

	pinMode(PRIORITY_LED_PIN, OUTPUT);
	digitalWrite(PRIORITY_LED_PIN, LOW); // Initially no active input

	SystemClock_Config();

	// Initialize Serial for debugging
	Serial.begin(115200);

	// Initialize micro-ROS transports
	set_microros_transports();

	// Initialize CAN bus
	Can.begin();
	Can.setBaudRate(1000000U); // 1Mbps

	// Initialize iBus
	SerialIBus.begin(115200U); // iBus baud rate
	ibusInstance.begin(SerialIBus);

	// Initialize micro-ROS
	setup_micro_ros();

	// Initialize actual_motor_commands
	actual_motor_commands.reset();

	// Enter operational mode for motors
	asi::enter_operational_mode();
}

void setup_micro_ros()
{
	allocator = rcl_get_default_allocator();

	// Initialize init_options
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	handle_rc_error(rcl_init_options_init(&init_options, allocator));

	// Set the ROS Domain ID to 7
	handle_rc_error(rcl_init_options_set_domain_id(&init_options, 7));

	// Initialize the support structure with init_options
	handle_rc_error(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// Create the node
	handle_rc_error(rclc_node_init_default(&node, "motor_control_node", "", &support));

	// Initialize publishers
	create_publishers();

	// Initialize subscription
	handle_rc_error(rclc_subscription_init_default(
		&cmd_vel_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"cmd_vel_out"));

	// Initialize timers
	handle_rc_error(rclc_timer_init_default(
		&publish_timer,
		&support,
		RCL_MS_TO_NS(PUBLISH_TIMER_INTERVAL_MS),
		publish_motor_data_callback));

	handle_rc_error(rclc_timer_init_default(
		&control_timer,
		&support,
		RCL_MS_TO_NS(CONTROL_COMMAND_INTERVAL_MS),
		send_control_commands_callback));

	// Initialize executor with increased capacity
	handle_rc_error(rclc_executor_init(&executor, &support.context, 10U, &allocator));

	// Add timers and subscription to executor
	handle_rc_error(rclc_executor_add_timer(&executor, &publish_timer));
	handle_rc_error(rclc_executor_add_timer(&executor, &control_timer));

	handle_rc_error(rclc_executor_add_subscription(
		&executor,
		&cmd_vel_subscriber,
		&cmd_vel_msg,
		&cmd_vel_callback,
		ON_NEW_DATA));

	// Synchronize time with the ROS agent
	syncTime();

	// Finalize init_options to avoid memory leaks
	handle_rc_error(rcl_init_options_fini(&init_options));
}


void create_publishers()
{
	handle_rc_error(rclc_publisher_init_default(
		&publisher_left_ticks,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"left_wheel_ticks"));

	handle_rc_error(rclc_publisher_init_default(
		&publisher_right_ticks,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"right_wheel_ticks"));
}

// ----------------------- Loop ---------------------------
void loop()
{
	// Spin the executor with a shorter spin time to improve responsiveness
	rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5U));

	// Process CAN messages efficiently
	CAN_message_t rx_msg;
	while (Can.read(rx_msg))
	{
		asi::canopen::can_frame frame{rx_msg.id, rx_msg.len, {}};
		memcpy(frame.data, rx_msg.buf, rx_msg.len);
		asi::receive_frame(frame);
	}

	// Process iBus messages
	ibusInstance.loop();

	// Handle iBus inputs
	process_iBus_input();

	// Handle motor timeouts
	handle_motor_timeouts();

	// Additional processing can be added here if necessary
}

// ------------------- Time Synchronization ---------------
void syncTime()
{
	// Attempt to synchronize time with the ROS agent
	handle_rc_error(rmw_uros_sync_session(10));
	unsigned long long ros_time_ms = rmw_uros_epoch_millis();
	unsigned long now = millis();
	time_offset = ros_time_ms - now;
}

void SystemClock_Config() {
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	// Configure the main internal regulator output voltage
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	// Initialize HSE Oscillator and activate PLL with HSE as source
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 432;  // VCO output clock = 2 * PLLN = 864 MHz
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // PLLCLK = 864/2 = 432 MHz
	RCC_OscInitStruct.PLL.PLLQ = 9;    // USB clock = 48 MHz
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	// Activate the OverDrive to reach the 216 MHz Frequency
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	// Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;  // HCLK = 216 MHz
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;   // APB1 = 54 MHz
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;   // APB2 = 108 MHz

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
		Error_Handler();
	}

	// Configure peripheral clock for USB, UART, etc.
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}

	// Configure the flash latency
	__HAL_FLASH_SET_LATENCY(FLASH_LATENCY_7);
}

struct timespec getTime()
{
	struct timespec tp = {0};
	// Add time difference between microcontroller time and ROS time to synchronize
	unsigned long long now_time = millis() + time_offset;
	tp.tv_sec = now_time / 1000;
	tp.tv_nsec = (now_time % 1000) * 1000000;

	return tp;
}

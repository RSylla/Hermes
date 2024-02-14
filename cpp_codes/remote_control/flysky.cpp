#include <IBusBM.h>

// Create iBus Object
IBusBM ibus;

// Motor control pins
const int greyPinThrottle = 9; // PWM support needed
const int greypurplePinThrottle = 8;
const int whitePinReverse = 7;
const int bluewhitePinReverse = 6;

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
int maxSpeed = 255; // Maximum speed

// Read the value of a given channel and convert to the range provided.
// If the channel is off, return the default value.
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue) {
    uint16_t ch = ibus.readChannel(channelInput);
    if (ch < 100) return defaultValue;
    // Adjusting the range to use the full range of motor speed
    return map(ch, 1000, 2000, minLimit, maxLimit);
}

void setup() {
    Serial.begin(115200); // Start serial monitor
    ibus.begin(Serial1); // Attach iBus object to serial port

    // Initialize motor control pins
    pinMode(greyPinThrottle, OUTPUT);
    pinMode(greypurplePinThrottle, OUTPUT);
    pinMode(whitePinReverse, OUTPUT);
    pinMode(bluewhitePinReverse, OUTPUT);

    analogWrite(greyPinThrottle, 0);
    analogWrite(greypurplePinThrottle, 0);
}

void loop() {
    // Read joystick values with the full range for speed
    int rightStickVerticalValue = readChannel(1, -maxSpeed, maxSpeed, 0);
    int rightStickHorizontalValue = readChannel(0, -maxSpeed, maxSpeed, 0);

    // Check if the throttle is in the neutral position
    if (rightStickVerticalValue == 0 && rightStickHorizontalValue == 0) {
        // Reset all actions
        leftMotorSpeed = 0;
        rightMotorSpeed = 0;
        digitalWrite(whitePinReverse, LOW);
        digitalWrite(bluewhitePinReverse, LOW);
    } else {
        // Determine the direction of movement
        bool isForward = rightStickVerticalValue > 0;
        digitalWrite(whitePinReverse, isForward ? HIGH : LOW);
        digitalWrite(bluewhitePinReverse, isForward ? HIGH : LOW);

        // Calculate base speed
        int baseSpeed;
        if (isForward) {
            // Full speed for forward
            baseSpeed = map(abs(rightStickVerticalValue), 0, maxSpeed, 0, maxSpeed / 2);
        } else {
            // Half speed for backward
            baseSpeed = map(abs(rightStickVerticalValue), 0, maxSpeed, 0, maxSpeed / 6);
        }

        // Adjust turning based on the horizontal joystick value
        int turnAdjustment;
        if (abs(rightStickHorizontalValue) > 5) {
            // Sharp turn: larger turn adjustment
            turnAdjustment = map(abs(rightStickHorizontalValue), 0, maxSpeed, 0, baseSpeed);
        } else {
            // Gentle turn or straight: smaller turn adjustment
            turnAdjustment = 0;
        }

        if (rightStickHorizontalValue < 0) {
            // Turning left
            leftMotorSpeed = baseSpeed - turnAdjustment;
            rightMotorSpeed = baseSpeed + turnAdjustment;
        } else if (rightStickHorizontalValue > 0) {
            // Turning right
            rightMotorSpeed = baseSpeed - turnAdjustment;
            leftMotorSpeed = baseSpeed + turnAdjustment;
        } else {
            // Moving straight
            leftMotorSpeed = rightMotorSpeed = baseSpeed;
        }
    }

    // Update motor speeds
    analogWrite(greyPinThrottle, leftMotorSpeed);
    analogWrite(greypurplePinThrottle, rightMotorSpeed);

    // Debugging output
    Serial.print("Left Motor Speed: ");
    Serial.println(leftMotorSpeed);
    Serial.print("Right Motor Speed: ");
    Serial.println(rightMotorSpeed);

    delay(10); // Stability delay
}
const int greyPinThrottle = 9; // PWM support needed
const int greypurplePinThrottle = 8;
const int whitePinReverse = 7;
const int bluewhitePinReverse = 6;

int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
int maxSpeed = 150; // Reduced maximum speed for smoother control
int accelerationRate = 5; // Rate of change of speed for smoother acceleration

void mapJoystickToMotors(int xAxis, int yAxis) {
    // Dead zone around the neutral position
    const int deadZone = 50; // Adjust this value as needed

    // Check if the joystick is in the dead zone
    if (abs(xAxis - 1500) <= deadZone && abs(yAxis - 1500) <= deadZone) {
        leftMotorSpeed = 0;
        rightMotorSpeed = 0;
        return;
    }

    bool isForward = yAxis > 1500;

    // Apply different speed scaling for forward and backward with smoother curve
    int targetSpeed = isForward ? map(yAxis, 1500, 2000, 0, maxSpeed) 
                                : map(yAxis, 1000, 1500, maxSpeed, 0);

    // Turning adjustment
    int turnAdjustment = map(abs(xAxis - 1500), 0, 500, 0, targetSpeed);

    // Calculate target motor speeds with turn adjustment
    int targetLeftMotorSpeed, targetRightMotorSpeed;
    if (xAxis > 1500) {
        // Turning right
        targetRightMotorSpeed = targetSpeed - turnAdjustment;
        targetLeftMotorSpeed = targetSpeed + turnAdjustment;
    } else if (xAxis < 1500) {
        // Turning left
        targetLeftMotorSpeed = targetSpeed - turnAdjustment;
        targetRightMotorSpeed = targetSpeed + turnAdjustment;
    } else {
        // Moving straight
        targetLeftMotorSpeed = targetRightMotorSpeed = targetSpeed;
    }

    // Invert motor speeds for backward movement
    if (!isForward) {
        targetLeftMotorSpeed = -targetLeftMotorSpeed;
        targetRightMotorSpeed = -targetRightMotorSpeed;
    }

    // Gradual acceleration to the target speed
    leftMotorSpeed += (targetLeftMotorSpeed - leftMotorSpeed) / accelerationRate;
    rightMotorSpeed += (targetRightMotorSpeed - rightMotorSpeed) / accelerationRate;

    // Set motor direction pins
    digitalWrite(whitePinReverse, isForward ? LOW : HIGH);
    digitalWrite(bluewhitePinReverse, isForward ? LOW : HIGH);
}


void setup() {
    Serial.begin(115200); // Start serial monitor
    // Initialize motor control pins
    pinMode(greyPinThrottle, OUTPUT);
    pinMode(greypurplePinThrottle, OUTPUT);
    pinMode(whitePinReverse, OUTPUT);
    pinMode(bluewhitePinReverse, OUTPUT);
    analogWrite(greyPinThrottle, 0);
    analogWrite(greypurplePinThrottle, 0);
}

void loop() {
    if (Serial.available() >= 4) {
        int xAxis = Serial.read() << 8 | Serial.read();
        int yAxis = Serial.read() << 8 | Serial.read();

        mapJoystickToMotors(xAxis, yAxis);

        analogWrite(greyPinThrottle, abs(leftMotorSpeed));
        analogWrite(greypurplePinThrottle, abs(rightMotorSpeed));

        Serial.print("Left Motor Speed: ");
        Serial.println(leftMotorSpeed);
        Serial.print("Right Motor Speed: ");
        Serial.println(rightMotorSpeed);
    }
}
#include <Arduino.h>
#include "PS4-Remote.h"
#include "MotionMotors.h"

// Configuration flags
#define EVENTS 1
#define BUTTONS 1
#define JOYSTICKS 1
#define SENSORS 1
#define USE_ACCELEROMETER 0 // Flag to enable/disable accelerometer data (1=enabled, 0=disabled)
#define DEADZONE 10         // Deadzone threshold for joystick drift (values from 0-127)
#define GYRO_DEADZONE 100   // Deadzone threshold for gyroscope (higher value = less sensitive)
#define ACC_DEADZONE 1000   // Deadzone threshold for accelerometer (higher value = less sensitive)
#define ACC_PRECISION 1000  // Precision divisor for accelerometer (higher value = less precise)

// Motor pin definitions
#define LEFT_MOTOR_PIN_A 5   // Left motor pin A
#define LEFT_MOTOR_PIN_B 6   // Left motor pin B
#define RIGHT_MOTOR_PIN_A 9  // Right motor pin A
#define RIGHT_MOTOR_PIN_B 10 // Right motor pin B

// Motor calibration factors
#define LEFT_MOTOR_CALIBRATION 1.0  // Adjust if left motor is faster/slower than right
#define RIGHT_MOTOR_CALIBRATION 1.0 // Adjust if right motor is faster/slower than left

// Motor speed settings
#define MAX_MOTOR_SPEED 255   // Maximum motor speed (0-255)
#define TURN_SPEED_FACTOR 0.7 // Factor to reduce speed during turns (0.0-1.0)

// Make flags accessible to PS4Remote class
int EVENTS_VALUE = EVENTS;
int BUTTONS_VALUE = BUTTONS;
int JOYSTICKS_VALUE = JOYSTICKS;
int SENSORS_VALUE = SENSORS;
int USE_ACCELEROMETER_VALUE = USE_ACCELEROMETER;
int DEADZONE_VALUE = DEADZONE;
int GYRO_DEADZONE_VALUE = GYRO_DEADZONE;
int ACC_DEADZONE_VALUE = ACC_DEADZONE;
int ACC_PRECISION_VALUE = ACC_PRECISION;

// Create PS4Remote instance
PS4Remote ps4;

// Create MotionMotors instance with pin definitions
MotionMotors motors(
    LEFT_MOTOR_PIN_A, LEFT_MOTOR_PIN_B,
    RIGHT_MOTOR_PIN_A, RIGHT_MOTOR_PIN_B,
    LEFT_MOTOR_CALIBRATION, RIGHT_MOTOR_CALIBRATION);

// Custom connection callback
void onPS4ControllerConnect()
{
    Serial.println("\n*** PS4 Controller Connected ***");
}

// Custom disconnection callback
void onPS4ControllerDisconnect()
{
    Serial.println("\n*** PS4 Controller Disconnected ***");

    // Stop motors when controller disconnects for safety
    motors.stop();
}

// Function to control motors based on joystick input
void controlMotorsWithJoystick(PS4Remote::ControllerState state)
{
    // Get left joystick Y-axis for forward/backward motion (inverted: up is negative)
    int leftY = -state.ly; // Invert so positive is forward

    // Get right joystick X-axis for turning
    int rightX = state.rx;

    // Calculate left and right motor speeds based on joystick positions
    int leftMotorSpeed = 0;
    int rightMotorSpeed = 0;

    // Forward/backward movement from left joystick
    if (abs(leftY) > DEADZONE)
    {
        // Map joystick value to motor speed
        int baseSpeed = map(abs(leftY), DEADZONE, 127, 0, MAX_MOTOR_SPEED);

        // Set direction based on joystick position
        if (leftY > 0)
        {
            // Forward
            leftMotorSpeed = baseSpeed;
            rightMotorSpeed = baseSpeed;
        }
        else
        {
            // Backward
            leftMotorSpeed = -baseSpeed;
            rightMotorSpeed = -baseSpeed;
        }
    }

    // Apply turning from right joystick
    if (abs(rightX) > DEADZONE)
    {
        // Calculate turn adjustment
        int turnAdjustment = map(abs(rightX), DEADZONE, 127, 0, MAX_MOTOR_SPEED * TURN_SPEED_FACTOR);

        // Apply turn adjustment based on direction
        if (rightX > 0)
        {
            // Turn right
            leftMotorSpeed += turnAdjustment;
            rightMotorSpeed -= turnAdjustment;
        }
        else
        {
            // Turn left
            leftMotorSpeed -= turnAdjustment;
            rightMotorSpeed += turnAdjustment;
        }
    }

    // Constrain motor speeds to valid range
    leftMotorSpeed = constrain(leftMotorSpeed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    rightMotorSpeed = constrain(rightMotorSpeed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);

    // Apply motor speeds
    if (leftMotorSpeed > 0)
    {
        motors.leftForward(leftMotorSpeed);
    }
    else if (leftMotorSpeed < 0)
    {
        motors.leftBackward(abs(leftMotorSpeed));
    }
    else
    {
        motors.leftStop();
    }

    if (rightMotorSpeed > 0)
    {
        motors.rightForward(rightMotorSpeed);
    }
    else if (rightMotorSpeed < 0)
    {
        motors.rightBackward(abs(rightMotorSpeed));
    }
    else
    {
        motors.rightStop();
    }
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Initializing PS4 Controller and Motors...");

    // Set custom callbacks
    PS4.attachOnConnect(onPS4ControllerConnect);
    PS4.attachOnDisconnect(onPS4ControllerDisconnect);

    // Initialize PS4 controller
    ps4.begin();

    // Initialize motors
    motors.begin();

    Serial.println("System initialized. Waiting for PS4 controller connection...");
}

void loop()
{
    // Get current controller state
    PS4Remote::ControllerState state = ps4.getState();

    // Control motors based on joystick input if controller is connected
    if (PS4.isConnected())
    {
        controlMotorsWithJoystick(state);

        // Handle special button controls
        if (state.options)
        {
            // Options button: Emergency stop
            motors.stop();
            Serial.println("EMERGENCY STOP");
            delay(500); // Debounce
        }

        // Toggle smooth acceleration with Share button
        static bool lastShareState = false;
        if (state.share && !lastShareState)
        {
            bool smooth = !motors.isSmoothEnabled();
            motors.setSmoothEnabled(smooth);
            Serial.print("Smooth acceleration: ");
            Serial.println(smooth ? "ENABLED" : "DISABLED");
            delay(200); // Debounce
        }
        lastShareState = state.share;
    }
    else
    {
        // No controller connected, ensure motors are stopped
        motors.stop();
    }

    delay(20); // Short delay for stability
}
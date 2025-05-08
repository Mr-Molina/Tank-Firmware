#include <Arduino.h>
#include "TankController.h"

// Configuration flags
#define EVENTS 1
#define BUTTONS 1
#define JOYSTICKS 1
#define SENSORS 1
#define USE_ACCELEROMETER 0 // Flag to enable/disable accelerometer data
#define DEADZONE 10         // Joystick deadzone (0-127)
#define GYRO_DEADZONE 100   // Gyroscope deadzone
#define ACC_DEADZONE 1000   // Accelerometer deadzone
#define ACC_PRECISION 1000  // Accelerometer precision divisor

// Motor pin definitions
#define LEFT_MOTOR_PIN_A 5
#define LEFT_MOTOR_PIN_B 6
#define RIGHT_MOTOR_PIN_A 9
#define RIGHT_MOTOR_PIN_B 10

// Motor settings
#define LEFT_MOTOR_CALIBRATION 1.0
#define RIGHT_MOTOR_CALIBRATION 1.0
#define MAX_MOTOR_SPEED 255
#define TURN_SPEED_FACTOR 0.7

// Create controller instance
TankController tank(
    // PS4 controller config
    EVENTS, BUTTONS, JOYSTICKS, SENSORS,
    USE_ACCELEROMETER, DEADZONE, GYRO_DEADZONE,
    ACC_DEADZONE, ACC_PRECISION,
    // Motor config
    LEFT_MOTOR_PIN_A, LEFT_MOTOR_PIN_B,
    RIGHT_MOTOR_PIN_A, RIGHT_MOTOR_PIN_B,
    LEFT_MOTOR_CALIBRATION, RIGHT_MOTOR_CALIBRATION,
    MAX_MOTOR_SPEED, TURN_SPEED_FACTOR);

void setup()
{
    Serial.begin(115200);
    Serial.println("Initializing PS4 Controller and Motors...");

    // Initialize the tank controller
    tank.begin();

    Serial.println("System initialized. Waiting for PS4 controller connection...");
}

void loop()
{
    // Update controller and handle all motor control
    // The timing is now handled internally by the TankController class
    tank.update();
    
    // No delay needed here - the controller handles its own timing
    // This allows the loop to run as fast as possible for other tasks
    
    // Other non-blocking code can be added here if needed
    // For example, reading sensors, updating displays, etc.
}
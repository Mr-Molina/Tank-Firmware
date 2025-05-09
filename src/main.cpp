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

// Debug flags
#define DEBUG_PS4_DATA 0    // Enable/disable PS4 controller data debug messages
#define DEBUG_MOTOR_ACTIONS 1 // Enable/disable motor action debug messages
#define DEBUG_CALIBRATION 1   // Enable/disable calibration adjustment debug messages

// Motor pin definitions
#define LEFT_MOTOR_PIN_A 18
#define LEFT_MOTOR_PIN_B 19
#define RIGHT_MOTOR_PIN_A 16
#define RIGHT_MOTOR_PIN_B 17

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

// Make configuration flags accessible to other modules
int DEADZONE_VALUE = DEADZONE;
int GYRO_DEADZONE_VALUE = GYRO_DEADZONE;
int ACC_DEADZONE_VALUE = ACC_DEADZONE;
int ACC_PRECISION_VALUE = ACC_PRECISION;
int USE_ACCELEROMETER_VALUE = USE_ACCELEROMETER;
int EVENTS_VALUE = EVENTS;
int DEBUG_PS4_DATA_VALUE = DEBUG_PS4_DATA;
int DEBUG_MOTOR_ACTIONS_VALUE = DEBUG_MOTOR_ACTIONS;
int DEBUG_CALIBRATION_VALUE = DEBUG_CALIBRATION;

void setup()
{
    Serial.begin(115200);
    Serial.println("Starting initialization...");
    
    Serial.printf("Debug settings - PS4 Data: %s, Motor Actions: %s, Calibration: %s\n", 
                 DEBUG_PS4_DATA_VALUE ? "ON" : "OFF", 
                 DEBUG_MOTOR_ACTIONS_VALUE ? "ON" : "OFF",
                 DEBUG_CALIBRATION_VALUE ? "ON" : "OFF");

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

    // Add a small delay to prevent the ESP32 from running too fast
    // This helps prevent potential issues with the Bluetooth stack
    delay(1);

    // Check for debug toggle commands from Serial
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command == "debug ps4 on") {
            DEBUG_PS4_DATA_VALUE = 1;
            Serial.println("PS4 debug output enabled");
        } 
        else if (command == "debug ps4 off") {
            DEBUG_PS4_DATA_VALUE = 0;
            Serial.println("PS4 debug output disabled");
        }
        else if (command == "debug motor on") {
            DEBUG_MOTOR_ACTIONS_VALUE = 1;
            Serial.println("Motor debug output enabled");
        }
        else if (command == "debug motor off") {
            DEBUG_MOTOR_ACTIONS_VALUE = 0;
            Serial.println("Motor debug output disabled");
        }
        else if (command == "debug status") {
            Serial.printf("Debug status - PS4 Data: %s, Motor Actions: %s, Calibration: %s\n", 
                         DEBUG_PS4_DATA_VALUE ? "ON" : "OFF", 
                         DEBUG_MOTOR_ACTIONS_VALUE ? "ON" : "OFF",
                         DEBUG_CALIBRATION_VALUE ? "ON" : "OFF");
        }
        else if (command == "debug calibration on") {
            DEBUG_CALIBRATION_VALUE = 1;
            Serial.println("Calibration debug output enabled");
        }
        else if (command == "debug calibration off") {
            DEBUG_CALIBRATION_VALUE = 0;
            Serial.println("Calibration debug output disabled");
        }
        else if (command == "calibration status") {
            // Get current calibration values from the tank controller
            float leftCal = tank.getLeftMotorCalibration();
            float rightCal = tank.getRightMotorCalibration();
            Serial.printf("Motor Calibration - Left: %.2f, Right: %.2f\n", leftCal, rightCal);
        }
    }
}
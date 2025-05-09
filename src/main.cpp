#include <Arduino.h>
#include "TankController.h"

/********************************************
 * ROBOT SETTINGS
 * 
 * These settings control how the robot works.
 * You can change these values to adjust how
 * the robot behaves!
 ********************************************/

// Controller settings - What features to use
#define EVENTS 1            // Do we want to track button press events?
#define BUTTONS 1           // Do we want to use controller buttons?
#define JOYSTICKS 1         // Do we want to use the joysticks?
#define SENSORS 1           // Do we want to use controller sensors?
#define USE_ACCELEROMETER 0 // Should we use the motion sensors? (0 = No, 1 = Yes)

// Joystick settings
#define DEADZONE 10         // How far to move joystick before robot responds (0-127)
                            // Higher number = less sensitive to small movements

// Motion sensor settings
#define GYRO_DEADZONE 100   // Ignore small gyroscope movements
#define ACC_DEADZONE 1000   // Ignore small accelerometer movements
#define ACC_PRECISION 1000  // How precise the accelerometer readings should be

// Debug message settings - What information to show on the computer
#define DEBUG_PS4_DATA 0      // Show controller button presses? (0 = No, 1 = Yes)
#define DEBUG_MOTOR_ACTIONS 1 // Show motor movements? (0 = No, 1 = Yes)
#define DEBUG_CALIBRATION 1   // Show motor power adjustments? (0 = No, 1 = Yes)

// Motor connection pins - Which wires connect where
#define LEFT_MOTOR_PIN_A 18   // Left motor forward pin
#define LEFT_MOTOR_PIN_B 19   // Left motor backward pin
#define RIGHT_MOTOR_PIN_A 16  // Right motor forward pin
#define RIGHT_MOTOR_PIN_B 17  // Right motor backward pin

// Motor power settings
#define LEFT_MOTOR_CALIBRATION 1.0  // Left motor power factor (1.0 = 100% power)
#define RIGHT_MOTOR_CALIBRATION 1.0 // Right motor power factor (1.0 = 100% power)
#define MAX_MOTOR_SPEED 255         // Maximum motor speed (0-255)
#define TURN_SPEED_FACTOR 0.7       // How fast the robot turns (0.0-1.0)

// Create our robot controller
// This sets up the "brain" of the robot with all our settings
TankController tank(
    // Tell the controller which features to use
    EVENTS, BUTTONS, JOYSTICKS, SENSORS,
    USE_ACCELEROMETER, DEADZONE, GYRO_DEADZONE,
    ACC_DEADZONE, ACC_PRECISION,
    
    // Tell the controller how the motors are connected
    LEFT_MOTOR_PIN_A, LEFT_MOTOR_PIN_B,    // Left motor pins
    RIGHT_MOTOR_PIN_A, RIGHT_MOTOR_PIN_B,  // Right motor pins
    
    // Tell the controller about motor power settings
    LEFT_MOTOR_CALIBRATION, RIGHT_MOTOR_CALIBRATION,  // Motor balance
    MAX_MOTOR_SPEED, TURN_SPEED_FACTOR);              // Speed settings

// Share our settings with other parts of the program
// These variables let other code files access our settings
int DEADZONE_VALUE = DEADZONE;                // Joystick dead zone
int GYRO_DEADZONE_VALUE = GYRO_DEADZONE;      // Motion sensor dead zone
int ACC_DEADZONE_VALUE = ACC_DEADZONE;        // Accelerometer dead zone
int ACC_PRECISION_VALUE = ACC_PRECISION;      // Motion sensor precision
int USE_ACCELEROMETER_VALUE = USE_ACCELEROMETER; // Use motion sensors?
int EVENTS_VALUE = EVENTS;                    // Track button events?
int DEBUG_PS4_DATA_VALUE = DEBUG_PS4_DATA;    // Show controller data?
int DEBUG_MOTOR_ACTIONS_VALUE = DEBUG_MOTOR_ACTIONS; // Show motor actions?
int DEBUG_CALIBRATION_VALUE = DEBUG_CALIBRATION;     // Show calibration changes?

// This function runs once when the robot first turns on
void setup()
{
    // Start communication with the computer at 115200 bits per second
    Serial.begin(115200);
    Serial.println("Robot is starting up...");
    
    // Show which debug messages will appear on the computer screen
    Serial.printf("Debug messages - Controller: %s, Motors: %s, Calibration: %s\n", 
                 DEBUG_PS4_DATA_VALUE ? "ON" : "OFF", 
                 DEBUG_MOTOR_ACTIONS_VALUE ? "ON" : "OFF",
                 DEBUG_CALIBRATION_VALUE ? "ON" : "OFF");

    Serial.println("Setting up PS4 Controller and Motors...");

    // Start the robot controller
    // This turns on Bluetooth and prepares the motors
    tank.begin();

    Serial.println("Robot ready! Please connect your PS4 controller...");
}

// This function runs over and over again while the robot is on
void loop()
{
    // Check the controller and update the motors
    // This reads the joysticks and buttons, then moves the motors
    tank.update();

    // Take a very short break (1 millisecond)
    // This helps the Bluetooth connection work better
    delay(1);

    // Check if someone is typing commands on the computer
    if (Serial.available() > 0) {
        // Read the command that was typed
        String command = Serial.readStringUntil('\n');
        command.trim();  // Remove any extra spaces
        
        // Handle different commands
        
        // Commands to turn debug messages on and off
        if (command == "debug ps4 on") {
            DEBUG_PS4_DATA_VALUE = 1;  // 1 means ON
            Serial.println("Controller debug messages: ON");
        } 
        else if (command == "debug ps4 off") {
            DEBUG_PS4_DATA_VALUE = 0;  // 0 means OFF
            Serial.println("Controller debug messages: OFF");
        }
        else if (command == "debug motor on") {
            DEBUG_MOTOR_ACTIONS_VALUE = 1;
            Serial.println("Motor debug messages: ON");
        }
        else if (command == "debug motor off") {
            DEBUG_MOTOR_ACTIONS_VALUE = 0;
            Serial.println("Motor debug messages: OFF");
        }
        else if (command == "debug calibration on") {
            DEBUG_CALIBRATION_VALUE = 1;
            Serial.println("Calibration debug messages: ON");
        }
        else if (command == "debug calibration off") {
            DEBUG_CALIBRATION_VALUE = 0;
            Serial.println("Calibration debug messages: OFF");
        }
        
        // Commands to check status
        else if (command == "debug status") {
            // Show which debug messages are turned on
            Serial.printf("Debug messages - Controller: %s, Motors: %s, Calibration: %s\n", 
                         DEBUG_PS4_DATA_VALUE ? "ON" : "OFF", 
                         DEBUG_MOTOR_ACTIONS_VALUE ? "ON" : "OFF",
                         DEBUG_CALIBRATION_VALUE ? "ON" : "OFF");
        }
        else if (command == "calibration status") {
            // Show the current motor power settings
            float leftCal = tank.getLeftMotorCalibration();
            float rightCal = tank.getRightMotorCalibration();
            Serial.printf("Motor Power Settings - Left: %.2f, Right: %.2f\n", leftCal, rightCal);
        }
    }
}
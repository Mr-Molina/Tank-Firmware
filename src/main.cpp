#include <Arduino.h>
#include "TankController.h"

/********************************************
 * ROBOT SETTINGS
 *
 * Hey there! These settings control how your robot works.
 * You can change these numbers to make your robot behave differently.
 * Think of it like adjusting the settings on a video game!
 ********************************************/

// Controller settings - What cool features to use
#define EVENTS 1            // Should the robot notice when you press and release buttons? (1 = Yes, 0 = No)
#define BUTTONS 1           // Should the robot respond to controller buttons? (1 = Yes, 0 = No)
#define JOYSTICKS 1         // Should the robot respond to the joysticks? (1 = Yes, 0 = No)
#define SENSORS 1           // Should the robot use the controller's motion sensors? (1 = Yes, 0 = No)
#define USE_ACCELEROMETER 0 // Should the robot detect tilting of the controller? (0 = No, 1 = Yes)

// Joystick settings
#define DEADZONE 10 // How far you need to move the joystick before the robot responds (0-127)
                    // Higher number = less sensitive (ignores tiny movements)

// Motion sensor settings
#define GYRO_DEADZONE 100  // Ignore small controller movements (higher = less sensitive)
#define ACC_DEADZONE 1000  // Ignore small tilting movements (higher = less sensitive)
#define ACC_PRECISION 1000 // How precise the tilt readings should be (higher = less precise)

// Debug message settings - What information to show on the computer screen
#define DEBUG_PS4_DATA 0      // Show controller button presses? (0 = No, 1 = Yes)
#define DEBUG_MOTOR_ACTIONS 1 // Show when motors move? (0 = No, 1 = Yes)
#define DEBUG_CALIBRATION 1   // Show when motor power is adjusted? (0 = No, 1 = Yes)

// Motor connection pins - Which wires connect where on the circuit board
#define LEFT_MOTOR_PIN_A 18  // Left motor forward pin
#define LEFT_MOTOR_PIN_B 19  // Left motor backward pin
#define RIGHT_MOTOR_PIN_A 16 // Right motor forward pin
#define RIGHT_MOTOR_PIN_B 17 // Right motor backward pin

// Motor power settings
#define LEFT_MOTOR_CALIBRATION 1.0  // Left motor power factor (1.0 = 100% power)
#define RIGHT_MOTOR_CALIBRATION 1.0 // Right motor power factor (1.0 = 100% power)
#define MAX_MOTOR_SPEED 255         // Maximum motor speed (0-255)
#define TURN_SPEED_FACTOR 0.7       // How fast the robot turns (0.0-1.0)
                                    // Lower number = gentler turns

// Create our robot controller
// This sets up the "brain" of the robot with all our settings
TankController tank(
    // Tell the controller which features to use
    EVENTS, BUTTONS, JOYSTICKS, SENSORS,
    USE_ACCELEROMETER, DEADZONE, GYRO_DEADZONE,
    ACC_DEADZONE, ACC_PRECISION,

    // Tell the controller how the motors are connected
    LEFT_MOTOR_PIN_A, LEFT_MOTOR_PIN_B,   // Left motor pins
    RIGHT_MOTOR_PIN_A, RIGHT_MOTOR_PIN_B, // Right motor pins

    // Tell the controller about motor power settings
    LEFT_MOTOR_CALIBRATION, RIGHT_MOTOR_CALIBRATION, // Motor balance
    MAX_MOTOR_SPEED, TURN_SPEED_FACTOR);             // Speed settings

// Share our settings with other parts of the program
// These variables let other code files access our settings
int DEADZONE_VALUE = DEADZONE;                       // Joystick dead zone
int GYRO_DEADZONE_VALUE = GYRO_DEADZONE;             // Motion sensor dead zone
int ACC_DEADZONE_VALUE = ACC_DEADZONE;               // Accelerometer dead zone
int ACC_PRECISION_VALUE = ACC_PRECISION;             // Motion sensor precision
int USE_ACCELEROMETER_VALUE = USE_ACCELEROMETER;     // Use motion sensors?
int EVENTS_VALUE = EVENTS;                           // Track button events?
int DEBUG_PS4_DATA_VALUE = DEBUG_PS4_DATA;           // Show controller data?
int DEBUG_MOTOR_ACTIONS_VALUE = DEBUG_MOTOR_ACTIONS; // Show motor actions?
int DEBUG_CALIBRATION_VALUE = DEBUG_CALIBRATION;     // Show calibration changes?

// This function runs once when the robot first turns on
void setup()
{
    // Start communication with the computer at 115200 bits per second
    // This is like opening a phone line to talk to the computer
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
// It's like the robot's heartbeat - it keeps checking what to do next
void loop()
{
    // Check the controller and update the motors
    // This reads the joysticks and buttons, then moves the motors
    tank.update();

    // Take a very short break (1 millisecond)
    // This helps the Bluetooth connection work better
    delay(1);
}
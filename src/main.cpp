#include <Arduino.h>
#include "PS4-Remote.h"

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

void setup()
{
    Serial.begin(115200);
    ps4.begin();
}

void loop()
{
    // Get current controller state if needed
    // PS4Remote::ControllerState state = ps4.getState();
    
    // Add your robot control code here
    
    delay(100);
}
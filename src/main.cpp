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

// Flag to track if we've printed the connection info
bool connectionInfoPrinted = false;

// Function to print controller battery level
void printBatteryLevel() {
    if (PS4.isConnected()) {
        int batteryLevel = PS4.Battery();
        Serial.print("PS4 Controller Battery Level: ");
        
        // Convert battery level to percentage or description
        switch (batteryLevel) {
            case 0:
                Serial.println("Empty (0%)");
                break;
            case 1:
                Serial.println("Low (20-40%)");
                break;
            case 2:
                Serial.println("Medium (40-60%)");
                break;
            case 3:
                Serial.println("High (60-80%)");
                break;
            case 4:
                Serial.println("Full (80-100%)");
                break;
            default:
                Serial.println("Unknown");
                break;
        }
    }
}

// Function to print controller state information
void printControllerState() {
    if (PS4.isConnected()) {
        PS4Remote::ControllerState state = ps4.getState();
        
        Serial.println("\n--- PS4 Controller State Information ---");
        
        // Print connection status
        Serial.println("Connection: Established");
        
        // Print battery level
        printBatteryLevel();
        
        // Print button states
        Serial.println("\nButton States:");
        Serial.print("Square: "); Serial.println(state.square ? "Pressed" : "Released");
        Serial.print("Triangle: "); Serial.println(state.triangle ? "Pressed" : "Released");
        Serial.print("Cross: "); Serial.println(state.cross ? "Pressed" : "Released");
        Serial.print("Circle: "); Serial.println(state.circle ? "Pressed" : "Released");
        
        // Print D-pad states
        Serial.println("\nD-Pad:");
        Serial.print("Up: "); Serial.println(state.up ? "Pressed" : "Released");
        Serial.print("Down: "); Serial.println(state.down ? "Pressed" : "Released");
        Serial.print("Left: "); Serial.println(state.left ? "Pressed" : "Released");
        Serial.print("Right: "); Serial.println(state.right ? "Pressed" : "Released");
        
        // Print shoulder buttons and triggers
        Serial.println("\nShoulder Buttons & Triggers:");
        Serial.print("L1: "); Serial.println(state.l1 ? "Pressed" : "Released");
        Serial.print("R1: "); Serial.println(state.r1 ? "Pressed" : "Released");
        Serial.print("L2 Value: "); Serial.println(state.l2);
        Serial.print("R2 Value: "); Serial.println(state.r2);
        
        // Print other buttons
        Serial.println("\nOther Buttons:");
        Serial.print("L3: "); Serial.println(state.l3 ? "Pressed" : "Released");
        Serial.print("R3: "); Serial.println(state.r3 ? "Pressed" : "Released");
        Serial.print("Share: "); Serial.println(state.share ? "Pressed" : "Released");
        Serial.print("Options: "); Serial.println(state.options ? "Pressed" : "Released");
        Serial.print("PS Button: "); Serial.println(state.ps ? "Pressed" : "Released");
        Serial.print("Touchpad: "); Serial.println(state.touchpad ? "Pressed" : "Released");
        
        // Print joystick values
        Serial.println("\nJoystick Values:");
        Serial.print("Left Stick X: "); Serial.println(state.lx);
        Serial.print("Left Stick Y: "); Serial.println(state.ly);
        Serial.print("Right Stick X: "); Serial.println(state.rx);
        Serial.print("Right Stick Y: "); Serial.println(state.ry);
        
        // Print gyroscope values if sensors are enabled
        if (SENSORS) {
            Serial.println("\nGyroscope Values:");
            Serial.print("X: "); Serial.println(state.gx);
            Serial.print("Y: "); Serial.println(state.gy);
            Serial.print("Z: "); Serial.println(state.gz);
        }
        
        // Print accelerometer values if enabled
        if (USE_ACCELEROMETER) {
            Serial.println("\nAccelerometer Values:");
            Serial.print("X: "); Serial.println(state.ax);
            Serial.print("Y: "); Serial.println(state.ay);
            Serial.print("Z: "); Serial.println(state.az);
        }
        
        Serial.println("---------------------------------------\n");
    }
}

// Custom connection callback
void onPS4ControllerConnect() {
    Serial.println("\n*** PS4 Controller Connected ***");
    connectionInfoPrinted = false;  // Reset flag to print info on next loop
}

// Custom disconnection callback
void onPS4ControllerDisconnect() {
    Serial.println("\n*** PS4 Controller Disconnected ***");
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Initializing PS4 Controller...");
    
    // Set custom callbacks
    PS4.attachOnConnect(onPS4ControllerConnect);
    PS4.attachOnDisconnect(onPS4ControllerDisconnect);
    
    ps4.begin();
    Serial.println("PS4 Controller initialized. Waiting for connection...");
}

void loop()
{
    // Check if controller is connected and we haven't printed the info yet
    if (PS4.isConnected() && !connectionInfoPrinted) {
        // Wait a moment for the connection to stabilize
        delay(500);
        
        // Print controller state information
        printControllerState();
        
        // Set flag to avoid printing repeatedly
        connectionInfoPrinted = true;
    }
    
    // Get current controller state if needed
    // PS4Remote::ControllerState state = ps4.getState();
    
    // Add your robot control code here
    
    // Periodically check battery level (every 30 seconds)
    static unsigned long lastBatteryCheck = 0;
    if (PS4.isConnected() && millis() - lastBatteryCheck > 30000) {
        printBatteryLevel();
        lastBatteryCheck = millis();
    }
    
    delay(100);
}
#ifndef PS4_REMOTE_H
#define PS4_REMOTE_H

#include <Arduino.h>
#include <PS4Controller.h>

// Default configuration values for PS4 controller
#define DEADZONE_VALUE 10         // Joystick deadzone (0-127)
#define GYRO_DEADZONE_VALUE 100   // Gyroscope deadzone
#define ACC_DEADZONE_VALUE 1000   // Accelerometer deadzone
#define ACC_PRECISION_VALUE 1000  // Accelerometer precision divisor
#define USE_ACCELEROMETER_VALUE 0 // Flag to enable/disable accelerometer data
#define EVENTS_VALUE 1            // Flag to enable/disable event processing

class PS4Remote
{
public:
     // Constructor and initialization
     PS4Remote();
     void begin();

     // Button states and values
     struct ControllerState
     {
          // Face buttons
          bool square;
          bool triangle;
          bool cross;
          bool circle;

          // D-pad
          bool up;
          bool down;
          bool left;
          bool right;

          // Shoulder buttons and triggers
          bool l1;
          bool r1;
          int l2; // Analog (0-255)
          int r2; // Analog (0-255)

          // Thumbstick buttons and other buttons
          bool l3;
          bool r3;
          bool share;
          bool options;
          bool ps;
          bool touchpad;

          // Joystick values
          int lx;
          int ly;
          int rx;
          int ry;

          // Gyroscope values
          int gx;
          int gy;
          int gz;

          // Accelerometer values
          int ax;
          int ay;
          int az;

          // Status flags
          bool dataChanged;
     };

     // Get current controller state
     ControllerState getState();

     // Update controller state (called by notify callback)
     void update();

     // Helper functions
     int applyDeadzone(int value, int deadzone);
     int reduceAccPrecision(int value);

     // Bluetooth functions
     void removePairedDevices();
     void printDeviceAddress();

     // Static callback functions
     static void notifyCallback();
     static void onConnectCallback();
     static void onDisconnectCallback();

     // Set instance for callbacks
     static void setInstance(PS4Remote *instance);

private:
     // Current and previous controller state
     ControllerState currentState;
     ControllerState prevState;

     // Timestamp for rate limiting
     unsigned long lastTimeStamp;

     // Static instance for callbacks
     static PS4Remote *_instance;
};

#endif // PS4_REMOTE_H
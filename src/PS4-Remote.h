#ifndef PS4_REMOTE_H
#define PS4_REMOTE_H

#include <Arduino.h>
#include <PS4Controller.h>

// Default settings for the PS4 controller
#define DEFAULT_DEADZONE 10         // How far to move joystick before robot responds (0-127)
#define DEFAULT_GYRO_DEADZONE 100   // How much to rotate controller before robot responds
#define DEFAULT_ACC_DEADZONE 1000   // How much to tilt controller before robot responds
#define DEFAULT_ACC_PRECISION 1000  // How precise the tilt readings should be
#define DEFAULT_USE_ACCELEROMETER 0 // Should we use tilt detection? (0 = No, 1 = Yes)
#define DEFAULT_EVENTS 1            // Should we track button press events? (0 = No, 1 = Yes)

class PS4Remote
{
public:
     // Constructor and initialization
     PS4Remote();
     void begin();

     // This keeps track of all the controller's buttons and joysticks
     struct ControllerState
     {
          // Face buttons (the buttons on the right side of the controller)
          bool square;    // Is Square button pressed?
          bool triangle;  // Is Triangle button pressed?
          bool cross;     // Is X button pressed?
          bool circle;    // Is Circle button pressed?

          // D-pad (the four buttons on the left side of the controller)
          bool up;        // Is Up button pressed?
          bool down;      // Is Down button pressed?
          bool left;      // Is Left button pressed?
          bool right;     // Is Right button pressed?

          // Shoulder buttons and triggers (the buttons on top of the controller)
          bool l1;        // Is L1 button pressed?
          bool r1;        // Is R1 button pressed?
          int l2;         // How hard is L2 trigger pressed? (0-255)
          int r2;         // How hard is R2 trigger pressed? (0-255)

          // Thumbstick buttons and other buttons
          bool l3;        // Is left thumbstick button pressed?
          bool r3;        // Is right thumbstick button pressed?
          bool share;     // Is Share button pressed?
          bool options;   // Is Options button pressed?
          bool ps;        // Is PlayStation button pressed?
          bool touchpad;  // Is touchpad pressed?

          // Joystick values (how far each joystick is moved)
          int lx;         // Left joystick left/right position (-127 to 127)
          int ly;         // Left joystick up/down position (-127 to 127)
          int rx;         // Right joystick left/right position (-127 to 127)
          int ry;         // Right joystick up/down position (-127 to 127)

          // Gyroscope values (how the controller is being rotated)
          int gx;         // Rotation around X axis
          int gy;         // Rotation around Y axis
          int gz;         // Rotation around Z axis

          // Accelerometer values (how the controller is being tilted)
          int ax;         // Tilt forward/backward
          int ay;         // Tilt left/right
          int az;         // Tilt up/down

          // Status flags
          bool dataChanged;  // Has anything changed since last update?
     };

     // Get current controller state
     ControllerState getState();

     // Update controller state (called by notify callback)
     void update();

     // Helper functions
     int applyDeadzone(int value, int deadzone);  // Ignore small joystick movements
     int reduceAccPrecision(int value);           // Make tilt readings less precise

     // Bluetooth functions
     void removePairedDevices();  // Remove previously paired controllers
     void printDeviceAddress();   // Show this device's Bluetooth address

     // Static callback functions - these run when controller events happen
     static void notifyCallback();        // Called when controller data changes
     static void onConnectCallback();     // Called when controller connects
     static void onDisconnectCallback();  // Called when controller disconnects

     // Set instance for callbacks
     static void setInstance(PS4Remote *instance);

private:
     // Current and previous controller state
     ControllerState currentState;  // What buttons are pressed now
     ControllerState prevState;     // What buttons were pressed before

     // Timestamp for rate limiting
     unsigned long lastTimeStamp;  // When we last updated controller data

     // Static instance for callbacks
     static PS4Remote *_instance;  // Pointer to this object for callbacks
};

#endif // PS4_REMOTE_H
#ifndef TANK_CONTROLLER_H
#define TANK_CONTROLLER_H

#include <Arduino.h>
#include "PS4-Remote.h"
#include "MotionMotors.h"

/**
 * TankController - The brain of our robot!
 *
 * This class connects the PS4 controller to the motors.
 * It reads joystick positions and button presses,
 * then tells the motors what to do.
 */
class TankController
{
private:
    // PS4 controller settings - What features to use
    int events;           // Should we track button press events?
    int buttons;          // Should we use controller buttons?
    int joysticks;        // Should we use the joysticks?
    int sensors;          // Should we use controller sensors?
    int useAccelerometer; // Should we use motion detection?
    int deadzone;         // How far to move joystick before robot responds
    int gyroDeadzone;     // Ignore small rotation movements
    int accDeadzone;      // Ignore small acceleration movements
    int accPrecision;     // How precise the motion readings should be

    // Motor settings - How the motors are connected and controlled
    int leftMotorPinA;           // Left motor forward pin
    int leftMotorPinB;           // Left motor backward pin
    int rightMotorPinA;          // Right motor forward pin
    int rightMotorPinB;          // Right motor backward pin
    float leftMotorCalibration;  // Left motor power factor
    float rightMotorCalibration; // Right motor power factor
    int maxMotorSpeed;           // Maximum motor speed (0-255)
    float turnSpeedFactor;       // How fast the robot turns

    // The objects that control our hardware
    PS4Remote ps4;       // Handles the PS4 controller
    MotionMotors motors; // Controls the robot's motors

    // Button memory - Remembers which buttons were pressed last time
    bool lastShareState;    // Was Share button pressed?
    bool lastUpState;       // Was D-pad Up pressed?
    bool lastDownState;     // Was D-pad Down pressed?
    bool lastTriangleState; // Was Triangle button pressed?
    bool lastXState;        // Was X button pressed?

    // Timing variables - When things last happened
    unsigned long lastUpdateTime;            // Last time we updated motors
    unsigned long lastButtonCheckTime;       // Last time we checked buttons
    unsigned long lastEmergencyStopTime;     // Last time we did emergency stop
    unsigned long lastShareButtonTime;       // Last time Share button was pressed
    unsigned long lastCalibrationButtonTime; // Last time we adjusted motor power

    // How much to change motor power each button press
    // 0.05 = 5% change each time you press a button
    const float CALIBRATION_STEP = 0.05;

    // How often things happen (in milliseconds)
    static const unsigned long UPDATE_INTERVAL;       // How often to update motors (20ms = 50 times per second)
    static const unsigned long BUTTON_CHECK_INTERVAL; // How often to check buttons (50ms = 20 times per second)
    static const unsigned long DEBOUNCE_TIME;         // How long to wait between button presses (200ms)

    // Helper functions that do specific jobs
    void controlMotorsWithJoystick(PS4Remote::ControllerState state); // Move motors based on joysticks
    void handleButtonControls(PS4Remote::ControllerState state);      // Handle button presses
    bool isTimeToUpdate();                                            // Check if it's time to update motors
    bool isTimeToCheckButtons();                                      // Check if it's time to check buttons

    // Special functions that run when controller connects/disconnects
    static void onConnect();    // Runs when controller connects
    static void onDisconnect(); // Runs when controller disconnects

    // A way to access this object from the special functions
    static TankController *instance;

public:
    /**
     * Creates a new robot controller with all the settings it needs
     *
     * This function sets up the robot's brain with all the necessary
     * information about the controller and motors.
     */
    TankController(
        // PS4 controller settings
        int events, int buttons, int joysticks, int sensors,
        int useAccelerometer, int deadzone, int gyroDeadzone,
        int accDeadzone, int accPrecision,
        // Motor connection settings
        int leftMotorPinA, int leftMotorPinB,
        int rightMotorPinA, int rightMotorPinB,
        float leftMotorCalibration, float rightMotorCalibration,
        int maxMotorSpeed, float turnSpeedFactor);

    /**
     * Starts up the robot controller
     *
     * This turns on Bluetooth and prepares the motors
     */
    void begin();

    /**
     * Updates the robot - reads controller and moves motors
     *
     * This function should be called over and over in the main loop
     */
    void update();

    /**
     * Changes how often the robot updates
     *
     * @param intervalMs How many milliseconds between updates
     */
    void setUpdateInterval(unsigned long intervalMs);

    /**
     * Tells you how often the robot updates
     *
     * @return How many milliseconds between updates
     */
    unsigned long getUpdateInterval() const;

    /**
     * Gets the current left motor power setting
     *
     * @return Left motor power factor (0.0 to 1.0)
     */
    float getLeftMotorCalibration() const;

    /**
     * Gets the current right motor power setting
     *
     * @return Right motor power factor (0.0 to 1.0)
     */
    float getRightMotorCalibration() const;
};

#endif // TANK_CONTROLLER_H
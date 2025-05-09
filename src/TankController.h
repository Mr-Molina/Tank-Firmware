#ifndef TANK_CONTROLLER_H
#define TANK_CONTROLLER_H

#include <Arduino.h>
#include "PS4-Remote.h"
#include "MotionMotors.h"

class TankController
{
private:
    // PS4 controller configuration
    int events;
    int buttons;
    int joysticks;
    int sensors;
    int useAccelerometer;
    int deadzone;
    int gyroDeadzone;
    int accDeadzone;
    int accPrecision;

    // Motor configuration
    int leftMotorPinA;
    int leftMotorPinB;
    int rightMotorPinA;
    int rightMotorPinB;
    float leftMotorCalibration;
    float rightMotorCalibration;
    int maxMotorSpeed;
    float turnSpeedFactor;

    // Controller and motor instances
    PS4Remote ps4;
    MotionMotors motors;

    // State tracking
    bool lastShareState;
    bool lastUpState;
    bool lastDownState;
    bool lastTriangleState;
    bool lastXState;

    // Timing control variables
    unsigned long lastUpdateTime;
    unsigned long lastButtonCheckTime;
    unsigned long lastEmergencyStopTime;
    unsigned long lastShareButtonTime;
    unsigned long lastCalibrationButtonTime;
    
    // Calibration adjustment step
    const float CALIBRATION_STEP = 0.05;

    // Default timing intervals (in milliseconds)
    static const unsigned long UPDATE_INTERVAL;       // 50Hz control loop
    static const unsigned long BUTTON_CHECK_INTERVAL; // 20Hz button polling
    static const unsigned long DEBOUNCE_TIME;         // Button debounce time

    // Private methods
    void controlMotorsWithJoystick(PS4Remote::ControllerState state);
    void handleButtonControls(PS4Remote::ControllerState state);
    bool isTimeToUpdate();
    bool isTimeToCheckButtons();

    // Static callback methods
    static void onConnect();
    static void onDisconnect();

    // Static instance pointer for callbacks
    static TankController *instance;

public:
    TankController(
        // PS4 controller config
        int events, int buttons, int joysticks, int sensors,
        int useAccelerometer, int deadzone, int gyroDeadzone,
        int accDeadzone, int accPrecision,
        // Motor config
        int leftMotorPinA, int leftMotorPinB,
        int rightMotorPinA, int rightMotorPinB,
        float leftMotorCalibration, float rightMotorCalibration,
        int maxMotorSpeed, float turnSpeedFactor);

    void begin();
    void update();

    /**
     * Sets a custom update interval for the control loop
     *
     * @param intervalMs Interval in milliseconds between updates
     */
    void setUpdateInterval(unsigned long intervalMs);

    /**
     * Gets the current update interval
     *
     * @return Current update interval in milliseconds
     */
    unsigned long getUpdateInterval() const;
    
    /**
     * Gets the current left motor calibration value
     * 
     * @return Current left motor calibration factor
     */
    float getLeftMotorCalibration() const;
    
    /**
     * Gets the current right motor calibration value
     * 
     * @return Current right motor calibration factor
     */
    float getRightMotorCalibration() const;
};

#endif // TANK_CONTROLLER_H
#ifndef MOTION_MOTORS_H
#define MOTION_MOTORS_H

#include <Arduino.h>

// Default configuration values
#define DEFAULT_SMOOTH_ENABLED true
#define SMOOTH_ACCEL_STEPS 10
#define SMOOTH_ACCEL_DELAY 20

// Motor calibration factors (adjust these to match your specific motors)
#define LEFT_CALIBRATION 1.0
#define RIGHT_CALIBRATION 1.0

// Motor pin definitions (adjust these to match your wiring)
#define M_LEFT_A_PIN 5   // Left motor pin A
#define M_LEFT_B_PIN 6   // Left motor pin B
#define M_RIGHT_A_PIN 9  // Right motor pin A
#define M_RIGHT_B_PIN 10 // Right motor pin B

/**
 * MotionMotors class for controlling a vehicle with 2 motors
 * 
 * This class provides independent control of left and right motors
 * with calibration and smooth acceleration/deceleration capabilities.
 */
class MotionMotors {
public:
    /**
     * Constructor
     */
    MotionMotors();

    /**
     * Initialize the motors
     * 
     * Sets up the motor pins and initializes the state
     */
    void begin();

    /**
     * Independent motor control functions
     */
    
    /**
     * Drives the left motor forward with optional smooth acceleration
     * 
     * @param power Motor power level (0-255)
     * @param smooth Whether to use smooth acceleration (if enabled globally)
     */
    void leftForward(uint8_t power, bool smooth = true);

    /**
     * Drives the left motor backward with optional smooth acceleration
     * 
     * @param power Motor power level (0-255)
     * @param smooth Whether to use smooth acceleration (if enabled globally)
     */
    void leftBackward(uint8_t power, bool smooth = true);

    /**
     * Drives the right motor forward with optional smooth acceleration
     * 
     * @param power Motor power level (0-255)
     * @param smooth Whether to use smooth acceleration (if enabled globally)
     */
    void rightForward(uint8_t power, bool smooth = true);

    /**
     * Drives the right motor backward with optional smooth acceleration
     * 
     * @param power Motor power level (0-255)
     * @param smooth Whether to use smooth acceleration (if enabled globally)
     */
    void rightBackward(uint8_t power, bool smooth = true);

    /**
     * Stops the left motor
     */
    void leftStop();

    /**
     * Stops the right motor
     */
    void rightStop();

    /**
     * Stops all motors immediately
     * 
     * Sets all motor control pins to 0, causing the vehicle to coast to a stop.
     * Note: This is not an active brake - the motors will coast rather than
     * actively resist motion.
     */
    void stop();

    /**
     * Enable or disable smooth acceleration/deceleration
     * 
     * @param enable True to enable, false to disable
     */
    void setSmoothEnabled(bool enable);

    /**
     * Check if smooth acceleration/deceleration is enabled
     * 
     * @return True if enabled, false if disabled
     */
    bool isSmoothEnabled();

private:
    // Low-level motor control functions
    
    /**
     * Drives the left motor forward
     * 
     * @param power Motor power level (0-255)
     */
    void left_forward(uint8_t power);

    /**
     * Drives the left motor backward
     * 
     * @param power Motor power level (0-255)
     */
    void left_backward(uint8_t power);

    /**
     * Stops the left motor
     */
    void left_stop();

    /**
     * Drives the right motor forward
     * 
     * @param power Motor power level (0-255)
     */
    void right_forward(uint8_t power);

    /**
     * Drives the right motor backward
     * 
     * @param power Motor power level (0-255)
     */
    void right_backward(uint8_t power);

    /**
     * Stops the right motor
     */
    void right_stop();

    // Smooth acceleration/deceleration functions
    
    /**
     * Gradually accelerate to target power
     * 
     * @param moveFunction Pointer to the movement function to use
     * @param targetPower Target power level (0-255)
     * @param steps Number of steps for acceleration
     * @param delayMs Delay between steps in milliseconds
     */
    void smoothAccelerate(void (MotionMotors::*moveFunction)(uint8_t), uint8_t targetPower,
                          uint8_t steps = SMOOTH_ACCEL_STEPS, uint8_t delayMs = SMOOTH_ACCEL_DELAY);

    /**
     * Gradually decelerate to a stop
     * 
     * @param moveFunction Pointer to the movement function currently in use
     * @param currentPower Current power level (0-255)
     * @param steps Number of steps for deceleration
     * @param delayMs Delay between steps in milliseconds
     */
    void smoothDecelerate(void (MotionMotors::*moveFunction)(uint8_t), uint8_t currentPower,
                          uint8_t steps = SMOOTH_ACCEL_STEPS, uint8_t delayMs = SMOOTH_ACCEL_DELAY);

    /**
     * Smoothly transition between two movement functions
     * 
     * @param currentFunction Pointer to the current movement function
     * @param currentPower Current power level (0-255)
     * @param targetFunction Pointer to the target movement function
     * @param targetPower Target power level (0-255)
     * @param steps Number of steps for transition
     * @param delayMs Delay between steps in milliseconds
     */
    void smoothTransition(void (MotionMotors::*currentFunction)(uint8_t), uint8_t currentPower,
                          void (MotionMotors::*targetFunction)(uint8_t), uint8_t targetPower,
                          uint8_t steps = SMOOTH_ACCEL_STEPS, uint8_t delayMs = SMOOTH_ACCEL_DELAY);

    // Member variables for tracking current movement state for each motor
    void (MotionMotors::*leftCurrentFunction)(uint8_t);
    void (MotionMotors::*rightCurrentFunction)(uint8_t);
    uint8_t leftCurrentPower;
    uint8_t rightCurrentPower;
    bool smoothEnabled;
};

#endif // MOTION_MOTORS_H
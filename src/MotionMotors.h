#ifndef MOTION_MOTORS_H
#define MOTION_MOTORS_H

#include <Arduino.h>

// Default configuration values
#define DEFAULT_SMOOTH_ENABLED true
#define SMOOTH_ACCEL_STEPS 10
#define SMOOTH_ACCEL_DELAY 20

// Motor calibration factors (adjust these to match your specific motors)
#define RL_CALIBRATION 1.0
#define RR_CALIBRATION 1.0

// Motor pin definitions (adjust these to match your wiring)
#define M_RL_A_PIN 5  // Rear Left motor pin A
#define M_RL_B_PIN 6  // Rear Left motor pin B
#define M_RR_A_PIN 9  // Rear Right motor pin A
#define M_RR_B_PIN 10 // Rear Right motor pin B

/**
 * MotionMotors class for controlling a vehicle with 2 rear motors
 * 
 * This class provides high-level movement functions for a tank-like vehicle
 * with two rear motors. It includes smooth acceleration/deceleration and
 * motor calibration to ensure straight movement.
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
     * High-level movement functions
     */
    
    /**
     * Moves the vehicle straight forward
     * 
     * Both rear wheels rotate forward at the same speed, causing the vehicle to move
     * in a straight line forward. The calibration factors ensure that both wheels
     * move at the same effective speed even if the motors have slight differences.
     * 
     * @param power Motor power level (0-255)
     * @param smooth Whether to use smooth acceleration (if enabled globally)
     */
    void forward(uint8_t power, bool smooth = true);

    /**
     * Moves the vehicle straight backward
     * 
     * Both rear wheels rotate backward at the same speed, causing the vehicle to move
     * in a straight line backward. The calibration factors ensure that both wheels
     * move at the same effective speed even if the motors have slight differences.
     * 
     * @param power Motor power level (0-255)
     * @param smooth Whether to use smooth acceleration (if enabled globally)
     */
    void backward(uint8_t power, bool smooth = true);

    /**
     * Rotates the vehicle to the left (counter-clockwise) in place
     * 
     * This creates a pivot turn where the vehicle rotates around its center.
     * The left wheel rotates backward while the right wheel rotates forward,
     * causing the vehicle to spin in place counter-clockwise.
     * 
     * @param power Motor power level (0-255)
     */
    void left(uint8_t power);

    /**
     * Rotates the vehicle to the right (clockwise) in place
     * 
     * This creates a pivot turn where the vehicle rotates around its center.
     * The right wheel rotates backward while the left wheel rotates forward,
     * causing the vehicle to spin in place clockwise.
     * 
     * @param power Motor power level (0-255)
     */
    void right(uint8_t power);

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
     * Drives the Rear Left motor forward
     * 
     * @param power Motor power level (0-255)
     */
    void RL_forward(uint8_t power);

    /**
     * Drives the Rear Left motor backward
     * 
     * @param power Motor power level (0-255)
     */
    void RL_backward(uint8_t power);

    /**
     * Stops the Rear Left motor
     */
    void RL_Stop();

    /**
     * Drives the Rear Right motor forward
     * 
     * @param power Motor power level (0-255)
     */
    void RR_forward(uint8_t power);

    /**
     * Drives the Rear Right motor backward
     * 
     * @param power Motor power level (0-255)
     */
    void RR_backward(uint8_t power);

    /**
     * Stops the Rear Right motor
     */
    void RR_Stop();

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

    // Member variables for tracking current movement state
    void (MotionMotors::*currentMoveFunction)(uint8_t);
    uint8_t currentPower;
    bool smoothEnabled;
};

#endif // MOTION_MOTORS_H
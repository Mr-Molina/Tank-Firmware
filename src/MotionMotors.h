#ifndef MOTION_MOTORS_H
#define MOTION_MOTORS_H

#include <Arduino.h>

// Default settings - These are the starting values
#define DEFAULT_SMOOTH_ENABLED true   // Smooth acceleration is on by default
#define SMOOTH_ACCEL_STEPS 10         // How many small steps to take when speeding up
#define SMOOTH_ACCEL_DELAY 20         // How many milliseconds between each step

// Default motor power settings (you can adjust these if one motor is stronger than the other)
#define DEFAULT_LEFT_CALIBRATION 1.0  // Left motor at 100% power
#define DEFAULT_RIGHT_CALIBRATION 1.0 // Right motor at 100% power

/**
 * MotionMotors class - This controls the robot's wheels
 * 
 * This is like the robot's legs - it handles moving the left and right wheels
 * independently, making the robot drive forward, backward, or turn.
 */
class MotionMotors {
public:
    /**
     * Constructor - Sets up a new MotionMotors object
     * 
     * This is like building the robot's legs before using them.
     * We need to know which pins control which motors.
     * 
     * @param leftA Left motor forward pin
     * @param leftB Left motor backward pin
     * @param rightA Right motor forward pin
     * @param rightB Right motor backward pin
     * @param leftCalibration How strong the left motor should be (0.0-1.0)
     * @param rightCalibration How strong the right motor should be (0.0-1.0)
     */
    MotionMotors(uint8_t leftA, uint8_t leftB, uint8_t rightA, uint8_t rightB, 
                 float leftCalibration = DEFAULT_LEFT_CALIBRATION, 
                 float rightCalibration = DEFAULT_RIGHT_CALIBRATION);

    /**
     * Begin - Gets the motors ready to use
     * 
     * This sets up the motor pins and makes sure everything is ready to go
     */
    void begin();

    /**
     * Motor control functions - These make the robot move
     */
    
    /**
     * Makes the left wheel go forward
     * 
     * @param power How fast to spin (0-255, where 0 is stopped and 255 is max speed)
     * @param smooth Whether to speed up gradually (true) or instantly (false)
     */
    void leftForward(uint8_t power, bool smooth = true);

    /**
     * Makes the left wheel go backward
     * 
     * @param power How fast to spin (0-255)
     * @param smooth Whether to speed up gradually
     */
    void leftBackward(uint8_t power, bool smooth = true);

    /**
     * Makes the right wheel go forward
     * 
     * @param power How fast to spin (0-255)
     * @param smooth Whether to speed up gradually
     */
    void rightForward(uint8_t power, bool smooth = true);

    /**
     * Makes the right wheel go backward
     * 
     * @param power How fast to spin (0-255)
     * @param smooth Whether to speed up gradually
     */
    void rightBackward(uint8_t power, bool smooth = true);

    /**
     * Stops the left wheel
     */
    void leftStop();

    /**
     * Stops the right wheel
     */
    void rightStop();

    /**
     * Stops both wheels immediately
     * 
     * This is like hitting the brakes - both wheels stop spinning.
     * The robot will coast to a stop rather than stopping instantly.
     */
    void stop();

    /**
     * Turn smooth acceleration on or off
     * 
     * Smooth acceleration makes the robot start and stop more gradually,
     * like a car instead of a bumper car.
     * 
     * @param enable True to turn it on, false to turn it off
     */
    void setSmoothEnabled(bool enable);

    /**
     * Check if smooth acceleration is turned on
     * 
     * @return True if it's on, false if it's off
     */
    bool isSmoothEnabled();
    
    /**
     * Update acceleration - Makes smooth speed changes happen
     * 
     * This needs to be called over and over in the main loop
     * for smooth acceleration to work properly.
     */
    void updateAcceleration();
    
    /**
     * Set how powerful the left motor should be
     * 
     * This is useful if one motor is stronger than the other
     * and you want to balance them.
     * 
     * @param calibration Power factor (0.0 to 1.0, where 1.0 is 100% power)
     */
    void setLeftCalibration(float calibration);
    
    /**
     * Set how powerful the right motor should be
     * 
     * @param calibration Power factor (0.0 to 1.0)
     */
    void setRightCalibration(float calibration);
    
    /**
     * Get the current left motor power setting
     * 
     * @return Left motor power factor (0.0 to 1.0)
     */
    float getLeftCalibration() const;
    
    /**
     * Get the current right motor power setting
     * 
     * @return Right motor power factor (0.0 to 1.0)
     */
    float getRightCalibration() const;

private:
    // These are the behind-the-scenes functions that actually control the motors
    
    /**
     * Makes the left motor spin forward
     * 
     * @param power Motor speed (0-255)
     */
    void left_forward(uint8_t power);

    /**
     * Makes the left motor spin backward
     * 
     * @param power Motor speed (0-255)
     */
    void left_backward(uint8_t power);

    /**
     * Stops the left motor
     */
    void left_stop();

    /**
     * Makes the right motor spin forward
     * 
     * @param power Motor speed (0-255)
     */
    void right_forward(uint8_t power);

    /**
     * Makes the right motor spin backward
     * 
     * @param power Motor speed (0-255)
     */
    void right_backward(uint8_t power);

    /**
     * Stops the right motor
     */
    void right_stop();

    // Functions for smooth acceleration
    
    /**
     * Starts a smooth acceleration sequence
     */
    void startAcceleration(
        void (MotionMotors::*leftFunc)(uint8_t), uint8_t leftPower,
        void (MotionMotors::*rightFunc)(uint8_t), uint8_t rightPower,
        uint8_t steps);
    
    /**
     * Gradually speeds up a motor
     * 
     * This is like pressing the gas pedal slowly instead of stomping on it.
     * 
     * @param moveFunction Which motor and direction to move
     * @param targetPower Final speed to reach (0-255)
     * @param steps How many small steps to take when speeding up
     * @param delayMs How many milliseconds between each step
     */
    void smoothAccelerate(void (MotionMotors::*moveFunction)(uint8_t), uint8_t targetPower,
                          uint8_t steps = SMOOTH_ACCEL_STEPS, uint8_t delayMs = SMOOTH_ACCEL_DELAY);

    /**
     * Gradually slows down a motor
     * 
     * This is like easing off the gas pedal instead of letting go suddenly.
     * 
     * @param moveFunction Which motor is currently moving
     * @param currentPower Current speed (0-255)
     * @param steps How many small steps to take when slowing down
     * @param delayMs How many milliseconds between each step
     */
    void smoothDecelerate(void (MotionMotors::*moveFunction)(uint8_t), uint8_t currentPower,
                          uint8_t steps = SMOOTH_ACCEL_STEPS, uint8_t delayMs = SMOOTH_ACCEL_DELAY);

    /**
     * Smoothly changes a motor from one direction/speed to another
     * 
     * This is like changing from driving forward to backward smoothly.
     * 
     * @param currentFunction Current movement direction
     * @param currentPower Current speed (0-255)
     * @param targetFunction New movement direction
     * @param targetPower New speed (0-255)
     * @param steps How many small steps to take during the change
     * @param delayMs How many milliseconds between each step
     */
    void smoothTransition(void (MotionMotors::*currentFunction)(uint8_t), uint8_t currentPower,
                          void (MotionMotors::*targetFunction)(uint8_t), uint8_t targetPower,
                          uint8_t steps = SMOOTH_ACCEL_STEPS, uint8_t delayMs = SMOOTH_ACCEL_DELAY);

    // Variables to keep track of what each motor is doing
    void (MotionMotors::*leftCurrentFunction)(uint8_t);  // What the left motor is doing
    void (MotionMotors::*rightCurrentFunction)(uint8_t); // What the right motor is doing
    uint8_t leftCurrentPower;   // How fast the left motor is going
    uint8_t rightCurrentPower;  // How fast the right motor is going
    bool smoothEnabled;         // Is smooth acceleration turned on?
    
    // Variables for smooth acceleration
    unsigned long lastAccelUpdateTime;  // When we last updated the acceleration
    void (MotionMotors::*leftTargetFunction)(uint8_t);   // What we want the left motor to do
    void (MotionMotors::*rightTargetFunction)(uint8_t);  // What we want the right motor to do
    uint8_t targetLeftPower;    // How fast we want the left motor to go
    uint8_t targetRightPower;   // How fast we want the right motor to go
    uint8_t currentAccelStep;   // Which step we're on in the acceleration
    uint8_t totalAccelSteps;    // Total number of steps to take
    bool isAccelerating;        // Are we currently accelerating?
    
    // Motor pin configuration - which pins control which motors
    uint8_t M_LEFT_A_PIN;   // Left motor forward pin
    uint8_t M_LEFT_B_PIN;   // Left motor backward pin
    uint8_t M_RIGHT_A_PIN;  // Right motor forward pin
    uint8_t M_RIGHT_B_PIN;  // Right motor backward pin
    
    // Motor calibration - how powerful each motor should be
    float LEFT_CALIBRATION;   // Left motor power factor (0.0-1.0)
    float RIGHT_CALIBRATION;  // Right motor power factor (0.0-1.0)
    
    // Variables to track what we last reported to the computer
    uint8_t lastReportedLeftPower;   // Last reported left motor power
    uint8_t lastReportedRightPower;  // Last reported right motor power
    void (MotionMotors::*lastReportedLeftFunction)(uint8_t);   // Last reported left motor direction
    void (MotionMotors::*lastReportedRightFunction)(uint8_t);  // Last reported right motor direction
};

#endif // MOTION_MOTORS_H
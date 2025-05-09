#include "TankController.h"

// Initialize static instance pointer and constants
TankController* TankController::instance = nullptr;

// How often we update different parts of the program:
const unsigned long TankController::UPDATE_INTERVAL = 20;       // Update motors 50 times per second (every 20ms)
const unsigned long TankController::BUTTON_CHECK_INTERVAL = 50; // Check buttons 20 times per second (every 50ms)
const unsigned long TankController::DEBOUNCE_TIME = 200;        // Wait 200ms between button presses to avoid accidental double-presses

// This function creates a new robot controller with all the settings it needs
TankController::TankController(
    // PS4 controller settings - what features to use
    int events, int buttons, int joysticks, int sensors,
    int useAccelerometer, int deadzone, int gyroDeadzone,
    int accDeadzone, int accPrecision,
    // Motor connection settings - which pins connect where
    int leftMotorPinA, int leftMotorPinB,
    int rightMotorPinA, int rightMotorPinB,
    float leftMotorCalibration, float rightMotorCalibration,
    int maxMotorSpeed, float turnSpeedFactor
) : 
    // Initialize member variables
    events(events),
    buttons(buttons),
    joysticks(joysticks),
    sensors(sensors),
    useAccelerometer(useAccelerometer),
    deadzone(deadzone),
    gyroDeadzone(gyroDeadzone),
    accDeadzone(accDeadzone),
    accPrecision(accPrecision),
    leftMotorPinA(leftMotorPinA),
    leftMotorPinB(leftMotorPinB),
    rightMotorPinA(rightMotorPinA),
    rightMotorPinB(rightMotorPinB),
    leftMotorCalibration(leftMotorCalibration),
    rightMotorCalibration(rightMotorCalibration),
    maxMotorSpeed(maxMotorSpeed),
    turnSpeedFactor(turnSpeedFactor),
    // Initialize motors with pins and calibration
    motors(
        leftMotorPinA, leftMotorPinB,
        rightMotorPinA, rightMotorPinB,
        leftMotorCalibration, rightMotorCalibration
    ),
    lastShareState(false),
    lastUpState(false),
    lastDownState(false),
    lastTriangleState(false),
    lastXState(false),
    // Initialize timing variables
    lastUpdateTime(0),
    lastButtonCheckTime(0),
    lastEmergencyStopTime(0),
    lastShareButtonTime(0),
    lastCalibrationButtonTime(0)
{
    // Store instance pointer for callbacks
    instance = this;
}

// This function starts up the robot controller
void TankController::begin() {
    // Tell the PS4 controller what to do when it connects or disconnects
    PS4.attachOnConnect(onConnect);         // Run our onConnect function when controller connects
    PS4.attachOnDisconnect(onDisconnect);   // Run our onDisconnect function when controller disconnects
    
    // Start the PS4 controller (turns on Bluetooth)
    ps4.begin();
    
    // Start the motors
    motors.begin();
    
    // Set up all our timing variables with the current time
    // This helps us keep track of when things last happened
    unsigned long currentTime = millis();  // Get the current time in milliseconds
    lastUpdateTime = currentTime;          // Time of last motor update
    lastButtonCheckTime = currentTime;     // Time of last button check
    lastEmergencyStopTime = currentTime;   // Time of last emergency stop
    lastShareButtonTime = currentTime;     // Time of last Share button press
    lastCalibrationButtonTime = currentTime; // Time of last calibration adjustment
    
    Serial.println("Robot controller is ready!");
}

// This function updates the robot - called repeatedly in the main loop
void TankController::update() {
    // Get the current state of all buttons and joysticks
    PS4Remote::ControllerState state = ps4.getState();
    
    // Update any smooth motor speed changes that are happening
    motors.updateAcceleration();
    
    // Only update the motors 50 times per second (every 20ms)
    // This prevents the program from trying to update too quickly
    if (isTimeToUpdate()) {
        // If the controller is connected, control the motors with the joysticks
        if (PS4.isConnected()) {
            controlMotorsWithJoystick(state);
        } else {
            // If no controller is connected, make sure motors are stopped
            // This is a safety feature to prevent the robot from running away!
            motors.stop();
        }
        
        // Give the ESP32 chip a chance to do other important tasks
        yield();
    }
    
    // Check buttons 20 times per second (every 50ms)
    // We don't need to check buttons as often as we update motors
    if (isTimeToCheckButtons() && PS4.isConnected()) {
        handleButtonControls(state);
    }
}

// This function handles all the button presses on the controller
void TankController::handleButtonControls(PS4Remote::ControllerState state) {
    // Get the current time so we know when buttons were pressed
    unsigned long currentTime = millis();
    
    // EMERGENCY STOP BUTTON
    // The Options button acts as an emergency stop
    // We also check that enough time has passed since the last press (debounce)
    if (state.options && (currentTime - lastEmergencyStopTime > DEBOUNCE_TIME)) {
        // Stop all motors immediately!
        motors.stop();
        
        // Show a message on the computer
        Serial.println("EMERGENCY STOP - All motors stopped!");
        
        // Remember when we did the emergency stop
        lastEmergencyStopTime = currentTime;
    }
    
    // SMOOTH ACCELERATION TOGGLE
    // The Share button turns smooth acceleration on or off
    // Smooth acceleration makes the robot start and stop more gradually
    
    // Check if Share button is pressed, wasn't pressed before, and enough time has passed
    if (state.share && !lastShareState && (currentTime - lastShareButtonTime > DEBOUNCE_TIME)) {
        // Toggle smooth acceleration (turn it on if it's off, or off if it's on)
        bool smooth = !motors.isSmoothEnabled();
        motors.setSmoothEnabled(smooth);
        
        // Show a message about what changed
        Serial.print("Smooth acceleration: ");
        Serial.println(smooth ? "ON - Robot will accelerate gradually" : "OFF - Robot will respond instantly");
        
        // Remember when we pressed the button
        lastShareButtonTime = currentTime;
    }
    // Remember the current state of the Share button for next time
    lastShareState = state.share;
    
    // These buttons let you adjust how powerful each motor is compared to the other
    // This helps balance the robot if one motor is stronger than the other
    if (currentTime - lastCalibrationButtonTime > DEBOUNCE_TIME) {
        // D-pad Up: Make the left motor stronger
        if (state.up && !lastUpState) {
            // Get the current power level (from 0.0 to 1.0)
            float currentCalibration = motors.getLeftCalibration();
            
            // Increase it a little bit (but don't go above 1.0)
            float newCalibration = min(currentCalibration + CALIBRATION_STEP, 1.0f);
            
            // Apply the new setting
            motors.setLeftCalibration(newCalibration);
            
            // Show a message about what changed
            extern int DEBUG_CALIBRATION_VALUE;
            if (DEBUG_CALIBRATION_VALUE) {
                Serial.printf("LEFT MOTOR POWER: %.2f -> %.2f\n", currentCalibration, newCalibration);
            }
            lastCalibrationButtonTime = currentTime;
        }
        
        // D-pad Down: Make the left motor weaker
        if (state.down && !lastDownState) {
            // Get the current power level
            float currentCalibration = motors.getLeftCalibration();
            
            // Decrease it a little bit (but don't go below 0.0)
            float newCalibration = max(currentCalibration - CALIBRATION_STEP, 0.0f);
            
            // Apply the new setting
            motors.setLeftCalibration(newCalibration);
            
            // Show a message about what changed
            extern int DEBUG_CALIBRATION_VALUE;
            if (DEBUG_CALIBRATION_VALUE) {
                Serial.printf("LEFT MOTOR POWER: %.2f -> %.2f\n", currentCalibration, newCalibration);
            }
            lastCalibrationButtonTime = currentTime;
        }
        
        // Triangle button: Make the right motor stronger
        if (state.triangle && !lastTriangleState) {
            // Get the current power level
            float currentCalibration = motors.getRightCalibration();
            
            // Increase it a little bit (but don't go above 1.0)
            float newCalibration = min(currentCalibration + CALIBRATION_STEP, 1.0f);
            
            // Apply the new setting
            motors.setRightCalibration(newCalibration);
            
            // Show a message about what changed
            extern int DEBUG_CALIBRATION_VALUE;
            if (DEBUG_CALIBRATION_VALUE) {
                Serial.printf("RIGHT MOTOR POWER: %.2f -> %.2f\n", currentCalibration, newCalibration);
            }
            lastCalibrationButtonTime = currentTime;
        }
        
        // X button: Make the right motor weaker
        if (state.cross && !lastXState) {
            // Get the current power level
            float currentCalibration = motors.getRightCalibration();
            
            // Decrease it a little bit (but don't go below 0.0)
            float newCalibration = max(currentCalibration - CALIBRATION_STEP, 0.0f);
            
            // Apply the new setting
            motors.setRightCalibration(newCalibration);
            
            // Show a message about what changed
            extern int DEBUG_CALIBRATION_VALUE;
            if (DEBUG_CALIBRATION_VALUE) {
                Serial.printf("RIGHT MOTOR POWER: %.2f -> %.2f\n", currentCalibration, newCalibration);
            }
            lastCalibrationButtonTime = currentTime;
        }
    }
    
    // Update button states
    lastUpState = state.up;
    lastDownState = state.down;
    lastTriangleState = state.triangle;
    lastXState = state.cross;
}

// This function checks if enough time has passed to update the motors
// We use this to make sure we don't update too frequently
bool TankController::isTimeToUpdate() {
    // Get the current time
    unsigned long currentTime = millis();
    
    // Check if enough time has passed since the last update
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
        // Remember when we did this update
        lastUpdateTime = currentTime;
        // Tell the program it's time to update
        return true;
    }
    
    // Not enough time has passed yet
    return false;
}

// This function checks if enough time has passed to check button states
// We use this to make sure we don't check buttons too frequently
bool TankController::isTimeToCheckButtons() {
    // Get the current time
    unsigned long currentTime = millis();
    
    // Check if enough time has passed since the last button check
    if (currentTime - lastButtonCheckTime >= BUTTON_CHECK_INTERVAL) {
        // Remember when we did this check
        lastButtonCheckTime = currentTime;
        // Tell the program it's time to check buttons
        return true;
    }
    
    // Not enough time has passed yet
    return false;
}

// This function tries to change how often the robot updates
// (Currently this doesn't actually change anything - it just shows a message)
void TankController::setUpdateInterval(unsigned long intervalMs) {
    // We can't actually change the update interval because it's fixed
    // So instead, we just show a message explaining that
    Serial.print("The robot updates every ");
    Serial.print(UPDATE_INTERVAL);
    Serial.println(" milliseconds (50 times per second)");
}

// This function tells you how often the robot updates
unsigned long TankController::getUpdateInterval() const {
    // Return the current update interval (20 milliseconds = 50 times per second)
    return UPDATE_INTERVAL;
}

// This function tells you the current power setting for the left motor
float TankController::getLeftMotorCalibration() const {
    // Get the left motor power factor (between 0.0 and 1.0)
    // 1.0 means full power, 0.5 means half power, etc.
    return motors.getLeftCalibration();
}

// This function tells you the current power setting for the right motor
float TankController::getRightMotorCalibration() const {
    // Get the right motor power factor (between 0.0 and 1.0)
    // 1.0 means full power, 0.5 means half power, etc.
    return motors.getRightCalibration();
}

// This function controls the motors based on the joystick positions
// It uses "tank drive" style where:
// - Left joystick controls the left motor
// - Right joystick controls the right motor
void TankController::controlMotorsWithJoystick(PS4Remote::ControllerState state) {
    // Get joystick positions
    // We flip the Y values so pushing up (negative Y) makes the robot go forward
    int leftY = -state.ly;   // Left joystick up/down position
    int rightY = -state.ry;  // Right joystick up/down position
    
    // Start with motors stopped (speed = 0)
    int leftMotorSpeed = 0;
    int rightMotorSpeed = 0;

    // Only respond to the left joystick if it's moved enough
    // (The deadzone helps ignore tiny movements when the joystick is at rest)
    if (abs(leftY) > deadzone) {
        // Convert joystick position (deadzone to 127) to motor speed (0 to maxMotorSpeed)
        leftMotorSpeed = map(abs(leftY), deadzone, 127, 0, maxMotorSpeed);
        
        // Set direction: positive = forward, negative = backward
        leftMotorSpeed = (leftY > 0) ? leftMotorSpeed : -leftMotorSpeed;
    }

    // Only respond to the right joystick if it's moved enough
    if (abs(rightY) > deadzone) {
        // Convert joystick position to motor speed
        rightMotorSpeed = map(abs(rightY), deadzone, 127, 0, maxMotorSpeed);
        
        // Set direction: positive = forward, negative = backward
        rightMotorSpeed = (rightY > 0) ? rightMotorSpeed : -rightMotorSpeed;
    }

    // Make sure speeds stay within our limits (-255 to 255)
    leftMotorSpeed = constrain(leftMotorSpeed, -maxMotorSpeed, maxMotorSpeed);
    rightMotorSpeed = constrain(rightMotorSpeed, -maxMotorSpeed, maxMotorSpeed);
    
    // Control the left motor based on its speed
    if (leftMotorSpeed > 0) {
        // Positive speed = go forward
        motors.leftForward(leftMotorSpeed);
    }
    else if (leftMotorSpeed < 0) {
        // Negative speed = go backward (we use abs() to make the number positive)
        motors.leftBackward(abs(leftMotorSpeed));
    }
    else {
        // Zero speed = stop
        motors.leftStop();
    }
    
    // Control the right motor based on its speed
    if (rightMotorSpeed > 0) {
        // Positive speed = go forward
        motors.rightForward(rightMotorSpeed);
    }
    else if (rightMotorSpeed < 0) {
        // Negative speed = go backward
        motors.rightBackward(abs(rightMotorSpeed));
    }
    else {
        // Zero speed = stop
        motors.rightStop();
    }
}

// This function runs automatically when the controller connects
void TankController::onConnect() {
    // Show a message that the controller connected
    Serial.println("\n*** PS4 Controller Connected! ***");
    Serial.println("You can now control the robot with the joysticks.");
    
    // Reset the disconnect flag so we'll show a message if it disconnects again
    extern bool wasDisconnected;
    wasDisconnected = false;
}

// This variable remembers if we've already shown a disconnect message
// We use it to avoid showing the same message over and over
bool wasDisconnected = false;

// This function runs automatically when the controller disconnects
void TankController::onDisconnect() {
    // Only show the disconnect message once (not repeatedly)
    if (!wasDisconnected) {
        // Show a warning message that the controller disconnected
        Serial.println("\n*** WARNING: PS4 Controller Disconnected! ***");
        Serial.println("The robot will stop moving until the controller reconnects.");
        
        // If debug messages are turned on, show an emergency stop message
        extern int DEBUG_MOTOR_ACTIONS_VALUE;
        if (DEBUG_MOTOR_ACTIONS_VALUE) {
            Serial.println("MOTOR: EMERGENCY STOP (CONTROLLER DISCONNECTED)");
        }
        
        // Remember that we've shown the message
        // This way we won't keep showing it over and over
        wasDisconnected = true;
    }
    
    // SAFETY FEATURE: Stop all motors when the controller disconnects
    // This prevents the robot from running away if you lose connection!
    if (instance) {
        // Stop all motors immediately
        instance->motors.stop();
    }
}
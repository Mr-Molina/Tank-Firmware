#include "TankController.h"

// Initialize static instance pointer and constants
TankController* TankController::instance = nullptr;
const unsigned long TankController::UPDATE_INTERVAL = 20;      // 50Hz control loop
const unsigned long TankController::BUTTON_CHECK_INTERVAL = 50; // 20Hz button polling
const unsigned long TankController::DEBOUNCE_TIME = 200;       // Button debounce time

// Constructor
TankController::TankController(
    // PS4 controller config
    int events, int buttons, int joysticks, int sensors,
    int useAccelerometer, int deadzone, int gyroDeadzone,
    int accDeadzone, int accPrecision,
    // Motor config
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

// Initialize controller and motors
void TankController::begin() {
    // Set custom callbacks
    PS4.attachOnConnect(onConnect);
    PS4.attachOnDisconnect(onDisconnect);
    
    // Initialize PS4 controller
    ps4.begin();
    
    // Initialize motors
    motors.begin();
    
    // Initialize timing variables with current time
    unsigned long currentTime = millis();
    lastUpdateTime = currentTime;
    lastButtonCheckTime = currentTime;
    lastEmergencyStopTime = currentTime;
    lastShareButtonTime = currentTime;
    lastCalibrationButtonTime = currentTime;
    
    Serial.println("TankController initialized with timing control");
}

// Update function called in main loop
void TankController::update() {
    // Get current controller state
    PS4Remote::ControllerState state = ps4.getState();
    
    // Update motor acceleration (non-blocking)
    motors.updateAcceleration();
    
    // Only update motor control at the specified interval
    if (isTimeToUpdate()) {
        // Control motors based on joystick input if controller is connected
        if (PS4.isConnected()) {
            controlMotorsWithJoystick(state);
        } else {
            // No controller connected, ensure motors are stopped
            motors.stop();
        }
        
        // Yield to allow ESP32 background tasks to run
        yield();
    }
    
    // Check buttons at a different rate to avoid overwhelming the system
    if (isTimeToCheckButtons() && PS4.isConnected()) {
        handleButtonControls(state);
    }
}

// Handle button controls separately from motor control
void TankController::handleButtonControls(PS4Remote::ControllerState state) {
    unsigned long currentTime = millis();
    
    // Handle special button controls
    if (state.options && (currentTime - lastEmergencyStopTime > DEBOUNCE_TIME)) {
        // Options button: Emergency stop
        motors.stop();
        Serial.println("EMERGENCY STOP");
        lastEmergencyStopTime = currentTime;
    }
    
    // Toggle smooth acceleration with Share button
    if (state.share && !lastShareState && (currentTime - lastShareButtonTime > DEBOUNCE_TIME)) {
        bool smooth = !motors.isSmoothEnabled();
        motors.setSmoothEnabled(smooth);
        Serial.print("Smooth acceleration: ");
        Serial.println(smooth ? "ENABLED" : "DISABLED");
        lastShareButtonTime = currentTime;
    }
    lastShareState = state.share;
    
    // Handle motor calibration adjustments
    if (currentTime - lastCalibrationButtonTime > DEBOUNCE_TIME) {
        // D-pad Up: Increase left motor calibration
        if (state.up && !lastUpState) {
            float currentCalibration = motors.getLeftCalibration();
            float newCalibration = min(currentCalibration + CALIBRATION_STEP, 1.0f);
            motors.setLeftCalibration(newCalibration);
            
            // Only output debug message if calibration debug is enabled
            extern int DEBUG_CALIBRATION_VALUE;
            if (DEBUG_CALIBRATION_VALUE) {
                Serial.printf("LEFT MOTOR CALIBRATION: %.2f -> %.2f\n", currentCalibration, newCalibration);
            }
            lastCalibrationButtonTime = currentTime;
        }
        
        // D-pad Down: Decrease left motor calibration
        if (state.down && !lastDownState) {
            float currentCalibration = motors.getLeftCalibration();
            float newCalibration = max(currentCalibration - CALIBRATION_STEP, 0.0f);
            motors.setLeftCalibration(newCalibration);
            
            // Only output debug message if calibration debug is enabled
            extern int DEBUG_CALIBRATION_VALUE;
            if (DEBUG_CALIBRATION_VALUE) {
                Serial.printf("LEFT MOTOR CALIBRATION: %.2f -> %.2f\n", currentCalibration, newCalibration);
            }
            lastCalibrationButtonTime = currentTime;
        }
        
        // Triangle: Increase right motor calibration
        if (state.triangle && !lastTriangleState) {
            float currentCalibration = motors.getRightCalibration();
            float newCalibration = min(currentCalibration + CALIBRATION_STEP, 1.0f);
            motors.setRightCalibration(newCalibration);
            
            // Only output debug message if calibration debug is enabled
            extern int DEBUG_CALIBRATION_VALUE;
            if (DEBUG_CALIBRATION_VALUE) {
                Serial.printf("RIGHT MOTOR CALIBRATION: %.2f -> %.2f\n", currentCalibration, newCalibration);
            }
            lastCalibrationButtonTime = currentTime;
        }
        
        // X: Decrease right motor calibration
        if (state.cross && !lastXState) {
            float currentCalibration = motors.getRightCalibration();
            float newCalibration = max(currentCalibration - CALIBRATION_STEP, 0.0f);
            motors.setRightCalibration(newCalibration);
            
            // Only output debug message if calibration debug is enabled
            extern int DEBUG_CALIBRATION_VALUE;
            if (DEBUG_CALIBRATION_VALUE) {
                Serial.printf("RIGHT MOTOR CALIBRATION: %.2f -> %.2f\n", currentCalibration, newCalibration);
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

// Check if it's time to update the motor control
bool TankController::isTimeToUpdate() {
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
        lastUpdateTime = currentTime;
        return true;
    }
    return false;
}

// Check if it's time to check button states
bool TankController::isTimeToCheckButtons() {
    unsigned long currentTime = millis();
    if (currentTime - lastButtonCheckTime >= BUTTON_CHECK_INTERVAL) {
        lastButtonCheckTime = currentTime;
        return true;
    }
    return false;
}

// Set a custom update interval
void TankController::setUpdateInterval(unsigned long intervalMs) {
    // Since UPDATE_INTERVAL is a const, we can't modify it directly
    // Instead, we could implement a variable update interval approach
    // For now, we'll just print a message indicating this isn't supported
    Serial.print("Note: Update interval is fixed at ");
    Serial.print(UPDATE_INTERVAL);
    Serial.println("ms");
}

// Get the current update interval
unsigned long TankController::getUpdateInterval() const {
    return UPDATE_INTERVAL;
}

// Get the current left motor calibration value
float TankController::getLeftMotorCalibration() const {
    return motors.getLeftCalibration();
}

// Get the current right motor calibration value
float TankController::getRightMotorCalibration() const {
    return motors.getRightCalibration();
}

// Function to control motors based on joystick input - Tank drive style
void TankController::controlMotorsWithJoystick(PS4Remote::ControllerState state) {
    // Get joystick values (invert Y so positive is forward)
    int leftY = -state.ly;  // Left joystick Y-axis controls left motor
    int rightY = -state.ry; // Right joystick Y-axis controls right motor
    
    // Initialize motor speeds
    int leftMotorSpeed = 0;
    int rightMotorSpeed = 0;

    // Process left motor forward/backward movement if beyond deadzone
    if (abs(leftY) > deadzone) {
        leftMotorSpeed = map(abs(leftY), deadzone, 127, 0, maxMotorSpeed);
        // Apply direction (positive = forward, negative = backward)
        leftMotorSpeed = (leftY > 0) ? leftMotorSpeed : -leftMotorSpeed;
    }

    // Process right motor forward/backward movement if beyond deadzone
    if (abs(rightY) > deadzone) {
        rightMotorSpeed = map(abs(rightY), deadzone, 127, 0, maxMotorSpeed);
        // Apply direction (positive = forward, negative = backward)
        rightMotorSpeed = (rightY > 0) ? rightMotorSpeed : -rightMotorSpeed;
    }

    // Constrain speeds and apply to motors
    leftMotorSpeed = constrain(leftMotorSpeed, -maxMotorSpeed, maxMotorSpeed);
    rightMotorSpeed = constrain(rightMotorSpeed, -maxMotorSpeed, maxMotorSpeed);
    
    // Set left motor direction and speed
    if (leftMotorSpeed > 0) motors.leftForward(leftMotorSpeed);
    else if (leftMotorSpeed < 0) motors.leftBackward(abs(leftMotorSpeed));
    else motors.leftStop();
    
    // Set right motor direction and speed
    if (rightMotorSpeed > 0) motors.rightForward(rightMotorSpeed);
    else if (rightMotorSpeed < 0) motors.rightBackward(abs(rightMotorSpeed));
    else motors.rightStop();
}

// Static callback for controller connection
void TankController::onConnect() {
    Serial.println("\n*** PS4 Controller Connected ***");
    
    // Reset the disconnect flag when controller reconnects
    extern bool wasDisconnected;
    wasDisconnected = false;
}

// Global variable to track disconnect state
bool wasDisconnected = false;

// Static callback for controller disconnection
void TankController::onDisconnect() {
    // Only show disconnect message once per connection session
    if (!wasDisconnected) {
        Serial.println("\n*** PS4 Controller Disconnected ***");
        
        // If debug is enabled, show emergency stop message
        extern int DEBUG_MOTOR_ACTIONS_VALUE;
        if (DEBUG_MOTOR_ACTIONS_VALUE) {
            Serial.println("MOTOR: EMERGENCY STOP (CONTROLLER DISCONNECTED)");
        }
        
        wasDisconnected = true;
    }
    
    // Stop motors when controller disconnects for safety
    if (instance) {
        instance->motors.stop();
    }
}
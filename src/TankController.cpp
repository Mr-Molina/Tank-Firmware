#include "TankController.h"

// Initialize static instance pointer
TankController* TankController::instance = nullptr;

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
    lastShareState(false)
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
}

// Update function called in main loop
void TankController::update() {
    // Get current controller state
    PS4Remote::ControllerState state = ps4.getState();
    
    // Control motors based on joystick input if controller is connected
    if (PS4.isConnected()) {
        controlMotorsWithJoystick(state);
        
        // Handle special button controls
        if (state.options) {
            // Options button: Emergency stop
            motors.stop();
            Serial.println("EMERGENCY STOP");
            delay(500); // Debounce
        }
        
        // Toggle smooth acceleration with Share button
        if (state.share && !lastShareState) {
            bool smooth = !motors.isSmoothEnabled();
            motors.setSmoothEnabled(smooth);
            Serial.print("Smooth acceleration: ");
            Serial.println(smooth ? "ENABLED" : "DISABLED");
            delay(200); // Debounce
        }
        lastShareState = state.share;
    } else {
        // No controller connected, ensure motors are stopped
        motors.stop();
    }
}

// Function to control motors based on joystick input
void TankController::controlMotorsWithJoystick(PS4Remote::ControllerState state) {
    // Get joystick values (invert Y so positive is forward)
    int leftY = -state.ly;
    int rightX = state.rx;
    
    // Initialize motor speeds
    int leftMotorSpeed = 0;
    int rightMotorSpeed = 0;

    // Process forward/backward movement if beyond deadzone
    if (abs(leftY) > deadzone) {
        int baseSpeed = map(abs(leftY), deadzone, 127, 0, maxMotorSpeed);
        // Apply direction (positive = forward, negative = backward)
        leftMotorSpeed = rightMotorSpeed = (leftY > 0) ? baseSpeed : -baseSpeed;
    }

    // Apply turning if beyond deadzone
    if (abs(rightX) > deadzone) {
        int turnAdjustment = map(abs(rightX), deadzone, 127, 0, maxMotorSpeed * turnSpeedFactor);
        // Apply turn adjustment (positive = right turn, negative = left turn)
        if (rightX > 0) {
            leftMotorSpeed += turnAdjustment;
            rightMotorSpeed -= turnAdjustment;
        } else {
            leftMotorSpeed -= turnAdjustment;
            rightMotorSpeed += turnAdjustment;
        }
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
}

// Static callback for controller disconnection
void TankController::onDisconnect() {
    Serial.println("\n*** PS4 Controller Disconnected ***");
    
    // Stop motors when controller disconnects for safety
    if (instance) {
        instance->motors.stop();
    }
}
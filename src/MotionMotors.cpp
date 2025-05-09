#include "MotionMotors.h"

// External reference to debug flag defined in main.cpp
extern int DEBUG_MOTOR_ACTIONS_VALUE;

/**
 * Constructor
 */
MotionMotors::MotionMotors(uint8_t leftA, uint8_t leftB, uint8_t rightA, uint8_t rightB,
                           float leftCalibration, float rightCalibration) {
  // Initialize motor pins
  M_LEFT_A_PIN = leftA;
  M_LEFT_B_PIN = leftB;
  M_RIGHT_A_PIN = rightA;
  M_RIGHT_B_PIN = rightB;
  
  // Initialize calibration factors
  LEFT_CALIBRATION = leftCalibration;
  RIGHT_CALIBRATION = rightCalibration;
  
  // Initialize state variables
  leftCurrentFunction = nullptr;
  rightCurrentFunction = nullptr;
  leftCurrentPower = 0;
  rightCurrentPower = 0;
  smoothEnabled = DEFAULT_SMOOTH_ENABLED;
  
  // Initialize debug tracking variables
  lastReportedLeftFunction = nullptr;
  lastReportedRightFunction = nullptr;
  lastReportedLeftPower = 0;
  lastReportedRightPower = 0;
  
  // Initialize timing variables for non-blocking acceleration
  lastAccelUpdateTime = 0;
  targetLeftPower = 0;
  targetRightPower = 0;
  currentAccelStep = 0;
  totalAccelSteps = 0;
  isAccelerating = false;
}

/**
 * Initialize the motors
 */
void MotionMotors::begin() {
  // Set up motor pins as outputs
  pinMode(M_LEFT_A_PIN, OUTPUT);
  pinMode(M_LEFT_B_PIN, OUTPUT);
  pinMode(M_RIGHT_A_PIN, OUTPUT);
  pinMode(M_RIGHT_B_PIN, OUTPUT);
  
  // Ensure motors are stopped
  stop();
}

/**
 * Independent motor control functions
 */

/**
 * Drives the left motor forward with optional smooth acceleration
 */
void MotionMotors::leftForward(uint8_t power, bool smooth) {
  // Direct control without smooth acceleration
  left_forward(power);
  leftCurrentFunction = &MotionMotors::left_forward;
  leftCurrentPower = power;
}

/**
 * Drives the left motor backward with optional smooth acceleration
 */
void MotionMotors::leftBackward(uint8_t power, bool smooth) {
  // Direct control without smooth acceleration
  left_backward(power);
  leftCurrentFunction = &MotionMotors::left_backward;
  leftCurrentPower = power;
}

/**
 * Drives the right motor forward with optional smooth acceleration
 */
void MotionMotors::rightForward(uint8_t power, bool smooth) {
  // Direct control without smooth acceleration
  right_forward(power);
  rightCurrentFunction = &MotionMotors::right_forward;
  rightCurrentPower = power;
}

/**
 * Drives the right motor backward with optional smooth acceleration
 */
void MotionMotors::rightBackward(uint8_t power, bool smooth) {
  // Direct control without smooth acceleration
  right_backward(power);
  rightCurrentFunction = &MotionMotors::right_backward;
  rightCurrentPower = power;
}

/**
 * Stops the left motor
 */
void MotionMotors::leftStop() {
  left_stop();
  leftCurrentFunction = nullptr;
  leftCurrentPower = 0;
}

/**
 * Stops the right motor
 */
void MotionMotors::rightStop() {
  right_stop();
  rightCurrentFunction = nullptr;
  rightCurrentPower = 0;
}

/**
 * Stops all motors immediately
 */
void MotionMotors::stop() {
  leftStop();
  rightStop();
  
  // Reset acceleration state
  isAccelerating = false;
}

/**
 * Enable or disable smooth acceleration/deceleration
 */
void MotionMotors::setSmoothEnabled(bool enable) {
  smoothEnabled = enable;
}

/**
 * Check if smooth acceleration/deceleration is enabled
 */
bool MotionMotors::isSmoothEnabled() {
  return smoothEnabled;
}

/**
 * Set the left motor calibration factor
 */
void MotionMotors::setLeftCalibration(float calibration) {
  // Constrain calibration to valid range (0.0 to 1.0)
  LEFT_CALIBRATION = constrain(calibration, 0.0, 1.0);
}

/**
 * Set the right motor calibration factor
 */
void MotionMotors::setRightCalibration(float calibration) {
  // Constrain calibration to valid range (0.0 to 1.0)
  RIGHT_CALIBRATION = constrain(calibration, 0.0, 1.0);
}

/**
 * Get the left motor calibration factor
 */
float MotionMotors::getLeftCalibration() const {
  return LEFT_CALIBRATION;
}

/**
 * Get the right motor calibration factor
 */
float MotionMotors::getRightCalibration() const {
  return RIGHT_CALIBRATION;
}

/**
 * Low-level motor control functions with calibration
 */

/**
 * Drives the left motor forward
 */
void MotionMotors::left_forward(uint8_t power) {
  uint8_t calibratedPower = power * LEFT_CALIBRATION;
  analogWrite(M_LEFT_A_PIN, calibratedPower);
  analogWrite(M_LEFT_B_PIN, 0);
  
  // Debug output if enabled and state has changed
  if (DEBUG_MOTOR_ACTIONS_VALUE) {
    // Only report if direction changed or power changed by more than 5%
    bool directionChanged = (lastReportedLeftFunction != &MotionMotors::left_forward);
    bool powerChanged = (abs((int)power - (int)lastReportedLeftPower) > 5);
    
    if (directionChanged || powerChanged) {
      Serial.printf("MOTOR: LEFT FORWARD (Power: %d) [Left Joystick]\n", power);
      
      // Update last reported state
      lastReportedLeftFunction = &MotionMotors::left_forward;
      lastReportedLeftPower = power;
    }
  }
}

/**
 * Drives the left motor backward
 */
void MotionMotors::left_backward(uint8_t power) {
  uint8_t calibratedPower = power * LEFT_CALIBRATION;
  analogWrite(M_LEFT_A_PIN, 0);
  analogWrite(M_LEFT_B_PIN, calibratedPower);
  
  // Debug output if enabled and state has changed
  if (DEBUG_MOTOR_ACTIONS_VALUE) {
    // Only report if direction changed or power changed by more than 5%
    bool directionChanged = (lastReportedLeftFunction != &MotionMotors::left_backward);
    bool powerChanged = (abs((int)power - (int)lastReportedLeftPower) > 5);
    
    if (directionChanged || powerChanged) {
      Serial.printf("MOTOR: LEFT BACKWARD (Power: %d) [Left Joystick]\n", power);
      
      // Update last reported state
      lastReportedLeftFunction = &MotionMotors::left_backward;
      lastReportedLeftPower = power;
    }
  }
}

/**
 * Stops the left motor
 */
void MotionMotors::left_stop() {
  analogWrite(M_LEFT_A_PIN, 0);
  analogWrite(M_LEFT_B_PIN, 0);
  
  // Only report stop if we were previously moving
  if (DEBUG_MOTOR_ACTIONS_VALUE && lastReportedLeftFunction != nullptr) {
    Serial.println("MOTOR: LEFT STOP [Left Joystick]");
    
    // Update last reported state
    lastReportedLeftFunction = nullptr;
    lastReportedLeftPower = 0;
  }
}

/**
 * Drives the right motor forward
 */
void MotionMotors::right_forward(uint8_t power) {
  uint8_t calibratedPower = power * RIGHT_CALIBRATION;
  analogWrite(M_RIGHT_A_PIN, calibratedPower);
  analogWrite(M_RIGHT_B_PIN, 0);
  
  // Debug output if enabled and state has changed
  if (DEBUG_MOTOR_ACTIONS_VALUE) {
    // Only report if direction changed or power changed by more than 5%
    bool directionChanged = (lastReportedRightFunction != &MotionMotors::right_forward);
    bool powerChanged = (abs((int)power - (int)lastReportedRightPower) > 5);
    
    if (directionChanged || powerChanged) {
      Serial.printf("MOTOR: RIGHT FORWARD (Power: %d) [Right Joystick]\n", power);
      
      // Update last reported state
      lastReportedRightFunction = &MotionMotors::right_forward;
      lastReportedRightPower = power;
    }
  }
}

/**
 * Drives the right motor backward
 */
void MotionMotors::right_backward(uint8_t power) {
  uint8_t calibratedPower = power * RIGHT_CALIBRATION;
  analogWrite(M_RIGHT_A_PIN, 0);
  analogWrite(M_RIGHT_B_PIN, calibratedPower);
  
  // Debug output if enabled and state has changed
  if (DEBUG_MOTOR_ACTIONS_VALUE) {
    // Only report if direction changed or power changed by more than 5%
    bool directionChanged = (lastReportedRightFunction != &MotionMotors::right_backward);
    bool powerChanged = (abs((int)power - (int)lastReportedRightPower) > 5);
    
    if (directionChanged || powerChanged) {
      Serial.printf("MOTOR: RIGHT BACKWARD (Power: %d) [Right Joystick]\n", power);
      
      // Update last reported state
      lastReportedRightFunction = &MotionMotors::right_backward;
      lastReportedRightPower = power;
    }
  }
}

/**
 * Stops the right motor
 */
void MotionMotors::right_stop() {
  analogWrite(M_RIGHT_A_PIN, 0);
  analogWrite(M_RIGHT_B_PIN, 0);
  
  // Only report stop if we were previously moving
  if (DEBUG_MOTOR_ACTIONS_VALUE && lastReportedRightFunction != nullptr) {
    Serial.println("MOTOR: RIGHT STOP [Right Joystick]");
    
    // Update last reported state
    lastReportedRightFunction = nullptr;
    lastReportedRightPower = 0;
  }
}

/**
 * Non-blocking acceleration update function
 * This should be called regularly from the main loop
 */
void MotionMotors::updateAcceleration() {
  // If not currently accelerating, do nothing
  if (!isAccelerating || !smoothEnabled) {
    return;
  }
  
  // Check if it's time for the next acceleration step
  unsigned long currentTime = millis();
  if (currentTime - lastAccelUpdateTime < SMOOTH_ACCEL_DELAY) {
    return;
  }
  
  // Update the timestamp
  lastAccelUpdateTime = currentTime;
  
  // Increment the step counter
  currentAccelStep++;
  
  // Check if we've completed all steps
  if (currentAccelStep >= totalAccelSteps) {
    // We've reached the target power, finalize
    if (leftTargetFunction != nullptr) {
      (this->*leftTargetFunction)(targetLeftPower);
      leftCurrentFunction = leftTargetFunction;
      leftCurrentPower = targetLeftPower;
    }
    
    if (rightTargetFunction != nullptr) {
      (this->*rightTargetFunction)(targetRightPower);
      rightCurrentFunction = rightTargetFunction;
      rightCurrentPower = targetRightPower;
    }
    
    // Reset acceleration state
    isAccelerating = false;
    return;
  }
  
  // Calculate intermediate power levels
  if (leftTargetFunction != nullptr) {
    float progress = (float)currentAccelStep / totalAccelSteps;
    uint8_t power = targetLeftPower * progress;
    (this->*leftTargetFunction)(power);
  }
  
  if (rightTargetFunction != nullptr) {
    float progress = (float)currentAccelStep / totalAccelSteps;
    uint8_t power = targetRightPower * progress;
    (this->*rightTargetFunction)(power);
  }
}

/**
 * Start a non-blocking acceleration sequence
 */
void MotionMotors::startAcceleration(
    void (MotionMotors::*leftFunc)(uint8_t), uint8_t leftPower,
    void (MotionMotors::*rightFunc)(uint8_t), uint8_t rightPower,
    uint8_t steps) {
  
  // Store target functions and powers
  leftTargetFunction = leftFunc;
  rightTargetFunction = rightFunc;
  targetLeftPower = leftPower;
  targetRightPower = rightPower;
  
  // Initialize acceleration state
  currentAccelStep = 0;
  totalAccelSteps = steps;
  lastAccelUpdateTime = millis();
  isAccelerating = true;
}

/**
 * Smoothly accelerate to target power (non-blocking version)
 */
void MotionMotors::smoothAccelerate(void (MotionMotors::*moveFunction)(uint8_t), uint8_t targetPower,
                                   uint8_t steps, uint8_t delayMs) {
  // If smooth acceleration is disabled, just set the power directly
  if (!smoothEnabled || steps <= 1) {
    (this->*moveFunction)(targetPower);
    
    // Update the appropriate current state based on which function was called
    if (moveFunction == &MotionMotors::left_forward || moveFunction == &MotionMotors::left_backward) {
      leftCurrentFunction = moveFunction;
      leftCurrentPower = targetPower;
    } else if (moveFunction == &MotionMotors::right_forward || moveFunction == &MotionMotors::right_backward) {
      rightCurrentFunction = moveFunction;
      rightCurrentPower = targetPower;
    }
    return;
  }
  
  // For left motor functions
  if (moveFunction == &MotionMotors::left_forward || moveFunction == &MotionMotors::left_backward) {
    startAcceleration(moveFunction, targetPower, nullptr, 0, steps);
  }
  // For right motor functions
  else if (moveFunction == &MotionMotors::right_forward || moveFunction == &MotionMotors::right_backward) {
    startAcceleration(nullptr, 0, moveFunction, targetPower, steps);
  }
}

/**
 * Smoothly decelerate to a stop (non-blocking version)
 */
void MotionMotors::smoothDecelerate(void (MotionMotors::*moveFunction)(uint8_t), uint8_t currentPower,
                                   uint8_t steps, uint8_t delayMs) {
  // If smooth deceleration is disabled, just stop the appropriate motor
  if (!smoothEnabled || steps <= 1) {
    if (moveFunction == &MotionMotors::left_forward || moveFunction == &MotionMotors::left_backward) {
      leftStop();
    } else if (moveFunction == &MotionMotors::right_forward || moveFunction == &MotionMotors::right_backward) {
      rightStop();
    }
    return;
  }
  
  // For now, just stop immediately to avoid crashes
  if (moveFunction == &MotionMotors::left_forward || moveFunction == &MotionMotors::left_backward) {
    leftStop();
  } else if (moveFunction == &MotionMotors::right_forward || moveFunction == &MotionMotors::right_backward) {
    rightStop();
  }
}

/**
 * Smoothly transition between two movement functions (non-blocking version)
 */
void MotionMotors::smoothTransition(void (MotionMotors::*currentFunction)(uint8_t), uint8_t currentPower,
                                   void (MotionMotors::*targetFunction)(uint8_t), uint8_t targetPower,
                                   uint8_t steps, uint8_t delayMs) {
  // If smooth transition is disabled or functions are the same, just set the power directly
  if (!smoothEnabled || steps <= 1 || currentFunction == targetFunction) {
    (this->*targetFunction)(targetPower);
    
    // Update the appropriate current state based on which function was called
    if (targetFunction == &MotionMotors::left_forward || targetFunction == &MotionMotors::left_backward) {
      leftCurrentFunction = targetFunction;
      leftCurrentPower = targetPower;
    } else if (targetFunction == &MotionMotors::right_forward || targetFunction == &MotionMotors::right_backward) {
      rightCurrentFunction = targetFunction;
      rightCurrentPower = targetPower;
    }
    return;
  }
  
  // For now, just go directly to the target to avoid crashes
  (this->*targetFunction)(targetPower);
  
  // Update the appropriate current state
  if (targetFunction == &MotionMotors::left_forward || targetFunction == &MotionMotors::left_backward) {
    leftCurrentFunction = targetFunction;
    leftCurrentPower = targetPower;
  } else if (targetFunction == &MotionMotors::right_forward || targetFunction == &MotionMotors::right_backward) {
    rightCurrentFunction = targetFunction;
    rightCurrentPower = targetPower;
  }
}
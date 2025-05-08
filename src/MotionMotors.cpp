#include "MotionMotors.h"

/**
 * Constructor
 */
MotionMotors::MotionMotors() {
  currentMoveFunction = nullptr;
  currentPower = 0;
  smoothEnabled = DEFAULT_SMOOTH_ENABLED;
}

/**
 * Initialize the motors
 */
void MotionMotors::begin() {
  // Set up motor pins as outputs
  pinMode(M_RL_A_PIN, OUTPUT);
  pinMode(M_RL_B_PIN, OUTPUT);
  pinMode(M_RR_A_PIN, OUTPUT);
  pinMode(M_RR_B_PIN, OUTPUT);
  
  // Ensure motors are stopped
  stop();
}

/**
 * High-level movement functions
 */

/**
 * Moves the vehicle straight forward
 */
void MotionMotors::forward(uint8_t power, bool smooth) {
  // Direct function pointer for use with smooth acceleration
  auto forwardFunc = &MotionMotors::RL_forward;

  if (smooth && smoothEnabled && currentMoveFunction != forwardFunc) {
    // If we're already moving, transition smoothly
    if (currentMoveFunction != nullptr && currentPower > 0) {
      smoothTransition(currentMoveFunction, currentPower, forwardFunc, power);
    } else {
      // Otherwise just accelerate
      smoothAccelerate(forwardFunc, power);
    }
  } else {
    // Direct control without smooth acceleration
    RL_forward(power);
    RR_forward(power);
    currentMoveFunction = forwardFunc;
    currentPower = power;
  }
}

/**
 * Moves the vehicle straight backward
 */
void MotionMotors::backward(uint8_t power, bool smooth) {
  // Direct function pointer for use with smooth acceleration
  auto backwardFunc = &MotionMotors::RL_backward;

  if (smooth && smoothEnabled && currentMoveFunction != backwardFunc) {
    // If we're already moving, transition smoothly
    if (currentMoveFunction != nullptr && currentPower > 0) {
      smoothTransition(currentMoveFunction, currentPower, backwardFunc, power);
    } else {
      // Otherwise just accelerate
      smoothAccelerate(backwardFunc, power);
    }
  } else {
    // Direct control without smooth acceleration
    RL_backward(power);
    RR_backward(power);
    currentMoveFunction = backwardFunc;
    currentPower = power;
  }
}

/**
 * Rotates the vehicle to the left (counter-clockwise) in place
 */
void MotionMotors::left(uint8_t power) {
  RL_backward(power);
  RR_forward(power);
  
  // Update current state
  currentMoveFunction = nullptr; // Not using smooth transitions for rotation
  currentPower = power;
}

/**
 * Rotates the vehicle to the right (clockwise) in place
 */
void MotionMotors::right(uint8_t power) {
  RL_forward(power);
  RR_backward(power);
  
  // Update current state
  currentMoveFunction = nullptr; // Not using smooth transitions for rotation
  currentPower = power;
}

/**
 * Stops all motors immediately
 */
void MotionMotors::stop() {
  RL_Stop();
  RR_Stop();

  // Update current state
  currentMoveFunction = nullptr;
  currentPower = 0;
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
 * Low-level motor control functions with calibration
 */

/**
 * Drives the Rear Left motor forward
 */
void MotionMotors::RL_forward(uint8_t power) {
  uint8_t calibratedPower = power * RL_CALIBRATION;
  analogWrite(M_RL_A_PIN, calibratedPower);
  analogWrite(M_RL_B_PIN, 0);
}

/**
 * Drives the Rear Left motor backward
 */
void MotionMotors::RL_backward(uint8_t power) {
  uint8_t calibratedPower = power * RL_CALIBRATION;
  analogWrite(M_RL_A_PIN, 0);
  analogWrite(M_RL_B_PIN, calibratedPower);
}

/**
 * Stops the Rear Left motor
 */
void MotionMotors::RL_Stop() {
  analogWrite(M_RL_A_PIN, 0);
  analogWrite(M_RL_B_PIN, 0);
}

/**
 * Drives the Rear Right motor forward
 */
void MotionMotors::RR_forward(uint8_t power) {
  uint8_t calibratedPower = power * RR_CALIBRATION;
  analogWrite(M_RR_A_PIN, calibratedPower);
  analogWrite(M_RR_B_PIN, 0);
}

/**
 * Drives the Rear Right motor backward
 */
void MotionMotors::RR_backward(uint8_t power) {
  uint8_t calibratedPower = power * RR_CALIBRATION;
  analogWrite(M_RR_A_PIN, 0);
  analogWrite(M_RR_B_PIN, calibratedPower);
}

/**
 * Stops the Rear Right motor
 */
void MotionMotors::RR_Stop() {
  analogWrite(M_RR_A_PIN, 0);
  analogWrite(M_RR_B_PIN, 0);
}

/**
 * Smooth acceleration/deceleration implementation
 */

/**
 * Gradually accelerate to target power
 */
void MotionMotors::smoothAccelerate(void (MotionMotors::*moveFunction)(uint8_t), uint8_t targetPower,
                                    uint8_t steps, uint8_t delayMs) {
  // If smooth acceleration is disabled, just set the power directly
  if (!smoothEnabled || steps <= 1) {
    (this->*moveFunction)(targetPower);
    RR_forward(targetPower); // Also apply to the other motor
    currentMoveFunction = moveFunction;
    currentPower = targetPower;
    return;
  }

  // Calculate step size
  float stepSize = (float)targetPower / steps;

  // Gradually increase power
  for (uint8_t i = 1; i <= steps; i++) {
    uint8_t power = constrain((uint8_t)(stepSize * i), 0, 255);
    (this->*moveFunction)(power);
    
    // Apply the same power to the other motor based on which function is being called
    if (moveFunction == &MotionMotors::RL_forward) {
      RR_forward(power);
    } else if (moveFunction == &MotionMotors::RL_backward) {
      RR_backward(power);
    }
    
    delay(delayMs);
  }

  // Ensure we reach exactly the target power
  (this->*moveFunction)(targetPower);
  
  // Apply the same power to the other motor based on which function is being called
  if (moveFunction == &MotionMotors::RL_forward) {
    RR_forward(targetPower);
  } else if (moveFunction == &MotionMotors::RL_backward) {
    RR_backward(targetPower);
  }

  // Update current state
  currentMoveFunction = moveFunction;
  currentPower = targetPower;
}

/**
 * Gradually decelerate to a stop
 */
void MotionMotors::smoothDecelerate(void (MotionMotors::*moveFunction)(uint8_t), uint8_t currentPower,
                                    uint8_t steps, uint8_t delayMs) {
  // If smooth deceleration is disabled, just stop
  if (!smoothEnabled || steps <= 1) {
    stop();
    return;
  }

  // Calculate step size
  float stepSize = (float)currentPower / steps;

  // Gradually decrease power
  for (int i = steps - 1; i >= 0; i--) {
    uint8_t power = constrain((uint8_t)(stepSize * i), 0, 255);
    (this->*moveFunction)(power);
    
    // Apply the same power to the other motor based on which function is being called
    if (moveFunction == &MotionMotors::RL_forward) {
      RR_forward(power);
    } else if (moveFunction == &MotionMotors::RL_backward) {
      RR_backward(power);
    }
    
    delay(delayMs);
  }

  // Ensure we stop completely
  stop();
}

/**
 * Smoothly transition between two movement functions
 */
void MotionMotors::smoothTransition(void (MotionMotors::*currentFunction)(uint8_t), uint8_t currentPower,
                                    void (MotionMotors::*targetFunction)(uint8_t), uint8_t targetPower,
                                    uint8_t steps, uint8_t delayMs) {
  // If smooth transition is disabled or functions are the same, just set the power directly
  if (!smoothEnabled || steps <= 1 || currentFunction == targetFunction) {
    (this->*targetFunction)(targetPower);
    
    // Apply the same power to the other motor based on which function is being called
    if (targetFunction == &MotionMotors::RL_forward) {
      RR_forward(targetPower);
    } else if (targetFunction == &MotionMotors::RL_backward) {
      RR_backward(targetPower);
    }
    
    currentMoveFunction = targetFunction;
    currentPower = targetPower;
    return;
  }

  // First decelerate the current movement
  smoothDecelerate(currentFunction, currentPower, steps / 2, delayMs);

  // Then accelerate to the new movement
  smoothAccelerate(targetFunction, targetPower, steps / 2, delayMs);
}
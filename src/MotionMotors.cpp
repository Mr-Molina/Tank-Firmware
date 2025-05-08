#include "MotionMotors.h"

/**
 * Constructor
 */
MotionMotors::MotionMotors() {
  leftCurrentFunction = nullptr;
  rightCurrentFunction = nullptr;
  leftCurrentPower = 0;
  rightCurrentPower = 0;
  smoothEnabled = DEFAULT_SMOOTH_ENABLED;
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
  auto forwardFunc = &MotionMotors::left_forward;

  if (smooth && smoothEnabled && leftCurrentFunction != forwardFunc) {
    // If we're already moving, transition smoothly
    if (leftCurrentFunction != nullptr && leftCurrentPower > 0) {
      smoothTransition(leftCurrentFunction, leftCurrentPower, forwardFunc, power);
    } else {
      // Otherwise just accelerate
      smoothAccelerate(forwardFunc, power);
    }
  } else {
    // Direct control without smooth acceleration
    left_forward(power);
    leftCurrentFunction = forwardFunc;
    leftCurrentPower = power;
  }
}

/**
 * Drives the left motor backward with optional smooth acceleration
 */
void MotionMotors::leftBackward(uint8_t power, bool smooth) {
  auto backwardFunc = &MotionMotors::left_backward;

  if (smooth && smoothEnabled && leftCurrentFunction != backwardFunc) {
    // If we're already moving, transition smoothly
    if (leftCurrentFunction != nullptr && leftCurrentPower > 0) {
      smoothTransition(leftCurrentFunction, leftCurrentPower, backwardFunc, power);
    } else {
      // Otherwise just accelerate
      smoothAccelerate(backwardFunc, power);
    }
  } else {
    // Direct control without smooth acceleration
    left_backward(power);
    leftCurrentFunction = backwardFunc;
    leftCurrentPower = power;
  }
}

/**
 * Drives the right motor forward with optional smooth acceleration
 */
void MotionMotors::rightForward(uint8_t power, bool smooth) {
  auto forwardFunc = &MotionMotors::right_forward;

  if (smooth && smoothEnabled && rightCurrentFunction != forwardFunc) {
    // If we're already moving, transition smoothly
    if (rightCurrentFunction != nullptr && rightCurrentPower > 0) {
      smoothTransition(rightCurrentFunction, rightCurrentPower, forwardFunc, power);
    } else {
      // Otherwise just accelerate
      smoothAccelerate(forwardFunc, power);
    }
  } else {
    // Direct control without smooth acceleration
    right_forward(power);
    rightCurrentFunction = forwardFunc;
    rightCurrentPower = power;
  }
}

/**
 * Drives the right motor backward with optional smooth acceleration
 */
void MotionMotors::rightBackward(uint8_t power, bool smooth) {
  auto backwardFunc = &MotionMotors::right_backward;

  if (smooth && smoothEnabled && rightCurrentFunction != backwardFunc) {
    // If we're already moving, transition smoothly
    if (rightCurrentFunction != nullptr && rightCurrentPower > 0) {
      smoothTransition(rightCurrentFunction, rightCurrentPower, backwardFunc, power);
    } else {
      // Otherwise just accelerate
      smoothAccelerate(backwardFunc, power);
    }
  } else {
    // Direct control without smooth acceleration
    right_backward(power);
    rightCurrentFunction = backwardFunc;
    rightCurrentPower = power;
  }
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
  left_stop();
  right_stop();

  // Update current state
  leftCurrentFunction = nullptr;
  rightCurrentFunction = nullptr;
  leftCurrentPower = 0;
  rightCurrentPower = 0;
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
 * Drives the left motor forward
 */
void MotionMotors::left_forward(uint8_t power) {
  uint8_t calibratedPower = power * LEFT_CALIBRATION;
  analogWrite(M_LEFT_A_PIN, calibratedPower);
  analogWrite(M_LEFT_B_PIN, 0);
}

/**
 * Drives the left motor backward
 */
void MotionMotors::left_backward(uint8_t power) {
  uint8_t calibratedPower = power * LEFT_CALIBRATION;
  analogWrite(M_LEFT_A_PIN, 0);
  analogWrite(M_LEFT_B_PIN, calibratedPower);
}

/**
 * Stops the left motor
 */
void MotionMotors::left_stop() {
  analogWrite(M_LEFT_A_PIN, 0);
  analogWrite(M_LEFT_B_PIN, 0);
}

/**
 * Drives the right motor forward
 */
void MotionMotors::right_forward(uint8_t power) {
  uint8_t calibratedPower = power * RIGHT_CALIBRATION;
  analogWrite(M_RIGHT_A_PIN, calibratedPower);
  analogWrite(M_RIGHT_B_PIN, 0);
}

/**
 * Drives the right motor backward
 */
void MotionMotors::right_backward(uint8_t power) {
  uint8_t calibratedPower = power * RIGHT_CALIBRATION;
  analogWrite(M_RIGHT_A_PIN, 0);
  analogWrite(M_RIGHT_B_PIN, calibratedPower);
}

/**
 * Stops the right motor
 */
void MotionMotors::right_stop() {
  analogWrite(M_RIGHT_A_PIN, 0);
  analogWrite(M_RIGHT_B_PIN, 0);
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

  // Calculate step size
  float stepSize = (float)targetPower / steps;

  // Gradually increase power
  for (uint8_t i = 1; i <= steps; i++) {
    uint8_t power = constrain((uint8_t)(stepSize * i), 0, 255);
    (this->*moveFunction)(power);
    delay(delayMs);
  }

  // Ensure we reach exactly the target power
  (this->*moveFunction)(targetPower);

  // Update the appropriate current state based on which function was called
  if (moveFunction == &MotionMotors::left_forward || moveFunction == &MotionMotors::left_backward) {
    leftCurrentFunction = moveFunction;
    leftCurrentPower = targetPower;
  } else if (moveFunction == &MotionMotors::right_forward || moveFunction == &MotionMotors::right_backward) {
    rightCurrentFunction = moveFunction;
    rightCurrentPower = targetPower;
  }
}

/**
 * Gradually decelerate to a stop
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

  // Calculate step size
  float stepSize = (float)currentPower / steps;

  // Gradually decrease power
  for (int i = steps - 1; i >= 0; i--) {
    uint8_t power = constrain((uint8_t)(stepSize * i), 0, 255);
    (this->*moveFunction)(power);
    delay(delayMs);
  }

  // Ensure we stop completely the appropriate motor
  if (moveFunction == &MotionMotors::left_forward || moveFunction == &MotionMotors::left_backward) {
    leftStop();
  } else if (moveFunction == &MotionMotors::right_forward || moveFunction == &MotionMotors::right_backward) {
    rightStop();
  }
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

  // First decelerate the current movement
  smoothDecelerate(currentFunction, currentPower, steps / 2, delayMs);

  // Then accelerate to the new movement
  smoothAccelerate(targetFunction, targetPower, steps / 2, delayMs);
}
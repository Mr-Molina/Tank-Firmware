#include "MotionMotors.h"

// This connects to the debug setting in main.cpp
extern int DEBUG_MOTOR_ACTIONS_VALUE;

/**
 * Constructor - This sets up a new MotionMotors object
 * 
 * Think of this as building the robot's "legs" before using them.
 */
MotionMotors::MotionMotors(uint8_t leftA, uint8_t leftB, uint8_t rightA, uint8_t rightB,
                           float leftCalibration, float rightCalibration) {
  // Remember which pins control which motors
  M_LEFT_A_PIN = leftA;    // Left motor forward pin
  M_LEFT_B_PIN = leftB;    // Left motor backward pin
  M_RIGHT_A_PIN = rightA;  // Right motor forward pin
  M_RIGHT_B_PIN = rightB;  // Right motor backward pin
  
  // Set up how powerful each motor should be
  LEFT_CALIBRATION = leftCalibration;   // Left motor power (0.0-1.0)
  RIGHT_CALIBRATION = rightCalibration; // Right motor power (0.0-1.0)
  
  // Start with motors not moving
  leftCurrentFunction = nullptr;   // nullptr means "not moving"
  rightCurrentFunction = nullptr;  // nullptr means "not moving"
  leftCurrentPower = 0;            // 0 power = stopped
  rightCurrentPower = 0;           // 0 power = stopped
  smoothEnabled = DEFAULT_SMOOTH_ENABLED;  // Turn on smooth acceleration
  
  // Set up variables to track what we've told the computer
  lastReportedLeftFunction = nullptr;   // No left motor movement reported yet
  lastReportedRightFunction = nullptr;  // No right motor movement reported yet
  lastReportedLeftPower = 0;            // No left motor power reported yet
  lastReportedRightPower = 0;           // No right motor power reported yet
  
  // Set up variables for smooth acceleration
  lastAccelUpdateTime = 0;    // When we last updated acceleration
  targetLeftPower = 0;        // Target power for left motor
  targetRightPower = 0;       // Target power for right motor
  currentAccelStep = 0;       // Which step we're on in acceleration
  totalAccelSteps = 0;        // Total steps to take
  isAccelerating = false;     // Not currently accelerating
}

/**
 * Begin - Gets the motors ready to use
 * 
 * This is like turning on the robot's legs so they're ready to move.
 */
void MotionMotors::begin() {
  // Set up all motor pins as outputs (they send signals, not receive them)
  pinMode(M_LEFT_A_PIN, OUTPUT);
  pinMode(M_LEFT_B_PIN, OUTPUT);
  pinMode(M_RIGHT_A_PIN, OUTPUT);
  pinMode(M_RIGHT_B_PIN, OUTPUT);
  
  // Make sure all motors are stopped when we start
  stop();
}

/**
 * Motor control functions - These make the robot move
 */

/**
 * Makes the left wheel go forward
 * 
 * This is like telling the left leg to step forward.
 */
void MotionMotors::leftForward(uint8_t power, bool smooth) {
  // Tell the left motor to go forward at the specified power
  left_forward(power);
  
  // Remember what the motor is doing
  leftCurrentFunction = &MotionMotors::left_forward;  // It's going forward
  leftCurrentPower = power;                           // At this power level
}

/**
 * Makes the left wheel go backward
 * 
 * This is like telling the left leg to step backward.
 */
void MotionMotors::leftBackward(uint8_t power, bool smooth) {
  // Tell the left motor to go backward at the specified power
  left_backward(power);
  
  // Remember what the motor is doing
  leftCurrentFunction = &MotionMotors::left_backward;  // It's going backward
  leftCurrentPower = power;                            // At this power level
}

/**
 * Makes the right wheel go forward
 * 
 * This is like telling the right leg to step forward.
 */
void MotionMotors::rightForward(uint8_t power, bool smooth) {
  // Tell the right motor to go forward at the specified power
  right_forward(power);
  
  // Remember what the motor is doing
  rightCurrentFunction = &MotionMotors::right_forward;  // It's going forward
  rightCurrentPower = power;                            // At this power level
}

/**
 * Makes the right wheel go backward
 * 
 * This is like telling the right leg to step backward.
 */
void MotionMotors::rightBackward(uint8_t power, bool smooth) {
  // Tell the right motor to go backward at the specified power
  right_backward(power);
  
  // Remember what the motor is doing
  rightCurrentFunction = &MotionMotors::right_backward;  // It's going backward
  rightCurrentPower = power;                             // At this power level
}

/**
 * Stops the left wheel
 * 
 * This is like telling the left leg to stop moving.
 */
void MotionMotors::leftStop() {
  // Tell the left motor to stop
  left_stop();
  
  // Remember that the motor is stopped
  leftCurrentFunction = nullptr;  // nullptr means "not moving"
  leftCurrentPower = 0;           // 0 power = stopped
}

/**
 * Stops the right wheel
 * 
 * This is like telling the right leg to stop moving.
 */
void MotionMotors::rightStop() {
  // Tell the right motor to stop
  right_stop();
  
  // Remember that the motor is stopped
  rightCurrentFunction = nullptr;  // nullptr means "not moving"
  rightCurrentPower = 0;           // 0 power = stopped
}

/**
 * Stops both wheels immediately
 * 
 * This is like telling both legs to freeze in place.
 */
void MotionMotors::stop() {
  // Stop both motors
  leftStop();
  rightStop();
  
  // Turn off any acceleration that might be happening
  isAccelerating = false;
}

/**
 * Turn smooth acceleration on or off
 * 
 * Smooth acceleration is like starting to walk slowly before running,
 * instead of suddenly sprinting from a standstill.
 */
void MotionMotors::setSmoothEnabled(bool enable) {
  smoothEnabled = enable;
}

/**
 * Check if smooth acceleration is turned on
 */
bool MotionMotors::isSmoothEnabled() {
  return smoothEnabled;
}

/**
 * Set how powerful the left motor should be
 * 
 * This is useful if one motor is stronger than the other
 * and you want to balance them.
 */
void MotionMotors::setLeftCalibration(float calibration) {
  // Make sure the calibration value is between 0.0 and 1.0
  // (0.0 = 0% power, 1.0 = 100% power)
  LEFT_CALIBRATION = constrain(calibration, 0.0, 1.0);
}

/**
 * Set how powerful the right motor should be
 */
void MotionMotors::setRightCalibration(float calibration) {
  // Make sure the calibration value is between 0.0 and 1.0
  RIGHT_CALIBRATION = constrain(calibration, 0.0, 1.0);
}

/**
 * Get the current left motor power setting
 */
float MotionMotors::getLeftCalibration() const {
  return LEFT_CALIBRATION;
}

/**
 * Get the current right motor power setting
 */
float MotionMotors::getRightCalibration() const {
  return RIGHT_CALIBRATION;
}

/**
 * These are the behind-the-scenes functions that actually control the motors
 */

/**
 * Makes the left motor spin forward
 * 
 * This sends electrical signals to the motor to make it turn in the forward direction.
 */
void MotionMotors::left_forward(uint8_t power) {
  // Apply the calibration factor to adjust the motor power
  // This helps balance the motors if one is stronger than the other
  uint8_t calibratedPower = power * LEFT_CALIBRATION;
  
  // Turn on the forward direction pin with our power level
  analogWrite(M_LEFT_A_PIN, calibratedPower);
  
  // Make sure the backward direction pin is off (set to 0)
  analogWrite(M_LEFT_B_PIN, 0);
  
  // Show helpful messages on the computer screen if debugging is turned on
  if (DEBUG_MOTOR_ACTIONS_VALUE) {
    // We only want to show a message when something important changes:
    // 1. If the motor changed direction (like from stopped to moving forward)
    bool directionChanged = (lastReportedLeftFunction != &MotionMotors::left_forward);
    
    // 2. Or if the power level changed by more than 5 units
    bool powerChanged = (abs((int)power - (int)lastReportedLeftPower) > 5);
    
    // If either important thing changed, show a message
    if (directionChanged || powerChanged) {
      Serial.printf("MOTOR: LEFT FORWARD (Power: %d) [Left Joystick]\n", power);
      
      // Remember what we just reported so we don't repeat the same message
      lastReportedLeftFunction = &MotionMotors::left_forward;
      lastReportedLeftPower = power;
    }
  }
}

/**
 * Makes the left motor spin backward
 * 
 * This sends electrical signals to the motor to make it turn in the reverse direction.
 */
void MotionMotors::left_backward(uint8_t power) {
  // Apply the calibration factor to adjust the motor power
  // This helps balance the motors if one is stronger than the other
  uint8_t calibratedPower = power * LEFT_CALIBRATION;
  
  // Make sure the forward direction pin is off (set to 0)
  analogWrite(M_LEFT_A_PIN, 0);
  
  // Turn on the backward direction pin with our power level
  analogWrite(M_LEFT_B_PIN, calibratedPower);
  
  // Show helpful messages on the computer screen if debugging is turned on
  if (DEBUG_MOTOR_ACTIONS_VALUE) {
    // We only want to show a message when something important changes:
    // 1. If the motor changed direction (like from stopped to moving backward)
    bool directionChanged = (lastReportedLeftFunction != &MotionMotors::left_backward);
    
    // 2. Or if the power level changed by more than 5 units
    bool powerChanged = (abs((int)power - (int)lastReportedLeftPower) > 5);
    
    // If either important thing changed, show a message
    if (directionChanged || powerChanged) {
      Serial.printf("MOTOR: LEFT BACKWARD (Power: %d) [Left Joystick]\n", power);
      
      // Remember what we just reported so we don't repeat the same message
      lastReportedLeftFunction = &MotionMotors::left_backward;
      lastReportedLeftPower = power;
    }
  }
}

/**
 * Stops the left motor from moving
 * 
 * This turns off all power to the left motor so it stops spinning.
 */
void MotionMotors::left_stop() {
  // Turn off both direction pins to stop the motor
  analogWrite(M_LEFT_A_PIN, 0);
  analogWrite(M_LEFT_B_PIN, 0);
  
  // Only show a message if the motor was actually moving before
  // (We don't want messages about stopping an already stopped motor)
  if (DEBUG_MOTOR_ACTIONS_VALUE && lastReportedLeftFunction != nullptr) {
    Serial.println("MOTOR: LEFT STOP [Left Joystick]");
    
    // Reset our memory of what the motor was doing
    lastReportedLeftFunction = nullptr;  // nullptr means "nothing" or "not moving"
    lastReportedLeftPower = 0;           // Power is now zero
  }
}

/**
 * Makes the right motor spin forward
 * 
 * This sends electrical signals to the motor to make it turn in the forward direction.
 */
void MotionMotors::right_forward(uint8_t power) {
  // Apply the calibration factor to adjust the motor power
  // This helps balance the motors if one is stronger than the other
  uint8_t calibratedPower = power * RIGHT_CALIBRATION;
  
  // Turn on the forward direction pin with our power level
  analogWrite(M_RIGHT_A_PIN, calibratedPower);
  
  // Make sure the backward direction pin is off (set to 0)
  analogWrite(M_RIGHT_B_PIN, 0);
  
  // Show helpful messages on the computer screen if debugging is turned on
  if (DEBUG_MOTOR_ACTIONS_VALUE) {
    // We only want to show a message when something important changes:
    // 1. If the motor changed direction (like from stopped to moving forward)
    bool directionChanged = (lastReportedRightFunction != &MotionMotors::right_forward);
    
    // 2. Or if the power level changed by more than 5 units
    bool powerChanged = (abs((int)power - (int)lastReportedRightPower) > 5);
    
    // If either important thing changed, show a message
    if (directionChanged || powerChanged) {
      Serial.printf("MOTOR: RIGHT FORWARD (Power: %d) [Right Joystick]\n", power);
      
      // Remember what we just reported so we don't repeat the same message
      lastReportedRightFunction = &MotionMotors::right_forward;
      lastReportedRightPower = power;
    }
  }
}

/**
 * Makes the right motor spin backward
 * 
 * This sends electrical signals to the motor to make it turn in the reverse direction.
 */
void MotionMotors::right_backward(uint8_t power) {
  // Apply the calibration factor to adjust the motor power
  // This helps balance the motors if one is stronger than the other
  uint8_t calibratedPower = power * RIGHT_CALIBRATION;
  
  // Make sure the forward direction pin is off (set to 0)
  analogWrite(M_RIGHT_A_PIN, 0);
  
  // Turn on the backward direction pin with our power level
  analogWrite(M_RIGHT_B_PIN, calibratedPower);
  
  // Show helpful messages on the computer screen if debugging is turned on
  if (DEBUG_MOTOR_ACTIONS_VALUE) {
    // We only want to show a message when something important changes:
    // 1. If the motor changed direction (like from stopped to moving backward)
    bool directionChanged = (lastReportedRightFunction != &MotionMotors::right_backward);
    
    // 2. Or if the power level changed by more than 5 units
    bool powerChanged = (abs((int)power - (int)lastReportedRightPower) > 5);
    
    // If either important thing changed, show a message
    if (directionChanged || powerChanged) {
      Serial.printf("MOTOR: RIGHT BACKWARD (Power: %d) [Right Joystick]\n", power);
      
      // Remember what we just reported so we don't repeat the same message
      lastReportedRightFunction = &MotionMotors::right_backward;
      lastReportedRightPower = power;
    }
  }
}

/**
 * Stops the right motor from moving
 * 
 * This turns off all power to the right motor so it stops spinning.
 */
void MotionMotors::right_stop() {
  // Turn off both direction pins to stop the motor
  analogWrite(M_RIGHT_A_PIN, 0);
  analogWrite(M_RIGHT_B_PIN, 0);
  
  // Only show a message if the motor was actually moving before
  // (We don't want messages about stopping an already stopped motor)
  if (DEBUG_MOTOR_ACTIONS_VALUE && lastReportedRightFunction != nullptr) {
    Serial.println("MOTOR: RIGHT STOP [Right Joystick]");
    
    // Reset our memory of what the motor was doing
    lastReportedRightFunction = nullptr;  // nullptr means "nothing" or "not moving"
    lastReportedRightPower = 0;           // Power is now zero
  }
}

/**
 * Makes motors speed up or slow down gradually (smoothly)
 * 
 * This is like how a car doesn't go from 0 to 60 mph instantly -
 * it gradually speeds up over time. This makes the robot move more smoothly.
 */
void MotionMotors::updateAcceleration() {
  // If we're not in the middle of a smooth speed change, do nothing
  if (!isAccelerating || !smoothEnabled) {
    return;
  }
  
  // Check if enough time has passed for the next small speed change
  unsigned long currentTime = millis();
  if (currentTime - lastAccelUpdateTime < SMOOTH_ACCEL_DELAY) {
    return;  // Not enough time has passed, so exit and try again later
  }
  
  // Remember when we did this update
  lastAccelUpdateTime = currentTime;
  
  // Move to the next step in our smooth acceleration
  currentAccelStep++;
  
  // Check if we've finished all the steps to reach our target speed
  if (currentAccelStep >= totalAccelSteps) {
    // We've reached the final speed! Set motors to their final power levels
    
    // Set left motor to final speed if it's being controlled
    if (leftTargetFunction != nullptr) {
      (this->*leftTargetFunction)(targetLeftPower);
      leftCurrentFunction = leftTargetFunction;
      leftCurrentPower = targetLeftPower;
    }
    
    // Set right motor to final speed if it's being controlled
    if (rightTargetFunction != nullptr) {
      (this->*rightTargetFunction)(targetRightPower);
      rightCurrentFunction = rightTargetFunction;
      rightCurrentPower = targetRightPower;
    }
    
    // We're done accelerating, so turn off the acceleration system
    isAccelerating = false;
    return;
  }
  
  // We're still accelerating, so calculate the next speed level
  
  // For the left motor (if it's being controlled)
  if (leftTargetFunction != nullptr) {
    // Calculate how far along we are in the acceleration (0.0 to 1.0)
    float progress = (float)currentAccelStep / totalAccelSteps;
    
    // Calculate the new power level based on our progress
    uint8_t power = targetLeftPower * progress;
    
    // Set the motor to this intermediate power level
    (this->*leftTargetFunction)(power);
  }
  
  // For the right motor (if it's being controlled)
  if (rightTargetFunction != nullptr) {
    // Calculate how far along we are in the acceleration (0.0 to 1.0)
    float progress = (float)currentAccelStep / totalAccelSteps;
    
    // Calculate the new power level based on our progress
    uint8_t power = targetRightPower * progress;
    
    // Set the motor to this intermediate power level
    (this->*rightTargetFunction)(power);
  }
}

/**
 * Begins a smooth speed change for the motors
 * 
 * This is like planning a trip - we decide where we want to go
 * and how we'll get there, but we haven't started moving yet.
 */
void MotionMotors::startAcceleration(
    void (MotionMotors::*leftFunc)(uint8_t), uint8_t leftPower,
    void (MotionMotors::*rightFunc)(uint8_t), uint8_t rightPower,
    uint8_t steps) {
  
  // Save what we want the motors to do when finished:
  // - Which direction each motor should move (forward/backward)
  // - How fast each motor should go
  leftTargetFunction = leftFunc;      // Direction for left motor
  rightTargetFunction = rightFunc;    // Direction for right motor
  targetLeftPower = leftPower;        // Final speed for left motor
  targetRightPower = rightPower;      // Final speed for right motor
  
  // Set up the acceleration process
  currentAccelStep = 0;               // Start at the beginning (step 0)
  totalAccelSteps = steps;            // How many small steps to take
  lastAccelUpdateTime = millis();     // Remember when we started
  isAccelerating = true;              // Turn on the acceleration system
}

/**
 * Gradually speeds up a motor from stopped to a target speed
 * 
 * This is like how you start walking before running - you don't
 * instantly jump to full speed!
 */
void MotionMotors::smoothAccelerate(void (MotionMotors::*moveFunction)(uint8_t), uint8_t targetPower,
                                   uint8_t steps, uint8_t delayMs) {
  // Check if smooth acceleration is turned off or not needed
  if (!smoothEnabled || steps <= 1) {
    // If smooth acceleration is off, just set the motor to the final speed immediately
    (this->*moveFunction)(targetPower);
    
    // Keep track of what the motor is doing
    if (moveFunction == &MotionMotors::left_forward || moveFunction == &MotionMotors::left_backward) {
      // It's the left motor
      leftCurrentFunction = moveFunction;  // Remember which direction it's moving
      leftCurrentPower = targetPower;      // Remember how fast it's going
    } else if (moveFunction == &MotionMotors::right_forward || moveFunction == &MotionMotors::right_backward) {
      // It's the right motor
      rightCurrentFunction = moveFunction; // Remember which direction it's moving
      rightCurrentPower = targetPower;     // Remember how fast it's going
    }
    return;
  }
  
  // Figure out which motor we're controlling and start the smooth acceleration
  
  // If it's the left motor
  if (moveFunction == &MotionMotors::left_forward || moveFunction == &MotionMotors::left_backward) {
    // Start accelerating the left motor only (right motor stays as is)
    startAcceleration(moveFunction, targetPower, nullptr, 0, steps);
  }
  // If it's the right motor
  else if (moveFunction == &MotionMotors::right_forward || moveFunction == &MotionMotors::right_backward) {
    // Start accelerating the right motor only (left motor stays as is)
    startAcceleration(nullptr, 0, moveFunction, targetPower, steps);
  }
}

/**
 * Gradually slows down a motor until it stops
 * 
 * This is like slowing down to a stop when walking, instead of
 * suddenly freezing in place.
 * 
 * Note: Currently this function just stops immediately, but in the future it
 * could be improved to slow down more gradually.
 */
void MotionMotors::smoothDecelerate(void (MotionMotors::*moveFunction)(uint8_t), uint8_t currentPower,
                                   uint8_t steps, uint8_t delayMs) {
  // Check if smooth deceleration is turned off or not needed
  if (!smoothEnabled || steps <= 1) {
    // If smooth deceleration is off, just stop the motor immediately
    
    // Figure out which motor we're controlling
    if (moveFunction == &MotionMotors::left_forward || moveFunction == &MotionMotors::left_backward) {
      // It's the left motor - stop it
      leftStop();
    } else if (moveFunction == &MotionMotors::right_forward || moveFunction == &MotionMotors::right_backward) {
      // It's the right motor - stop it
      rightStop();
    }
    return;
  }
  
  // For now, we just stop immediately
  // (In the future, this could be improved to slow down more gradually)
  
  // Figure out which motor we're controlling
  if (moveFunction == &MotionMotors::left_forward || moveFunction == &MotionMotors::left_backward) {
    // It's the left motor - stop it
    leftStop();
  } else if (moveFunction == &MotionMotors::right_forward || moveFunction == &MotionMotors::right_backward) {
    // It's the right motor - stop it
    rightStop();
  }
}

/**
 * Smoothly changes a motor from one direction/speed to another
 * 
 * This is like changing from walking forward to walking backward -
 * you need to slow down, stop, and then start going the other way.
 */
void MotionMotors::smoothTransition(void (MotionMotors::*currentFunction)(uint8_t), uint8_t currentPower,
                                   void (MotionMotors::*targetFunction)(uint8_t), uint8_t targetPower,
                                   uint8_t steps, uint8_t delayMs) {
  // Check if smooth transition is turned off or not needed
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
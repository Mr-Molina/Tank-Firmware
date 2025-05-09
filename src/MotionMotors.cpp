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
 * Makes the left motor go backward
 * 
 * This function tells the left motor to spin in reverse.
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
 * This function makes the left motor come to a stop by turning off power to both pins.
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
 * Makes the right motor go forward
 * 
 * This function tells the right motor to spin forward, which makes
 * the right side of the robot move forward.
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
 * Makes the right motor go backward
 * 
 * This function tells the right motor to spin in reverse, which makes
 * the right side of the robot move backward.
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
 * This function makes the right motor come to a stop by turning off power to both pins.
 * When both motors stop, the whole robot stops moving.
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
 * This function needs to be called over and over again in the main program loop.
 * It's like checking a timer - each time it runs, it updates the motor speed
 * a little bit until it reaches the final speed we want.
 * 
 * This makes the robot move more smoothly instead of jerking around.
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
 * This function sets up everything needed to start a gradual speed change.
 * It's like setting up a plan for how the motors will change speed over time.
 * 
 * After calling this, the updateAcceleration function will take care of
 * actually changing the speeds bit by bit.
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
 * This function makes a motor start moving smoothly instead of jerking
 * to full speed right away. It's like how a car gradually speeds up
 * rather than jumping instantly to 60 mph!
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
 * This function makes a motor slow down smoothly instead of stopping suddenly.
 * It's like how a car gradually slows down when you take your foot off the gas,
 * rather than slamming on the brakes!
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
 * This function helps a motor change what it's doing without jerky movements.
 * For example, if a motor is going forward and needs to go backward,
 * this function can help it slow down first, then start going backward.
 * 
 * It's like how a car doesn't instantly switch from driving forward to
 * driving backward - it has to stop first, then start going the other way.
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
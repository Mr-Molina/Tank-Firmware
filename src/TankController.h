#ifndef TANK_CONTROLLER_H
#define TANK_CONTROLLER_H

#include <Arduino.h>
#include "PS4-Remote.h"
#include "MotionMotors.h"

class TankController
{
private:
     // PS4 controller configuration
     int events;
     int buttons;
     int joysticks;
     int sensors;
     int useAccelerometer;
     int deadzone;
     int gyroDeadzone;
     int accDeadzone;
     int accPrecision;

     // Motor configuration
     int leftMotorPinA;
     int leftMotorPinB;
     int rightMotorPinA;
     int rightMotorPinB;
     float leftMotorCalibration;
     float rightMotorCalibration;
     int maxMotorSpeed;
     float turnSpeedFactor;

     // Controller and motor instances
     PS4Remote ps4;
     MotionMotors motors;

     // State tracking
     bool lastShareState;

     // Private methods
     void controlMotorsWithJoystick(PS4Remote::ControllerState state);

     // Static callback methods
     static void onConnect();
     static void onDisconnect();

     // Static instance pointer for callbacks
     static TankController *instance;

public:
     TankController(
         // PS4 controller config
         int events, int buttons, int joysticks, int sensors,
         int useAccelerometer, int deadzone, int gyroDeadzone,
         int accDeadzone, int accPrecision,
         // Motor config
         int leftMotorPinA, int leftMotorPinB,
         int rightMotorPinA, int rightMotorPinB,
         float leftMotorCalibration, float rightMotorCalibration,
         int maxMotorSpeed, float turnSpeedFactor);

     void begin();
     void update();
};

#endif // TANK_CONTROLLER_H
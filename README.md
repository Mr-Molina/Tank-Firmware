# Tank-Firmware

## 🤖 Robot Tank Controller for Middle School Students

This project provides firmware for a tank-style robot controlled by a PS4 controller. It's designed to be educational, fun, and accessible for middle school students learning about robotics and programming.

![Robot Tank Illustration](https://via.placeholder.com/800x400?text=Robot+Tank+Project)

## 📋 Overview

The Tank-Firmware project allows you to control a two-motor robot using a PS4 controller. The robot uses "tank drive" style controls where:
- Left joystick controls the left motor
- Right joystick controls the right motor

This makes the robot easy to drive and understand - push both joysticks forward to go straight, pull both back to reverse, or push one forward and one back to turn in place!

## 🎮 Controller Features

### Basic Controls
- **Left Joystick**: Controls the left motor
- **Right Joystick**: Controls the right motor

### Special Buttons
- **Share Button**: Toggles smooth acceleration on/off
- **Options Button**: Emergency stop (immediately stops all motors)
- **D-pad Up**: Increase left motor power
- **D-pad Down**: Decrease left motor power
- **Triangle Button**: Increase right motor power
- **X Button**: Decrease right motor power

## 🔧 Hardware Requirements

- ESP32 microcontroller board
- Two DC motors with motor driver
- PS4 controller
- Battery pack (for portable operation)
- Robot chassis

## 🔌 Wiring

The default pin connections are:
- Left Motor: Pins 18 (forward) and 19 (backward)
- Right Motor: Pins 16 (forward) and 17 (backward)

You can change these in the `main.cpp` file if needed.

## ⚙️ Customizable Settings

You can easily customize how your robot works by changing settings in the `main.cpp` file:

### Controller Settings
```cpp
#define EVENTS 1            // Track button press events
#define BUTTONS 1           // Use controller buttons
#define JOYSTICKS 1         // Use the joysticks
#define SENSORS 1           // Use controller sensors
#define USE_ACCELEROMETER 0 // Use motion sensors (tilt detection)
```

### Joystick Sensitivity
```cpp
#define DEADZONE 10         // How far to move joystick before robot responds
```

### Motor Settings
```cpp
#define LEFT_MOTOR_CALIBRATION 1.0  // Left motor power factor
#define RIGHT_MOTOR_CALIBRATION 1.0 // Right motor power factor
#define MAX_MOTOR_SPEED 255         // Maximum motor speed
#define TURN_SPEED_FACTOR 0.7       // How fast the robot turns
```

## 🚀 Getting Started

1. **Setup the Hardware**:
   - Connect the motors to the motor driver
   - Connect the motor driver to the ESP32 using the pins defined in `main.cpp`
   - Power up the system

2. **Upload the Firmware**:
   - Clone this repository
   - Open the project in PlatformIO or Arduino IDE
   - Upload the firmware to your ESP32

3. **Connect the Controller**:
   - Turn on your PS4 controller
   - Press and hold the Share button and the PS button until the light bar flashes
   - The controller should connect to the ESP32 automatically

4. **Drive Your Robot**:
   - Use the left and right joysticks to control the robot
   - Experiment with the special buttons to adjust motor power and driving style

## 🔍 Code Structure

- **main.cpp**: Main program with settings and initialization
- **TankController.h/cpp**: Connects controller input to motor movements
- **MotionMotors.h/cpp**: Controls the robot's motors
- **PS4-Remote.h/cpp**: Handles communication with the PS4 controller

## 🛠️ Advanced Features

### Smooth Acceleration
The robot can gradually speed up and slow down for smoother movement. Toggle this feature with the Share button on the controller.

### Motor Calibration
If one motor is stronger than the other, you can adjust their relative power using the D-pad Up/Down (left motor) and Triangle/X buttons (right motor).

### Safety Features
- Motors automatically stop if the controller disconnects
- Emergency stop button (Options) immediately stops all motors

## 🔮 Future Enhancements

Some ideas for expanding the project:
- Add sensors for obstacle detection
- Implement autonomous navigation modes
- Add LED indicators or a small display
- Create custom attachments (arm, gripper, etc.)

## 📚 Learning Opportunities

This project helps students learn about:
- Programming concepts (variables, functions, conditionals)
- Electronics and circuit connections
- Wireless communication (Bluetooth)
- Motor control and calibration
- User interface design
- Robotics principles

## 🤝 Contributing

Contributions to improve the project are welcome! Please feel free to submit pull requests or open issues with suggestions.

## 📄 License

This project is open source and available under the MIT License.

---

Created for educational purposes to help middle school students learn about robotics and programming. Happy building and coding! 🤖
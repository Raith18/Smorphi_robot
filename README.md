# Smorphi Autonomous Robot with HuskyLens

This project provides a complete autonomous control system for the Smorphi mobile robot with integrated HuskyLens object detection capabilities.

## Features

- **Autonomous Navigation**: The robot can navigate autonomously while avoiding obstacles
- **Object Detection**: Uses HuskyLens camera for real-time object detection and tracking
- **Object Following**: Automatically follows detected objects while maintaining optimal distance
- **Obstacle Avoidance**: Ultrasonic sensor-based obstacle detection and avoidance
- **Multiple Behavior States**: Idle, searching, moving to target, avoiding obstacles, and following objects

## Hardware Requirements

- Smorphi mobile robot platform
- HuskyLens AI camera sensor
- HC-SR04 ultrasonic distance sensor (or compatible)
- Arduino-compatible microcontroller (tested with Arduino Uno/Mega)
- 4 DC motors with motor driver shield
- Jumper wires and breadboard

## Software Requirements

- Arduino IDE 1.8.0 or later
- No additional libraries required (custom HuskyLens implementation included)

## Installation

1. **Hardware Setup**:
   - Connect HuskyLens to Arduino pins D5 (TX) and D6 (RX)
   - Connect ultrasonic sensor to pins D8 (TRIG) and D9 (ECHO)
   - Connect motor driver to motor control pins D2, D3, D4, D5
   - Connect direction control pins to D10, D11, D12, D13
   - Connect status LED to pin D7

2. **Software Setup**:
   - Clone or download this repository
   - Open `smorphi_autonomous.ino` in Arduino IDE
   - Compile and upload the code to your Arduino board

## Configuration

Edit `robot_config.h` to adjust robot parameters:

```cpp
// Motor speeds
#define BASE_SPEED 150
#define TURN_SPEED 100

// Object detection
#define OBJECT_SIZE_THRESHOLD 50
#define DISTANCE_THRESHOLD_CM 20

// Navigation
#define CAMERA_CENTER_X 160
#define CAMERA_CENTER_Y 120
```

## Usage

1. **Power on the robot**: The robot will start in idle mode
2. **Automatic operation**: After 5 seconds, the robot enters search mode and begins looking for objects
3. **Object detection**: When an object is detected, the robot moves toward it
4. **Object following**: Once the object is centered and at appropriate distance, the robot follows it
5. **Obstacle avoidance**: If obstacles are detected, the robot backs up and turns to avoid them

## Behavior States

- **IDLE**: Robot stopped, LED off, waiting to start
- **SEARCHING**: Robot rotates slowly, searching for objects (LED on)
- **MOVING_TO_TARGET**: Robot moves toward detected object
- **AVOIDING_OBSTACLE**: Robot backs up and turns to avoid obstacles
- **FOLLOWING_OBJECT**: Robot follows the detected object while maintaining distance

## HuskyLens Integration

The system includes a custom HuskyLens library that provides:
- Real-time object detection
- Object position tracking (X, Y coordinates)
- Object size measurement
- Object ID recognition

## Serial Monitor

Open the Serial Monitor (9600 baud) to see:
- Current robot state
- Detected object information
- Obstacle detection alerts
- Navigation decisions

## Troubleshooting

**Robot not moving**:
- Check motor connections and power supply
- Verify motor driver is properly connected
- Check if motors are enabled in the code

**HuskyLens not detecting objects**:
- Ensure HuskyLens is properly connected
- Check if objects are within the camera's field of view
- Verify HuskyLens firmware is up to date

**Obstacle avoidance not working**:
- Check ultrasonic sensor connections
- Ensure sensor is mounted at the front of the robot
- Verify distance threshold settings

## Customization

**Adding new behaviors**:
- Modify the `RobotState` enum in `smorphi_autonomous.ino`
- Add new handler methods in `SmorphiRobot` class
- Update the `update()` method to include new states

**Adjusting navigation parameters**:
- Modify timing values in `robot_config.h`
- Adjust speed and distance thresholds
- Fine-tune object detection parameters

## File Structure

```
smorphi_autonomous/
├── smorphi_autonomous.ino    # Main Arduino sketch
├── SmorphiRobot.h           # Robot class definition
├── HUSKYLENS.h              # HuskyLens library header
├── HUSKYLENS.cpp            # HuskyLens library implementation
├── robot_config.h           # Configuration parameters
└── README.md               # This file
```

## License

This project is open source and available under the MIT License.

## Support

For issues and questions:
1. Check the troubleshooting section
2. Verify all connections match the pin definitions
3. Monitor serial output for debugging information
4. Ensure all hardware components are functioning properly

## Version History

- v1.0.0: Initial release with autonomous navigation and object following
- Includes HuskyLens integration and obstacle avoidance
- Custom library implementations for all dependencies
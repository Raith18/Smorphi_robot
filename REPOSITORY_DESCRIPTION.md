# Smorphi Autonomous Robot Control System

A comprehensive autonomous control system for the Smorphi mobile robot platform with integrated HuskyLens AI camera for intelligent object detection, tracking, and following capabilities.

## 🤖 Key Features

- **Autonomous Navigation**: Advanced path planning and obstacle avoidance using ultrasonic sensors
- **AI-Powered Object Detection**: Real-time object recognition and tracking via HuskyLens AI camera
- **Intelligent Object Following**: Dynamic distance maintenance and smooth pursuit of detected objects
- **Multi-State Behavior System**: Sophisticated state machine managing idle, search, targeting, and following modes
- **Custom Hardware Integration**: Optimized for Smorphi robot platform with 4-wheel differential drive

## 🛠️ Technical Implementation

- **Custom HuskyLens Library**: Lightweight implementation for seamless AI camera integration
- **Advanced Motor Control**: Precise 4-wheel speed and direction management
- **Sensor Fusion**: Ultrasonic distance sensing combined with computer vision
- **Configurable Parameters**: Extensive customization options for robot behavior tuning
- **Serial Debugging**: Comprehensive monitoring and diagnostic output

## 📁 Project Structure

```
├── smorphi_autonomous.ino    # Main autonomous control firmware
├── SmorphiRobot.h           # Robot behavior class definition
├── HUSKYLENS.h              # AI camera interface header
├── HUSKYLENS.cpp            # AI camera implementation
├── robot_config.h           # System configuration parameters
└── README.md               # Comprehensive documentation
```

## 🎯 Use Cases

- **Research & Education**: Perfect for robotics learning and autonomous systems research
- **Object Tracking**: Ideal for following applications in dynamic environments
- **Autonomous Navigation**: Demonstrates advanced path planning and obstacle avoidance
- **Computer Vision Integration**: Showcases AI camera applications in mobile robotics

## 🔧 Hardware Requirements

- Smorphi mobile robot platform
- HuskyLens AI camera sensor
- HC-SR04 ultrasonic distance sensor
- Arduino-compatible microcontroller
- 4x DC motors with motor driver

## 🚀 Quick Start

1. Upload `smorphi_autonomous.ino` to Arduino board
2. Connect HuskyLens to designated serial pins
3. Mount ultrasonic sensor on robot front
4. Power on and observe autonomous behavior

*Ready to deploy autonomous robot with just a few connections!*

---

*Built with ❤️ for the robotics community* | *Open source under MIT License*
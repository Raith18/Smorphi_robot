#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// Motor pin definitions
#define MOTOR_LEFT_FRONT_PIN 2
#define MOTOR_LEFT_REAR_PIN 3
#define MOTOR_RIGHT_FRONT_PIN 4
#define MOTOR_RIGHT_REAR_PIN 5

// Motor direction pins (if using separate direction control)
#define MOTOR_LEFT_FRONT_DIR 10
#define MOTOR_LEFT_REAR_DIR 11
#define MOTOR_RIGHT_FRONT_DIR 12
#define MOTOR_RIGHT_REAR_DIR 13

// Ultrasonic sensor pins
#define ULTRASONIC_TRIG_PIN 8
#define ULTRASONIC_ECHO_PIN 9

// LED indicator pin
#define STATUS_LED_PIN 7

// HuskyLens communication pins
#define HUSKYLENS_RX_PIN 6
#define HUSKYLENS_TX_PIN 5

// Robot behavior parameters
#define BASE_SPEED 150
#define TURN_SPEED 100
#define MAX_SPEED 255
#define MIN_SPEED 80

// Object detection parameters
#define CAMERA_CENTER_X 160
#define CAMERA_CENTER_Y 120
#define OBJECT_SIZE_THRESHOLD 50
#define DISTANCE_THRESHOLD_CM 20

// Timing parameters (milliseconds)
#define DETECTION_TIMEOUT 2000
#define OBSTACLE_CHECK_INTERVAL 100
#define MAIN_LOOP_DELAY 50

// Navigation parameters
#define POSITION_TOLERANCE 15
#define SIZE_TOLERANCE 10
#define FOLLOW_DISTANCE_MIN 60
#define FOLLOW_DISTANCE_MAX 120

// Robot physical parameters
#define WHEEL_DIAMETER_MM 65
#define WHEEL_BASE_MM 120
#define MAX_TURN_RATE 180  // degrees per second

#endif
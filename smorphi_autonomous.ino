#include <SoftwareSerial.h>
#include "HUSKYLENS.h"
#include "SmorphiRobot.h"

// Pin definitions for Smorphi robot
#define MOTOR_LEFT_FRONT 2
#define MOTOR_LEFT_REAR 3
#define MOTOR_RIGHT_FRONT 4
#define MOTOR_RIGHT_REAR 5

#define ULTRASONIC_TRIG 8
#define ULTRASONIC_ECHO 9

#define LED_PIN 13

// HuskyLens SoftwareSerial
#define HUSKY_RX 10
#define HUSKY_TX 11
SoftwareSerial huskySerial(HUSKY_RX, HUSKY_TX);
HUSKYLENS huskylens;

// Robot state definitions
enum RobotState {
  STATE_IDLE,
  STATE_SEARCHING,
  STATE_MOVING_TO_TARGET,
  STATE_AVOIDING_OBSTACLE,
  STATE_FOLLOWING_OBJECT
};

RobotState currentState = STATE_IDLE;

// Object detection variables
int detectedObjectX = 0;
int detectedObjectY = 0;
int detectedObjectWidth = 0;
int detectedObjectHeight = 0;
String detectedObjectID = "";

// Navigation variables
int targetX = 160; // Center of camera frame (320px width)
int targetY = 120; // Center of camera frame (240px height)
int distanceThreshold = 20; // cm
int objectSizeThreshold = 50; // Minimum object size to consider

// Motor speed variables
int baseSpeed = 150;
int turnSpeed = 100;
int maxSpeed = 255;

// Timing variables
unsigned long lastDetectionTime = 0;
unsigned long detectionTimeout = 2000; // 2 seconds
unsigned long lastObstacleCheck = 0;
unsigned long obstacleCheckInterval = 100; // 100ms

SmorphiRobot::SmorphiRobot() {
  // Constructor - initialize robot
}

void SmorphiRobot::begin() {
  // Initialize HuskyLens
  huskySerial.begin(9600);
  while (!huskylens.begin(huskySerial)) {
    Serial.println("HuskyLens connection failed, retrying...");
    delay(1000);
  }
  Serial.println("HuskyLens connected successfully!");

  // Initialize ultrasonic sensor
  pinMode(ULTRASONIC_TRIG, OUTPUT);
  pinMode(ULTRASONIC_ECHO, INPUT);

  // Initialize LED
  pinMode(LED_PIN, OUTPUT);

  // Set initial state
  currentState = STATE_SEARCHING;
}

void SmorphiRobot::update() {
  unsigned long currentTime = millis();

  // Check for obstacles periodically
  if (currentTime - lastObstacleCheck >= obstacleCheckInterval) {
    checkObstacles();
    lastObstacleCheck = currentTime;
  }

  // Update based on current state
  switch (currentState) {
    case STATE_IDLE:
      handleIdleState();
      break;
    case STATE_SEARCHING:
      handleSearchingState();
      break;
    case STATE_MOVING_TO_TARGET:
      handleMovingToTargetState();
      break;
    case STATE_AVOIDING_OBSTACLE:
      handleAvoidingObstacleState();
      break;
    case STATE_FOLLOWING_OBJECT:
      handleFollowingObjectState();
      break;
  }
}

void SmorphiRobot::checkObstacles() {
  int distance = getUltrasonicDistance();

  if (distance > 0 && distance < distanceThreshold) {
    if (currentState != STATE_AVOIDING_OBSTACLE) {
      Serial.print("Obstacle detected at distance: ");
      Serial.println(distance);
      currentState = STATE_AVOIDING_OBSTACLE;
    }
  }
}

int SmorphiRobot::getUltrasonicDistance() {
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG, LOW);

  long duration = pulseIn(ULTRASONIC_ECHO, HIGH, 30000); // 30ms timeout
  if (duration == 0) return -1;

  int distance = duration * 0.034 / 2; // Convert to cm
  return distance;
}

void SmorphiRobot::handleIdleState() {
  stopMotors();
  digitalWrite(LED_PIN, LOW);

  // Check if we should start searching
  if (millis() - lastDetectionTime > 5000) { // Wait 5 seconds
    currentState = STATE_SEARCHING;
  }
}

void SmorphiRobot::handleSearchingState() {
  digitalWrite(LED_PIN, HIGH);

  // Rotate slowly while searching for objects
  setMotorSpeeds(turnSpeed, -turnSpeed, turnSpeed, -turnSpeed);

  // Try to detect objects with HuskyLens
  if (detectObjects()) {
    currentState = STATE_MOVING_TO_TARGET;
    Serial.println("Object detected, moving to target");
  }
}

void SmorphiRobot::handleMovingToTargetState() {
  if (detectObjects()) {
    lastDetectionTime = millis();

    // Calculate error from center
    int errorX = detectedObjectX - targetX;
    int errorY = detectedObjectY - targetY;

    // If object is roughly centered and close enough, follow it
    if (abs(errorX) < 30 && abs(errorY) < 30 && detectedObjectWidth > objectSizeThreshold) {
      currentState = STATE_FOLLOWING_OBJECT;
      Serial.println("Object centered, switching to follow mode");
      return;
    }

    // Adjust direction based on object position
    if (abs(errorX) > 20) {
      if (errorX > 0) {
        // Object is to the right, turn right
        setMotorSpeeds(-turnSpeed, -turnSpeed, turnSpeed, turnSpeed);
      } else {
        // Object is to the left, turn left
        setMotorSpeeds(turnSpeed, turnSpeed, -turnSpeed, -turnSpeed);
      }
    } else {
      // Object is centered horizontally, move forward
      setMotorSpeeds(baseSpeed, baseSpeed, baseSpeed, baseSpeed);
    }

    // Adjust speed based on object size (closer = larger)
    int dynamicSpeed = baseSpeed;
    if (detectedObjectWidth > 100) {
      dynamicSpeed = baseSpeed / 2; // Slow down if very close
    } else if (detectedObjectWidth < 30) {
      dynamicSpeed = baseSpeed * 1.5; // Speed up if far
    }

    // Apply dynamic speed if we're moving forward
    if (abs(errorX) < 20) {
      setMotorSpeeds(dynamicSpeed, dynamicSpeed, dynamicSpeed, dynamicSpeed);
    }

  } else {
    // Lost object, go back to searching
    if (millis() - lastDetectionTime > detectionTimeout) {
      currentState = STATE_SEARCHING;
      Serial.println("Object lost, returning to search mode");
    }
  }
}

void SmorphiRobot::handleAvoidingObstacleState() {
  // Back up and turn
  setMotorSpeeds(-baseSpeed, -baseSpeed, -baseSpeed, -baseSpeed);
  delay(500);

  // Turn right to avoid obstacle
  setMotorSpeeds(-turnSpeed, -turnSpeed, turnSpeed, turnSpeed);
  delay(800);

  // Check if obstacle is cleared
  int distance = getUltrasonicDistance();
  if (distance < 0 || distance >= distanceThreshold) {
    currentState = STATE_SEARCHING;
    Serial.println("Obstacle cleared, returning to search mode");
  }
}

void SmorphiRobot::handleFollowingObjectState() {
  if (detectObjects()) {
    lastDetectionTime = millis();

    // Keep object centered while maintaining distance
    int errorX = detectedObjectX - targetX;
    int errorY = detectedObjectY - targetY;

    // If object moves too far from center, go back to targeting
    if (abs(errorX) > 40 || abs(errorY) > 40) {
      currentState = STATE_MOVING_TO_TARGET;
      return;
    }

    // Adjust direction to keep object centered
    if (abs(errorX) > 15) {
      if (errorX > 0) {
        setMotorSpeeds(turnSpeed/2, turnSpeed/2, -turnSpeed/2, -turnSpeed/2);
      } else {
        setMotorSpeeds(-turnSpeed/2, -turnSpeed/2, turnSpeed/2, turnSpeed/2);
      }
    } else {
      // Maintain distance based on object size
      int dynamicSpeed = baseSpeed / 2; // Default follow speed

      if (detectedObjectWidth > 120) {
        dynamicSpeed = baseSpeed / 4; // Very close, slow down
      } else if (detectedObjectWidth < 60) {
        dynamicSpeed = baseSpeed / 1.5; // Far, speed up a bit
      }

      setMotorSpeeds(dynamicSpeed, dynamicSpeed, dynamicSpeed, dynamicSpeed);
    }

  } else {
    // Lost object while following
    if (millis() - lastDetectionTime > detectionTimeout) {
      currentState = STATE_SEARCHING;
      Serial.println("Object lost while following, returning to search mode");
    }
  }
}

bool SmorphiRobot::detectObjects() {
  if (huskylens.request()) {
    if (huskylens.available()) {
      HUSKYLENSResult result = huskylens.read();
      detectedObjectX = result.xCenter;
      detectedObjectY = result.yCenter;
      detectedObjectWidth = result.width;
      detectedObjectHeight = result.height;
      detectedObjectID = String(result.ID);

      Serial.print("Detected object ID: ");
      Serial.print(detectedObjectID);
      Serial.print(" at (");
      Serial.print(detectedObjectX);
      Serial.print(",");
      Serial.print(detectedObjectY);
      Serial.print(") size: ");
      Serial.print(detectedObjectWidth);
      Serial.print("x");
      Serial.println(detectedObjectHeight);

      return true;
    }
  }
  return false;
}

void SmorphiRobot::setMotorSpeeds(int leftFront, int leftRear, int rightFront, int rightRear) {
  // Set motor speeds with direction control
  analogWrite(MOTOR_LEFT_FRONT, abs(leftFront));
  analogWrite(MOTOR_LEFT_REAR, abs(leftRear));
  analogWrite(MOTOR_RIGHT_FRONT, abs(rightFront));
  analogWrite(MOTOR_RIGHT_REAR, abs(rightRear));

  // Set motor directions (assuming digital pins control direction)
  digitalWrite(MOTOR_LEFT_FRONT + 8, leftFront >= 0 ? HIGH : LOW); // Direction pins
  digitalWrite(MOTOR_LEFT_REAR + 8, leftRear >= 0 ? HIGH : LOW);
  digitalWrite(MOTOR_RIGHT_FRONT + 8, rightFront >= 0 ? HIGH : LOW);
  digitalWrite(MOTOR_RIGHT_REAR + 8, rightRear >= 0 ? HIGH : LOW);
}

void SmorphiRobot::stopMotors() {
  analogWrite(MOTOR_LEFT_FRONT, 0);
  analogWrite(MOTOR_LEFT_REAR, 0);
  analogWrite(MOTOR_RIGHT_FRONT, 0);
  analogWrite(MOTOR_RIGHT_REAR, 0);
}

// Global robot instance
SmorphiRobot robot;

void setup() {
  Serial.begin(9600);
  Serial.println("Smorphi Autonomous Robot Starting...");

  robot.begin();
}

void loop() {
  robot.update();
  delay(50); // Small delay for stability
}
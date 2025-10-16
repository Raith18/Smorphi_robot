#ifndef SMORPHI_ROBOT_H
#define SMORPHI_ROBOT_H

#include <Arduino.h>

class SmorphiRobot {
public:
    SmorphiRobot();
    void begin();
    void update();

private:
    void checkObstacles();
    int getUltrasonicDistance();
    void handleIdleState();
    void handleSearchingState();
    void handleMovingToTargetState();
    void handleAvoidingObstacleState();
    void handleFollowingObjectState();
    bool detectObjects();
    void setMotorSpeeds(int leftFront, int leftRear, int rightFront, int rightRear);
    void stopMotors();
};

#endif
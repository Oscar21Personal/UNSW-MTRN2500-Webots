#include "ScoutRobot.hpp"

ScoutRobot::ScoutRobot() 
    : leftMotor{ getMotor("left wheel motor") }
    , rightMotor{ getMotor("right wheel motor") }
    , camera{ getCamera("camera") } {

    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
  
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);
    
    camera->recognitionEnable(TIME_STEP);
    
    std::array<std::string, 8> psNames {
      "ps0", "ps1", "ps2", "ps3",
      "ps4", "ps5", "ps6", "ps7"
    };
    for (int i = 0; i < 8; i++) {
      ps[i].reset(getDistanceSensor(psNames[i]));
      ps[i]->enable(TIME_STEP);
    }
}

void ScoutRobot::run() {   
    
    int phase{ 0 };
    int count{ 0 };
    
    while (step(TIME_STEP) != -1) {
        updateCurrentPosition();
        switch (phase) {
            case 0:
                scoutPhase0(phase);
                break;
            case 1:
                scoutPhase1(phase);
                break;
            case 2:
                scoutPhase2(phase, count);
                break;
            case 3:
                // Tasks ended
                break;
        }      
    }
}

void ScoutRobot::move(double speed) {
    leftMotor->setVelocity(speed);
    rightMotor->setVelocity(speed);
}

void ScoutRobot::rotate(double speed) {
    leftMotor->setVelocity(speed);
    rightMotor->setVelocity(-speed);
}

bool ScoutRobot::readColour() {
    if (camera->getRecognitionNumberOfObjects() == 1) {
        return true;
    }
    return false;
}

// Helper functions below
bool ScoutRobot::detectObstacles(double obstacleThreshold) {
    std::array<double, 8> psValues{};
    for (int i = 0; i < 8 ; i++) {
      psValues[i] = ps[i]->getValue();
    }
    bool rightObstacle{ psValues[0] > obstacleThreshold || psValues[1] > obstacleThreshold || psValues[2] > obstacleThreshold };
    bool leftObstacle{ psValues[5] > obstacleThreshold || psValues[6] > obstacleThreshold || psValues[7] > obstacleThreshold };
    return (leftObstacle || rightObstacle);
}

void ScoutRobot::scoutPhase0(int& phase) {
    std::pair<std::string, std::string> target{ receiveMessage() };
    if (!target.first.empty() && !target.second.empty()) {
        setTargetPosition(std::stod(target.first), std::stod(target.second));
        phase = 1;
    }
}

void ScoutRobot::scoutPhase1(int& phase) { 
    if (detectObstacles(80.0)) {
        move(0);
        phase = 2;
    } else if (moveToTarget(0.4)) {
        if (readColour()) {
            sendMessage("0", std::to_string(targetPositionX) + " " + std::to_string(targetPositionY), ID);
            phase = 3;
        } else {
            sendMessage("0", "infinity", ID);
            phase = 0;
        }
    }  
}

void ScoutRobot::scoutPhase2(int& phase, int& count) {
    if (count < 20) {
        move(-maxSpeed);
    }
    else if (count < 60) {
        rotate(-maxSpeed);
    } 
    else if (count < 80) {
        move(maxSpeed);
    } 
    else if (count < 110) {
      count = 0;
      phase = 1;
    }
    count++;
}
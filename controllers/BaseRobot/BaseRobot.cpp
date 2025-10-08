// File:          BaseRobot.cpp
// Date:          04/11/2023
// Description:   Implementation of BaseRobot to be inherited by the Leader and Scout robots classes.
// Author:        Yee Lap Pong
// zID:           z5376231
// Modifications:

#include "BaseRobot.hpp"

BaseRobot::BaseRobot()
    : ID{ getName() }
    , receiver{ getReceiver("receiver") }
    , emitter{ getEmitter("emitter") }
    , keyboard{ getKeyboard() }
    , gps{ getGPS("gps") }
    , compass { getCompass("compass") } {
    
    receiver->enable(TIME_STEP);
    keyboard->enable(TIME_STEP); 
    gps->enable(TIME_STEP);    
    compass->enable(TIME_STEP);
    
    if (ID == "0") {
        maxSpeed = 26;
    } else {
        maxSpeed = 6.28;
    }
}

void BaseRobot::keyboardControl() {    
    int key{ keyboard->getKey() };
    if (key == 87) {  
        move(maxSpeed);
    }
    else if (key == 83) {  
        move(-maxSpeed);
    }
    else if (key == 65) {  
        rotate(-maxSpeed);
    }
    else if (key == 68) {  
        rotate(maxSpeed);
    }   
    else {
        move(0);
    }  
}

void BaseRobot::updateCurrentPosition() {
    currentPositionX = gps->getValues()[0];
    currentPositionY = gps->getValues()[1];
    currentYaw = atan2(compass->getValues()[0], compass->getValues()[1]);
}

void BaseRobot::setTargetPosition(double x, double y) {
    targetPositionX = x;
    targetPositionY = y;
}

bool BaseRobot::moveToTarget(double stopDistance) {
    double currentDistance { sqrt(pow(targetPositionX - currentPositionX, 2) + pow(targetPositionY - currentPositionY, 2)) };
    double targetAngle { atan2(targetPositionY - currentPositionY, targetPositionX - currentPositionX) };
    if ((currentYaw > targetAngle + ANGLE_RANGE) && (currentYaw < M_PI - ANGLE_RANGE)) {
        rotate(0.5 * maxSpeed);
        return false;
    } 
    if ((currentYaw < targetAngle - ANGLE_RANGE) && (currentYaw > -M_PI + ANGLE_RANGE)) {
        rotate(-0.5 * maxSpeed);
        return false;
    } 
    if (currentDistance > stopDistance) {
        move(maxSpeed);
        return false;
    }
    move(0);
    return true;
}

void BaseRobot::sendMessage(const std::string& ID, const std::string& data0, const std::string& data1) {
    std::cout << "Sending message to " << ID << std::endl;
    std::string message{};
    message.append(ID);
    message.append("|");
    message.append(data0);
    message.append("|");
    message.append(data1);
    emitter->send(message.c_str(), (int)strlen(message.c_str()) + 1);
}


std::pair<std::string, std::string> BaseRobot::receiveMessage() {
    if (receiver->getQueueLength() > 0) {
        std::string message{ static_cast<const char*>(receiver->getData()) };
        receiver->nextPacket();

        std::istringstream iss{ message };
        std::string incomingId{};
        std::getline(iss, incomingId, '|');

        if (ID.compare(incomingId) == 0) {
            // ID matches, now extract data0 and data1
            std::string data0{};
            std::string data1{};
            if (std::getline(iss, data0, '|') && std::getline(iss, data1, '|')) {
                std::cout << "Received message with matching ID: " << message << std::endl;
                return std::make_pair(data0, data1);
            }
        }
    }
    // If the ID doesn't match or the format is incorrect, return an empty pair
    return std::make_pair("", "");
}
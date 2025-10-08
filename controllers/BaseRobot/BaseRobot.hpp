// File:          BaseRobot.hpp
// Date:          04/11/2023
// Description:   Header file of BaseRobot to be inherited by the Leader and Scout robots classes.
// Author:        Yee Lap Pong
// zID:           z5376231
// Modifications:

#pragma once

// add additional includes as needed
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Emitter.hpp>
#include <webots/GPS.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Lidar.hpp>
#include <webots/Motor.hpp>
#include <webots/Receiver.hpp>
#include <webots/Robot.hpp>

constexpr int TIME_STEP{ 64 };
constexpr double ANGLE_RANGE { 0.1 };
constexpr int NUM_OF_SCOUTS{ 3 };

class BaseRobot : public webots::Robot {
public:
	BaseRobot();
	virtual ~BaseRobot() = default;
  
	virtual void run() = 0;
	virtual void move(double speed) = 0;
	virtual void rotate(double speed) = 0;

	void keyboardControl();
	void updateCurrentPosition();
	void setTargetPosition(double x, double y);
	bool moveToTarget(double stopDistance);
	   
	void sendMessage(const std::string& ID, const std::string& data0, const std::string& data1);
	std::pair<std::string, std::string> receiveMessage();

protected:
	std::string ID{};
	double currentPositionX{};
	double currentPositionY{};
	double currentYaw{};
	double targetPositionX{};
	double targetPositionY{};

	// add additional members as needed
	double maxSpeed{};	
  
private:
	std::unique_ptr<webots::Receiver> receiver{};
	std::unique_ptr<webots::Emitter> emitter{};
    
	// add additional members as needed
	std::unique_ptr<webots::Keyboard> keyboard{};
	std::unique_ptr<webots::GPS> gps{};
	std::unique_ptr<webots::Compass> compass{};
           
};
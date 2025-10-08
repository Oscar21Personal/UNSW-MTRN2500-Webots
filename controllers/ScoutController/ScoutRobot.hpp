#pragma once

#include "../BaseRobot/BaseRobot.hpp"

class ScoutRobot : public BaseRobot {
public:
	ScoutRobot();
	~ScoutRobot() = default;
  
	void run() override;
	void move(double speed) override;
	void rotate(double speed) override;
		
	bool readColour();
  
private:	
	std::unique_ptr<webots::Motor> leftMotor{};
	std::unique_ptr<webots::Motor> rightMotor{};
	std::unique_ptr<webots::Camera> camera{};
	
	std::array<std::unique_ptr<webots::DistanceSensor>, 8> ps{};
	 
	// Helper functions
	bool detectObstacles(double obstacleThreshold);
	void scoutPhase0(int& phase);
	void scoutPhase1(int& phase);
	void scoutPhase2(int& phase, int& count);

};
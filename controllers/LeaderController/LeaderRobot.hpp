#pragma once

#include "../BaseRobot/BaseRobot.hpp"

const std::string CONFIG_FILE{"keyboardConfig.txt"};
const std::string OUTPUT_FILE{"output.txt"};

struct OOIData {
    double x{};
    double y{};
    std::string assignedRobotID{};
};

class LeaderRobot : public BaseRobot {
public:
	LeaderRobot();
	~LeaderRobot() = default;
  
	void run() override;
	void move(double speed) override;
	void rotate(double speed) override;	
	
	void scanLidarData();
	void fileOutput(const std::string& output);
  
private:	
	std::unique_ptr<webots::Motor> frontLeftMotor{};
	std::unique_ptr<webots::Motor> frontRightMotor{};
	std::unique_ptr<webots::Motor> rearLeftMotor{};
	std::unique_ptr<webots::Motor> rearRightMotor{};
	std::unique_ptr<webots::Lidar> lidar{};
	std::vector<OOIData> OOIVec{};
	
	// Helper functions
	std::string formatDoubleToString(double number);
	void leaderPhase0(int& phase);
	void leaderPhase1(int& phase, int& batch);
	void leaderPhase2(int& phase, int& attempts, int batch, bool& moveOn);
	void leaderPhase3(int& phase);
           
};
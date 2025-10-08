#include "LeaderRobot.hpp"

LeaderRobot::LeaderRobot() 
    : frontLeftMotor{ getMotor("front left wheel motor") }
    , frontRightMotor{ getMotor("front right wheel motor") }
    , rearLeftMotor{ getMotor("rear left wheel motor") }
    , rearRightMotor{ getMotor("rear right wheel motor") }
    , lidar{ getLidar("lidar") } {

    frontLeftMotor->setPosition(INFINITY);
    frontRightMotor->setPosition(INFINITY);
    rearLeftMotor->setPosition(INFINITY);
    rearRightMotor->setPosition(INFINITY);
    
    frontLeftMotor->setVelocity(0.0);
    frontRightMotor->setVelocity(0.0);
    rearLeftMotor->setVelocity(0.0);
    rearRightMotor->setVelocity(0.0);
    
    lidar->enable(TIME_STEP);
    lidar->enablePointCloud();
    
    std::ofstream ofst{ CONFIG_FILE, std::ios::app };
    std::string tempStr{};
    if (std::filesystem::is_empty(CONFIG_FILE)) {
        ofst << "keyboardControl=true";
    }
    ofst.close();
    
    updateCurrentPosition();
    setTargetPosition(currentPositionX, currentPositionY);
}

void LeaderRobot::run() {
    
    std::ifstream ifst{ CONFIG_FILE };
    std::string str{};
    bool autoMode{ true };
    while (std::getline(ifst, str)) {
        if (str == "keyboardControl=true") {
            autoMode = false;
        }
    }
    ifst.close();
    
    int phase{ 0 };
    int batch{ 0 };
    int attempts{ 0 };
    bool moveOn{ false };
   
    while (step(TIME_STEP) != -1 && autoMode) {   
        updateCurrentPosition();     
        switch (phase) {
            case 0:
                leaderPhase0(phase);
                break;
            case 1:
                leaderPhase1(phase, batch);
                break;
            case 2:
                leaderPhase2(phase, attempts, batch, moveOn);
                break;
            case 3:
                leaderPhase3(phase);
                break;
            case 4:
                // Tasks ended
                break;
        }
    }
    while (step(TIME_STEP) != -1 && !autoMode) {
        keyboardControl();
    }
}

void LeaderRobot::move(double speed) {
    frontLeftMotor->setVelocity(speed);
    frontRightMotor->setVelocity(speed);
    rearLeftMotor->setVelocity(speed);
    rearRightMotor->setVelocity(speed);
}

void LeaderRobot::rotate(double speed) {
    frontLeftMotor->setVelocity(speed);
    frontRightMotor->setVelocity(-speed);
    rearLeftMotor->setVelocity(speed);
    rearRightMotor->setVelocity(-speed);
}

void LeaderRobot::scanLidarData() {
    const WbLidarPoint* pointCloud = lidar->getPointCloud();
    int pointCloudSize = lidar->getNumberOfPoints();
    int numOfPoints{ 0 };
    for (int i {0}; i < pointCloudSize; ++i) {
        if (sqrt(pow(pointCloud[i].x - pointCloud[i + 1].x, 2) + pow(pointCloud[i].y - pointCloud[i + 1].y, 2)) < 1.0) {
            ++numOfPoints;
        } 
        else if ((pointCloud[i].x != INFINITY) && (pointCloud[i].x != -INFINITY) && (pointCloud[i].y != INFINITY) && (pointCloud[i].y != -INFINITY)) {
            OOIData target{};
            target.x = pointCloud[i - numOfPoints / 2].x + currentPositionX;
            target.y = pointCloud[i - numOfPoints / 2].y + currentPositionY;
            int robotID{ static_cast<int>(OOIVec.size()) + 1 };
            while (robotID > NUM_OF_SCOUTS) {
                robotID -= NUM_OF_SCOUTS;
            }
            target.assignedRobotID = std::to_string(robotID);
            OOIVec.push_back(target);
            fileOutput("OOI discovered at x:" + formatDoubleToString(target.x) + " y:" + formatDoubleToString(target.y));
            numOfPoints = 0;
        }    
    }
}

void LeaderRobot::fileOutput(const std::string& output) {
    std::ofstream ofst{OUTPUT_FILE, std::ios::app};
    ofst << output << '\n';
    ofst.close();
    std::cout << output << '\n';
}

// Helper functions below
std::string LeaderRobot::formatDoubleToString(double number) {
    number = (round(number * 100) / 100);
    std::stringstream sst{};
    sst << std::fixed << std::setprecision(2) << number;   
    return sst.str();
}

void LeaderRobot::leaderPhase0(int& phase) {
    scanLidarData();
    phase = 1;
}

void LeaderRobot::leaderPhase1(int& phase, int& batch) {
    int i{ batch };
    int count{ 0 };
    while (i < batch + NUM_OF_SCOUTS && i < static_cast<int>(OOIVec.size())) {           
        sendMessage(OOIVec.at(i).assignedRobotID, std::to_string(OOIVec.at(i).x), std::to_string(OOIVec.at(i).y));
        fileOutput("Target pose x:" + formatDoubleToString(OOIVec.at(i).x) + " y:" + formatDoubleToString(OOIVec.at(i).y) + " sent to robot " + OOIVec.at(i).assignedRobotID);
        ++i;
        ++count;
    }
    batch += count;
    phase = 2;
}

void LeaderRobot::leaderPhase2(int& phase, int& attempts, int batch, bool& moveOn) {
    std::pair<std::string, std::string> message{ receiveMessage() };    
    if (!message.first.empty() && !message.second.empty()) {
        if (message.first == "infinity") {
            fileOutput("Robot " + message.second + " has identified a red OOI");    
        } else {
            fileOutput("Robot " + message.second + " has identified a green OOI");
            std::stringstream coords{ message.first };
            double x{};
            double y{};
            coords >> x >> y;
            fileOutput("Green OOI has been found, moving to x:" + formatDoubleToString(x) + " y:" + formatDoubleToString(y));
            setTargetPosition(x, y);
            moveOn = true;
        }  
        ++attempts;
        while (batch > NUM_OF_SCOUTS) {
            batch -= NUM_OF_SCOUTS;
        }
        if ((attempts == batch) && moveOn) {
            phase = 3;
        } else if (attempts == batch) {
            attempts = 0;
            phase = 1;
        }
    }
}

void LeaderRobot::leaderPhase3(int& phase) {
    if (moveToTarget(0.8)) {
        fileOutput("Successfully arrived at the green OOI");
        phase = 4;
    }
}
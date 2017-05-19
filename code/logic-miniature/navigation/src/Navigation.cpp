/**
 * Copyright (C) 2016 Chalmers Revere
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


#include <cstdlib>
#include <iostream>

#include <opendavinci/odcore/base/KeyValueConfiguration.h>
#include <opendavinci/odcore/base/Lock.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/strings/StringToolbox.h>

#include <opendavinci/odcore/wrapper/Eigen.h>
#include <opendavinci/odcore/data/TimeStamp.h>

#include <odvdopendlvdata/GeneratedHeaders_ODVDOpenDLVData.h>
#include <odvdminiature/GeneratedHeaders_ODVDMiniature.h>

#include "Navigation.h"
#include "Astar.h"
#include "KalmanFilter.h"

namespace opendlv {
namespace logic {
namespace miniature {

/*
  Constructor.
*/
Navigation::Navigation(const int &argc, char **argv)
    : TimeTriggeredConferenceClientModule(argc, argv, "logic-miniature-navigation")
    , m_mutex()
    , m_outerWalls()
    , m_innerWalls()
    , m_pointsOfInterest()
    , m_analogReadings()
    , m_gpioReadings()
    , m_gpioOutputPins()
    , m_pwmOutputPins()
    , m_pruReading()
    , m_sonarDetectionTime()
    , m_xPositionLPS(0.0)
    , m_yPositionLPS(0.0)
    , m_yawLPS(0.04)
    , m_timeLastLPSSignal()
    , m_prevLeftMotorDutyCycle(0)
    , m_prevRightMotorDutyCycle(0)
    , m_prevLeftWheelDirection(Direction::backward)
    , m_prevRightWheelDirection(Direction::backward)
    , m_PIDController(10000.0, 1000.0, 0.0)
    , m_path() //m_path({{0.0, 0.0}, {25.0, 0.0}, {30.0, -5.0}, {30.0, -10.0}, {40.0, -10.0}, {40.0, -20.0}, {10.0, -20.0}, {5.0, -10.0}, {0.0, 0.0}})
    , m_pathCurrentPointIndex(0)
    , m_followPathDirection(Direction::forward)
    , m_currentState(State::Stop)
    , m_stateTimer(0.0)
    , m_stateTimeout(5.0)
    , m_deltaTime()
{
}

/*
  Destructor.
*/
Navigation::~Navigation() 
{
}

/* 
  This method reads values from the configuration file. Note that there is only
  one global configuration storage loaded by the central odsupercomponent
  module. If the the configuration file is changed, the odsupercompnent module
  needs to be restarted.
*/
void Navigation::setUp()
{
  odcore::base::KeyValueConfiguration kv = getKeyValueConfiguration();
  std::string const gpioPinsString = 
      kv.getValue<std::string>("logic-miniature-navigation.gpio-pins");
  std::vector<std::string> gpioPinsVector = 
      odcore::strings::StringToolbox::split(gpioPinsString, ',');
  for (auto pin : gpioPinsVector) {
    m_gpioOutputPins.push_back(std::stoi(pin)); 
  }

  std::string const pwmPinsString = 
      kv.getValue<std::string>("logic-miniature-navigation.pwm-pins");
  std::vector<std::string> pwmPinsVector = 
      odcore::strings::StringToolbox::split(pwmPinsString, ',');
  for (auto pin : pwmPinsVector) {
    m_pwmOutputPins.push_back(std::stoi(pin));
  }
  
  std::string const outerWallsString = 
      kv.getValue<std::string>("logic-miniature-navigation.outer-walls");
  std::vector<data::environment::Point3> outerWallPoints = ReadPointString(outerWallsString);
  if (outerWallPoints.size() == 4) {
    m_outerWalls.push_back(data::environment::Line(outerWallPoints[0], outerWallPoints[1]));
    m_outerWalls.push_back(data::environment::Line(outerWallPoints[1], outerWallPoints[2]));
    m_outerWalls.push_back(data::environment::Line(outerWallPoints[2], outerWallPoints[3]));
    m_outerWalls.push_back(data::environment::Line(outerWallPoints[3], outerWallPoints[0]));

    std::cout << "Outer walls 1 - " << m_outerWalls[0].toString() <<  std::endl;
    std::cout << "Outer walls 2 - " << m_outerWalls[1].toString() <<  std::endl;
    std::cout << "Outer walls 3 - " << m_outerWalls[2].toString() <<  std::endl;
    std::cout << "Outer walls 4 - " << m_outerWalls[3].toString() <<  std::endl;
  } else {
    std::cout << "Warning: Outer walls format error. (" << outerWallsString << ")" << std::endl;
  }
  
  std::string const innerWallsString = 
      kv.getValue<std::string>("logic-miniature-navigation.inner-walls");
  std::vector<data::environment::Point3> innerWallPoints = ReadPointString(innerWallsString);
  for (uint32_t i = 0; i < innerWallPoints.size(); i += 2) {
    if (i < innerWallPoints.size() - 1) {
      data::environment::Line innerWall(innerWallPoints[i], innerWallPoints[i+1]);
      m_innerWalls.push_back(innerWall);
      std::cout << "Inner wall - " << innerWall.toString() << std::endl;
    }
  }
  
  std::string const pointsOfInterestString = 
      kv.getValue<std::string>("logic-miniature-navigation.points-of-interest");
  m_pointsOfInterest = ReadPointString(pointsOfInterestString);
  for (uint32_t i = 0; i < m_pointsOfInterest.size(); i++) {
    std::cout << "Point of interest " << i << ": " << m_pointsOfInterest[i].toString() << std::endl;
  }

  m_deltaTime = 1 / getFrequency();
}

/*
  This method is run automatically when the system is shutting down (before the
  destructor). It is typically used to close log files and de-allocate 
  dynamically allocated memory.
*/
void Navigation::tearDown()
{
}

/* 
  The while loop in this method runs at a predefined (in configuration) 
  frequency.
*/
odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Navigation::body()
{
  Direction turnDirection = Direction::left;
  bool turnDirectionSet = false;

  bool pathFound = false;
  std::vector<std::vector<double> > bestPathCoord;

  // Setup extended Kalman filter
  Eigen::Vector3d kalmanInitState(0.0, 0.0, 0.0);
  Eigen::Matrix3d kalmanQ = Eigen::MatrixXd::Identity(3, 3);
  Eigen::Matrix3d kalmanR = 0.001*Eigen::MatrixXd::Identity(3, 3);
  Eigen::Matrix3d kalmanP_0 = Eigen::MatrixXd::Identity(3, 3);
  KalmanFilter kalmanFilter(kalmanInitState, kalmanQ, kalmanR, kalmanP_0);
  uint32_t testTick = 0;
  
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == 
      odcore::data::dmcp::ModuleStateMessage::RUNNING) {

    // The mutex is required since 'body' and 'nextContainer' competes by
    // reading and writing to the class global maps, see also 'nextContainer'.
    odcore::base::Lock l(m_mutex);

    // Defalt duty cycles
    uint32_t leftMotorDutyCycle = 25000;
    uint32_t rightMotorDutyCycle = 25000;	
    // Default directions
    Direction leftWheelDirection = Direction::forward;
    Direction rightWheelDirection = Direction::forward;


    // Get sensor distances
    double sonarDistance = m_pruReading; // (centimeters)
    if(sonarDistance < 0) sonarDistance = 4000;
    std::cout << "Sonar distance (cm): " << sonarDistance << std::endl;

    bool irRightDetection = false;
    bool irLeftDetection = false;
    if(m_analogReadings[1] < 800) irRightDetection = true;
    if(m_analogReadings[5] < 800) irLeftDetection = true;

    std::cout << "Right IR sensor voltage: " << m_analogReadings[1] << std::endl;
    std::cout << "Left IR sensor voltage: " << m_analogReadings[5] << std::endl;
    
    // If there is a new LPS reading available, do the update step in the Kalman filter
    odcore::data::TimeStamp now;
    double timeSinceLastLPSSignal = static_cast<double>(now.toMicroseconds() - m_timeLastLPSSignal.toMicroseconds())/1000000.0;
    if (timeSinceLastLPSSignal < 1.0*m_deltaTime && testTick % 10 == 0) { // There is a new LPS reading available
      kalmanFilter.doUpdateStep(m_xPositionLPS, m_yPositionLPS, m_yawLPS);
    }
    testTick++;
    Eigen::Vector3d kalmanEstimate = kalmanFilter.getStateEstimate(); 
    std::cout << "Kalman estimates: " << kalmanEstimate(0) << ", " << kalmanEstimate(1) << ", " << kalmanEstimate(2) << std::endl;
    std::cout << "Actual coordinates: " << m_xPositionLPS << ", " << m_yPositionLPS << ", " << m_yawLPS << std::endl;

    // State follow path
    if(m_currentState == State::PathFollow)
    {
      const uint32_t baseMotorDutyCycleNs = 40000;

      std::vector<double> targetPosition = pathUpdateCurrentTarget(m_xPositionLPS, m_yPositionLPS, bestPathCoord);

      double headingX = cos(m_yawLPS);
      double headingY = sin(m_yawLPS);
      double diffX = targetPosition[0] - m_xPositionLPS;
      double diffY = targetPosition[1] - m_yPositionLPS;

      // Calculate angle between vector pointing to target and vector pointing in the direction of the robot
      double angleToTarget = acos((headingX*diffX + headingY*diffY) /
          (sqrt(headingX*headingX + headingY*headingY) * sqrt(diffX*diffX + diffY*diffY)));

      // Check cross product to see which way to turn
      bool turnLeft = true;
      if (headingX*diffY - headingY*diffX < 0)
        turnLeft = false;

      double angleError;
      if (turnLeft) {
        angleError = -angleToTarget;
      } else {
        angleError = angleToTarget;
      }

      // Get PID output
      uint32_t motorsDutyCycleDifference = m_PIDController.update(angleError, m_deltaTime);
      
      // Set motor pwms
      leftMotorDutyCycle = baseMotorDutyCycleNs + motorsDutyCycleDifference;
      rightMotorDutyCycle = baseMotorDutyCycleNs - motorsDutyCycleDifference;
      leftMotorDutyCycle = (leftMotorDutyCycle > 50000) ? 50000 : leftMotorDutyCycle;
      rightMotorDutyCycle = (rightMotorDutyCycle > 50000) ? 50000 : rightMotorDutyCycle;
      leftMotorDutyCycle = (leftMotorDutyCycle < 25000) ? 25000 : leftMotorDutyCycle;
      rightMotorDutyCycle = (rightMotorDutyCycle < 25000) ? 25000 : rightMotorDutyCycle;
      leftWheelDirection = Direction::forward;
      rightWheelDirection = Direction::forward;
     
      if(sonarDistance < 30 || irLeftDetection || irRightDetection)
      {
        m_currentState = State::Avoid;
        m_stateTimer = 0;
        m_stateTimeout = 1.5;
      }
    }

    if(m_currentState == State::Cruise)
    {
      std::cout << "Moving forward..." << std::endl;
      leftMotorDutyCycle = 41000;
      rightMotorDutyCycle = 41000;
      leftWheelDirection = Direction::forward;
      rightWheelDirection = Direction::forward;

      if(sonarDistance < 40 || irLeftDetection || irRightDetection)
      {
        m_currentState = State::Avoid;
        m_stateTimer = 0;
        m_stateTimeout = 0.5;
      }
    }
    // State avoid
    if(m_currentState == State::Avoid)
    {
      if(!turnDirectionSet)
      {
        if(irLeftDetection)
          turnDirection = Direction::right;
        else
          turnDirection = Direction::left;

        m_stateTimeout = 0.1;
        turnDirectionSet = true;
      }

      if(turnDirection == Direction::right)
      {
        std::cout << "Turning right..." << std::endl;
        leftMotorDutyCycle = 40000;
        rightMotorDutyCycle = 40000;
        leftWheelDirection = Direction::forward;
        rightWheelDirection = Direction::backward;
      }
      else
      {
        std::cout << "Turning left..." << std::endl;
        leftMotorDutyCycle = 40000;
        rightMotorDutyCycle = 40000;
        leftWheelDirection = Direction::backward;
        rightWheelDirection = Direction::forward;
      }

      if(m_stateTimer > m_stateTimeout)
      {
        if(!(sonarDistance < 40))
        {
          m_currentState = State::PathFollow;
          m_stateTimer = 0;
          turnDirectionSet = false;
        }
      }
    }

    // State Stop
    if(m_currentState == State::Stop)
    {
      leftMotorDutyCycle = 0;
      rightMotorDutyCycle = 0;

      if(m_stateTimer > m_stateTimeout)
      {
        m_currentState = State::PathFollow;
        m_stateTimer = 0;
      }
    }
	
	if (!pathFound) {
		std::vector<std::vector<double> > outerWalls = {{50.84, -23.93}, {-9.48, -24.49}, {-9.70, 5.50}, {50.54, 5.65}};

		std::vector<std::vector<double> > innerWalls = {{40.02, 5.86},{39.76, -6.63}, {2.88,5.33}, {2.92,0.57}, {-9.71,-4.17}, {-7.45,-4.11}, {33.06,-24.10}, {33.08,-19.09}, {33.08,-19.09}, {35.50,-19.10}, {20.74,-17.83}, {14.24,-7.36}, {14.24,-7.36}, {18.59,-4.93}, {18.59,-4.93}, {26.08,-6.93}, {26.08,-6.93}, {20.74,-17.83}};

		Astar aStar;
		aStar.setMapSize(60,30); // Set mapSize
		double xStart = -5.0;
		double yStart = 0.0;
		double xTarget = 45.0;
		double yTarget = -15.0;
		std::vector<int16_t> startIndex = aStar.coordToIndex(xStart,yStart,outerWalls);
		std::vector<int16_t> targetIndex = aStar.coordToIndex(xTarget,yTarget,outerWalls);
		cout << "Start Index: " << startIndex[0] << " " << startIndex[1] << endl;
		cout << "Target Index: " << targetIndex[0] << " " << targetIndex[1] << endl;
		aStar.startNode.set_position(startIndex[0],startIndex[1]); // Set start
		aStar.targetNode.set_position(targetIndex[0],targetIndex[1]); // Set target

		std::vector<std::vector<int16_t> > map = aStar.createMap(outerWalls, innerWalls);
		std::vector<std::vector<int16_t> > bestPath = aStar.getPath(map);

		//aStar.printMap(map, bestPath);
		bestPathCoord = aStar.indexPathToCoordinate(bestPath, outerWalls);
		pathFound = true;
		/*for (uint16_t i=0; i<bestPathCoord.size(); i++) {
			cout << "Idx: " << bestPath[i][0] << " " << bestPath[i][1] << endl;
			cout << "Coord: " << bestPathCoord[i][0] << " " << bestPathCoord[i][1] << endl;
		}*/
	}

    // Check if robot slave is initialized
    double initialSensorReading = 0.0;
    if(!(m_pruReading > initialSensorReading) && m_pruReading >= 0.0)
    {
      leftMotorDutyCycle = 0;
      rightMotorDutyCycle = 0;
      leftWheelDirection = Direction::backward;
      rightWheelDirection = Direction::backward;

      m_stateTimer = 0;
    }

    sendMotorCommands(leftMotorDutyCycle, rightMotorDutyCycle);
    sendGPIOCommands(leftWheelDirection, rightWheelDirection);

    // Do Kalman prediction step
    double leftWheelSpeed = 0.6 * 100 * (leftMotorDutyCycle-25000) / 25000;
    double rightWheelSpeed = 0.6 * 100 * (rightMotorDutyCycle-25000) / 25000;
    leftWheelSpeed = (leftWheelDirection == Direction::forward) ? leftWheelSpeed : -leftWheelSpeed;
    rightWheelSpeed = (rightWheelDirection == Direction::forward) ? rightWheelSpeed : -rightWheelSpeed;
    kalmanFilter.doPredictionStep(leftWheelSpeed, rightWheelSpeed, m_deltaTime);
        
    m_stateTimer += m_deltaTime;
    std::cout << "m_deltaTime = " << m_deltaTime << std::endl;

    std::cout << "\n" << std::endl;
  }
  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

void Navigation::sendMotorCommands(uint32_t leftMotorDutyCycle, uint32_t rightMotorDutyCycle)
{
    //std::cout << "Pwm left motor: " << leftMotorDutyCycle << std::endl;
    //std::cout << "Pwm right motor: " << rightMotorDutyCycle << std::endl;

  if(leftMotorDutyCycle != m_prevLeftMotorDutyCycle)
  {
    opendlv::proxy::PwmRequest request1(0, leftMotorDutyCycle);
    odcore::data::Container c1(request1);
    uint32_t stamp1 = 1;
    c1.setSenderStamp(stamp1);
    getConference().send(c1);

    m_prevLeftMotorDutyCycle = leftMotorDutyCycle;

    std::cout << "SEND  " << leftMotorDutyCycle << " to left motor" << std::endl;
  }

  if(rightMotorDutyCycle != m_prevRightMotorDutyCycle)
  {
    opendlv::proxy::PwmRequest request2(0, rightMotorDutyCycle);
    odcore::data::Container c2(request2);
    uint32_t stamp2 = 2;
    c2.setSenderStamp(stamp2);
    getConference().send(c2);

    m_prevRightMotorDutyCycle = rightMotorDutyCycle;

    std::cout << "SEND pwm: " << rightMotorDutyCycle << " to right motor" << std::endl;
  }
}

void Navigation::sendGPIOCommands(Direction leftWheelDirection, Direction rightWheelDirection)
{
  opendlv::proxy::ToggleRequest::ToggleState stateOn = opendlv::proxy::ToggleRequest::On;
  opendlv::proxy::ToggleRequest::ToggleState stateOff = opendlv::proxy::ToggleRequest::Off;


  if(leftWheelDirection != m_prevLeftWheelDirection)
  {
    m_prevLeftWheelDirection = leftWheelDirection;
    if(leftWheelDirection == Direction::forward)
    {
      opendlv::proxy::ToggleRequest request30(30, stateOn);
      odcore::data::Container c30(request30);
      getConference().send(c30);
      std::cout << "SET gpio 30 ON (left wheel forward)" << std::endl;

      opendlv::proxy::ToggleRequest request31(31, stateOff);
      odcore::data::Container c31(request31);
      getConference().send(c31);
      std::cout << "SET gpio 31 OFF (left wheel forward)" << std::endl;
    }
    else
    {
      opendlv::proxy::ToggleRequest request30(30, stateOff);
      odcore::data::Container c30(request30);
      getConference().send(c30);
      std::cout << "SET gpio 30 OFF (left wheel backward)" << std::endl;

      opendlv::proxy::ToggleRequest request31(31, stateOn);
      odcore::data::Container c31(request31);
      getConference().send(c31);
      std::cout << "SET gpio 31 ON (left wheel backward)" << std::endl;
    }
  }

  if(rightWheelDirection != m_prevRightWheelDirection)
  {
    m_prevRightWheelDirection = rightWheelDirection;
    if(rightWheelDirection == Direction::forward)
    {
      opendlv::proxy::ToggleRequest request51(51, stateOn);
      odcore::data::Container c51(request51);
      getConference().send(c51);
      std::cout << "SET gpio 51 ON (right wheel forward)" << std::endl;

      opendlv::proxy::ToggleRequest request60(60, stateOff);
      odcore::data::Container c60(request60);
      getConference().send(c60);
      std::cout << "SET gpio 60 OFF (right wheel forward)" << std::endl;
    }
    else
    {
      opendlv::proxy::ToggleRequest request51(51, stateOff);
      odcore::data::Container c51(request51);
      getConference().send(c51);
      std::cout << "SET gpio 51 OFF (right wheel backward)" << std::endl;

      opendlv::proxy::ToggleRequest request60(60, stateOn);
      odcore::data::Container c60(request60);
      getConference().send(c60);
      std::cout << "SET gpio 60 ON (right wheel backward)" << std::endl;
    }
  }
}

/* 
  This method receives messages from all other modules (in the same conference 
  id, cid). Here, the messages AnalogReading and ToggleReading is received
  from the modules interfacing to the hardware.
*/
void Navigation::nextContainer(odcore::data::Container &a_c)
{
  odcore::base::Lock l(m_mutex);

  int32_t dataType = a_c.getDataType();
  if (dataType == opendlv::proxy::AnalogReading::ID()) {
    opendlv::proxy::AnalogReading reading = 
        a_c.getData<opendlv::proxy::AnalogReading>();

    uint16_t pin = reading.getPin();
    float voltage = reading.getVoltage();

    m_analogReadings[pin] = voltage; // Save the input to the class global map.

    //std::cout << "[" << getName() << "] Received an AnalogReading: " 
    //    << reading.toString() << "." << std::endl;

  } else if (dataType == opendlv::proxy::ToggleReading::ID()) {
    opendlv::proxy::ToggleReading reading = 
        a_c.getData<opendlv::proxy::ToggleReading>();

    uint16_t pin = reading.getPin();
    bool state;
    if (reading.getState() == opendlv::proxy::ToggleReading::On) {
      state = true;
    } else {
      state = false;
    }

    m_gpioReadings[pin] = state; // Save the state to the class global map.

    //std::cout << "[" << getName() << "] Received a ToggleReading: "
    //    << reading.toString() << "." << std::endl;
  } else if (dataType == opendlv::proxy::ProximityReading::ID()) {
    opendlv::proxy::ProximityReading reading =
      a_c.getData<opendlv::proxy::ProximityReading>();

    double distance = reading.getProximity();

    m_pruReading = distance;
  } else if (dataType == opendlv::model::State::ID()) {
    opendlv::model::State state = a_c.getData<opendlv::model::State>();
    m_timeLastLPSSignal = a_c.getReceivedTimeStamp();

    m_xPositionLPS = static_cast<double>(state.getPosition().getX());
    m_yPositionLPS = static_cast<double>(state.getPosition().getY());
    m_yawLPS = static_cast<double>(state.getAngularDisplacement().getZ());
  
    //std::cout << "[" << getName() << "] Received a LPS-reading: "
    //    << state.toString() << "." << std::endl;
  }
}

std::vector<data::environment::Point3> Navigation::ReadPointString(std::string const &a_pointsString) const
{
  std::vector<data::environment::Point3> points;
  std::vector<std::string> pointStringVector = 
      odcore::strings::StringToolbox::split(a_pointsString, ';');
  for (auto pointString : pointStringVector) {
    std::vector<std::string> coordinateVector = 
        odcore::strings::StringToolbox::split(pointString, ',');
    if (coordinateVector.size() == 2) {
      double x = std::stod(coordinateVector[0]);
      double y = std::stod(coordinateVector[1]);
      double z = 0.0;
      points.push_back(data::environment::Point3(x, y, z));
    }
  }
  return points;
}

std::vector<double> Navigation::pathUpdateCurrentTarget(double currentX, double currentY, std::vector<std::vector<double>> path)
{
  const double distanceToSwitchTargetPoint = 2.0;

  double pointX = path[m_pathCurrentPointIndex][0];
  double pointY = path[m_pathCurrentPointIndex][1];
  double distance = sqrt((pointX-currentX)*(pointX-currentX) + (pointY-currentY)*(pointY-currentY));

  if (distance < distanceToSwitchTargetPoint) {
    if (m_followPathDirection == Direction::forward){
      // Following path in forward direction
      m_pathCurrentPointIndex++;
      if (m_pathCurrentPointIndex >= path.size()) {
        m_pathCurrentPointIndex = path.size() - 2;
        m_followPathDirection = Direction::backward;
      }
    } else {
      // Following path in backward direction
      if (m_pathCurrentPointIndex == 0) {
        m_pathCurrentPointIndex = 1;
        m_followPathDirection = Direction::forward;
      } else {
        m_pathCurrentPointIndex--;
      }
    }
  }

  return path[m_pathCurrentPointIndex];
}

}
}
}

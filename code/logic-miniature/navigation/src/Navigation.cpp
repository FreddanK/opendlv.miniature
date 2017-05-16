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

#include <odvdopendlvdata/GeneratedHeaders_ODVDOpenDLVData.h>
#include <odvdminiature/GeneratedHeaders_ODVDMiniature.h>

#include "Navigation.h"

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
    , m_prevLeftMotorDutyCycle(0)
    , m_prevRightMotorDutyCycle(0)
    , m_prevLeftWheelDirection(Direction::backward)
    , m_prevRightWheelDirection(Direction::backward)
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
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == 
      odcore::data::dmcp::ModuleStateMessage::RUNNING) {

    // The mutex is required since 'body' and 'nextContainer' competes by
    // reading and writing to the class global maps, see also 'nextContainer'.
    odcore::base::Lock l(m_mutex);
    //wait for sensors to read correct values.
    //set PWM signals.


    // Defalt duty cycles
    uint32_t leftMotorDutyCycle = 45000;
    uint32_t rightMotorDutyCycle = 45000;	
    // Default directions
    Direction leftWheelDirection = Direction::forward;
    Direction rightWheelDirection = Direction::forward;

    // Get sensor distances
    double sonarDistance = m_pruReading; // (centimeters)
    if(sonarDistance < 0) sonarDistance = 4000;

    bool irLeftDetection = false;
    bool irRightDetection = false;
    if(m_analogReadings[1] < (float)1.7) irLeftDetection = true;
    if(m_analogReadings[0] < (float)1.7) irRightDetection = true;


    if(m_currentState == State::Cruise)
    {
      std::cout << "Moving forward..." << std::endl;
      leftMotorDutyCycle = 41000;
      rightMotorDutyCycle = 41000;
      leftWheelDirection = Direction::forward;
      rightWheelDirection = Direction::forward;

      if(sonarDistance < 30 || irLeftDetection || irRightDetection)
      {
        m_currentState = State::Avoid;
        m_stateTimer = 0;
        m_stateTimeout = 1.5;
      }
    }

    // State avoid
    if(m_currentState == State::Avoid)
    {
      std::cout << "Turning..." << std::endl;
      leftMotorDutyCycle = 40000;
      rightMotorDutyCycle = 40000;
      leftWheelDirection = Direction::forward;
      rightWheelDirection = Direction::backward;

      if(m_stateTimer > m_stateTimeout)
      {
        m_currentState = State::Cruise;
        m_stateTimer = 0;
      }
    }

    // State Stop
    if(m_currentState == State::Stop)
    {
      leftMotorDutyCycle = 0;
      rightMotorDutyCycle = 0;

      if(m_stateTimer > m_stateTimeout)
      {
        m_currentState = State::Cruise;
        m_stateTimer = 0;
      }
    }

    // Check if robot slave is initialized
    double initialSensorReading = 0.0;
    if(!(m_pruReading > initialSensorReading) && m_pruReading >= 0.0)
    {
      leftMotorDutyCycle = 0;
      rightMotorDutyCycle = 0;
      m_stateTimer = 0;
    }

    sendMotorCommands(leftMotorDutyCycle, rightMotorDutyCycle);
    sendGPIOCommands(leftWheelDirection, rightWheelDirection);

    std::cout << "IR sensor 1 voltage: " << m_analogReadings[0] << std::endl;
    std::cout << "IR sensor 2 voltage: " << m_analogReadings[1] << std::endl;

    m_stateTimer += m_deltaTime;

  }
  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

void Navigation::sendMotorCommands(uint32_t leftMotorDutyCycle, uint32_t rightMotorDutyCycle)
{
    std::cout << "Pwm left motor: " << leftMotorDutyCycle << std::endl;
    std::cout << "Pwm right motor: " << rightMotorDutyCycle << std::endl;

  if(leftMotorDutyCycle != m_prevLeftMotorDutyCycle)
  {
    opendlv::proxy::PwmRequest request1(0, leftMotorDutyCycle);
    odcore::data::Container c1(request1);
    uint32_t stamp1 = 1;
    c1.setSenderStamp(stamp1);
    getConference().send(c1);

    m_prevLeftMotorDutyCycle = leftMotorDutyCycle;
    std::cout << "SEND left motor" << std::endl;
  }

  if(rightMotorDutyCycle != m_prevRightMotorDutyCycle)
  {
    opendlv::proxy::PwmRequest request2(0, rightMotorDutyCycle);
    odcore::data::Container c2(request2);
    uint32_t stamp2 = 2;
    c2.setSenderStamp(stamp2);
    getConference().send(c2);

    m_prevRightMotorDutyCycle = rightMotorDutyCycle;
    std::cout << "SEND right motor" << std::endl;
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

      opendlv::proxy::ToggleRequest request31(31, stateOff);
      odcore::data::Container c31(request31);
      getConference().send(c31);
    }
    else
    {
      opendlv::proxy::ToggleRequest request30(30, stateOff);
      odcore::data::Container c30(request30);
      getConference().send(c30);

      opendlv::proxy::ToggleRequest request31(31, stateOn);
      odcore::data::Container c31(request31);
      getConference().send(c31);
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
      opendlv::proxy::ToggleRequest request60(60, stateOff);
      odcore::data::Container c60(request60);
      getConference().send(c60);
    }
    else
    {
      opendlv::proxy::ToggleRequest request51(51, stateOff);
      odcore::data::Container c51(request51);
      getConference().send(c51);
      opendlv::proxy::ToggleRequest request60(60, stateOn);
      odcore::data::Container c60(request60);
      getConference().send(c60);
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

}
}
}

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


    odcore::data::TimeStamp now;

    // Check for collision
    if (m_pruReading < 30.0) {
      m_sonarDetectionTime = now;
    }

    double timeSinceLastSonarDetection = static_cast<double>(now.toMicroseconds() - m_sonarDetectionTime.toMicroseconds()) / 1000000.0;

    std::cout << "Sonar sensor reading: " << m_pruReading << ", with time stamp: " << timeSinceLastSonarDetection << std::endl;

    if (timeSinceLastSonarDetection < 1.5) {	
      std::cout << "Backing..." << std::endl;
      leftMotorDutyCycle = 40000;
      rightMotorDutyCycle = 40000;
      leftWheelDirection = Direction::backward;
      rightWheelDirection = Direction::backward;

    } else if (timeSinceLastSonarDetection < 3.0) {
      std::cout << "Turning..." << std::endl;
      leftMotorDutyCycle = 40000;
      rightMotorDutyCycle = 40000;
      leftWheelDirection = Direction::backward;
      rightWheelDirection = Direction::forward;

    } else {
      std::cout << "Moving forward..." << std::endl;
      leftMotorDutyCycle = 41000;
      rightMotorDutyCycle = 41000;
      leftWheelDirection = Direction::forward;
      rightWheelDirection = Direction::forward;
    }

    // Check if robot slave is initialized
    double initialSensorReading = 0.0;
    if(!(m_pruReading > initialSensorReading))
    {
      leftMotorDutyCycle = 0;
      rightMotorDutyCycle = 0;
    }

    sendMotorCommands(leftMotorDutyCycle, rightMotorDutyCycle);
    sendGPIOCommands(leftWheelDirection, rightWheelDirection);

    std::cout << "IR sensor voltage: " << m_analogReadings[0] << std::endl;

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

}
}
}

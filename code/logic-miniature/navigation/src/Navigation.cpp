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

    //// Example below.
    
    // Constant definintions
    float const MAX_VOLTAGE = 1.8;
    float const IR_MAX_DISTANCE = 3;
    float const SONIC_MAX_DISTANCE = 20;

    uint32_t const PWM_NEUTRAL = 1500000;
    uint32_t const PWM_MAX = 2000000;
    uint32_t const PWM_MIN = 1000000;
    uint32_t const PWM_STEP = 100000;


    uint32_t leftOutputPWM = PWM_NEUTRAL;
    uint32_t rightOutputPWM = PWM_NEUTRAL;

    // Print some data collected from the 'nextContainer' method below.
    //std::cout << "Reading from analog pin 0: " << voltageReadingPin0 << std::endl;
    
    float voltageMiddleIR = m_analogReadings[0];
    float voltageRightSonic = m_analogReadings[1];
    float voltageLeftSonic = m_analogReadings[2];

    float distanceMiddleIR = (voltageMiddleIR/MAX_VOLTAGE)*IR_MAX_DISTANCE;
    float distanceRightSonic = (voltageRightSonic/MAX_VOLTAGE)*SONIC_MAX_DISTANCE;
    float distanceLeftSonic = (voltageLeftSonic/MAX_VOLTAGE)*SONIC_MAX_DISTANCE;

    std::cout << "Distance middle infrared: " << distanceMiddleIR << std::endl;
    std::cout << "Distance right sonic: " << distanceRightSonic << std::endl;
    std::cout << "Distance left sonic: " << distanceLeftSonic << std::endl;

    float d = 2.5;
    if(distanceMiddleIR > d)
    {
      if(distanceRightSonic > 10 && distanceLeftSonic > 10) 
      {
        leftOutputPWM = PWM_NEUTRAL + PWM_STEP;
        rightOutputPWM = PWM_NEUTRAL + PWM_STEP;
      }
      else if(abs(distanceRightSonic - distanceLeftSonic) < 1)  
      {
        leftOutputPWM = PWM_NEUTRAL + PWM_STEP;
        rightOutputPWM = PWM_NEUTRAL + PWM_STEP;
      }
      else if(distanceRightSonic > distanceLeftSonic)
      {
        int diff = distanceRightSonic - distanceLeftSonic;
        leftOutputPWM = PWM_NEUTRAL + diff*PWM_STEP;
        rightOutputPWM = PWM_NEUTRAL + PWM_STEP;
      }
      else if(distanceRightSonic < distanceLeftSonic)
      {
        int diff = distanceLeftSonic - distanceRightSonic;
        leftOutputPWM = PWM_NEUTRAL + PWM_STEP;
        rightOutputPWM = PWM_NEUTRAL + diff*PWM_STEP;
      }
    }
    else
    {
      if(abs(distanceRightSonic - distanceLeftSonic) > 1 && distanceRightSonic > distanceLeftSonic)
      {
        leftOutputPWM = PWM_NEUTRAL + PWM_STEP;
        rightOutputPWM = PWM_NEUTRAL - PWM_STEP;
      }
      else
      {
        leftOutputPWM = PWM_NEUTRAL - PWM_STEP;
        rightOutputPWM = PWM_NEUTRAL + PWM_STEP;
      }
    }


    // Loop through all General Purpose IO (GPIO) pins and randomize their 
    // state. The state is then sent as a message to the module interfacing
    // to the actual hardware.
    //for (auto pin : m_gpioOutputPins) {
    //  bool value = static_cast<bool>(std::rand() % 2);

    //  opendlv::proxy::ToggleRequest::ToggleState state;
    //  if (value) {
    //    state = opendlv::proxy::ToggleRequest::On;
    //  } else {
    //    state = opendlv::proxy::ToggleRequest::Off;
    //  }

    //  opendlv::proxy::ToggleRequest request(pin, state);
    //  
    //  odcore::data::Container c(request);
    //  getConference().send(c);
    //  
    //  std::cout << "[" << getName() << "] Sending ToggleRequest: " 
    //      << request.toString() << std::endl;
    //}

    if(leftOutputPWM > PWM_MAX) leftOutputPWM = PWM_MAX;
    else if(leftOutputPWM < PWM_MIN) leftOutputPWM = PWM_MIN;

    if(rightOutputPWM > PWM_MAX) rightOutputPWM = PWM_MAX;
    else if(rightOutputPWM < PWM_MIN) rightOutputPWM = PWM_MIN;

    uint16_t leftWheelPin = 0;
    uint16_t rightWheelPin = 1;

    opendlv::proxy::PwmRequest requestLeft(leftWheelPin,leftOutputPWM);
    opendlv::proxy::PwmRequest requestRight(rightWheelPin,rightOutputPWM);

    odcore::data::Container cL(requestLeft);
    odcore::data::Container cR(requestRight);

    getConference().send(cL);
    getConference().send(cR);

    // Loop through all Pulse Width Modulation (PWM) pins and randomize their 
    // value. The value is then sent as a message to the module interfacing
    // to the actual hardware.
    //for (auto pin : m_pwmOutputPins) {
    //  int32_t rand = (std::rand() % 11) - 5 ;
    //  uint32_t value = 1500000 + rand * 100000;
    //  
    //  opendlv::proxy::PwmRequest request(pin, value);
    //  
    //  odcore::data::Container c(request);
    //  getConference().send(c);
    //  
    //  std::cout << "[" << getName() << "] Sending PwmRequest: " 
    //      << request.toString() << std::endl;
    //}

    ///// Example above.

    ///// TODO: Add proper behaviours.
    //std::cout << "TODO: Add proper behaviour." << std::endl;
  }
  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
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

    std::cout << "[" << getName() << "] Received an AnalogReading: " 
        << reading.toString() << "." << std::endl;

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

    std::cout << "[" << getName() << "] Received a ToggleReading: "
        << reading.toString() << "." << std::endl;
  }
}

}
}
}

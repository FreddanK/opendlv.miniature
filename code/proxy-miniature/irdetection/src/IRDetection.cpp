/**
 * proxy-miniature-irdetection - Interface to irdetection.
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

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <opendavinci/odcore/base/KeyValueConfiguration.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/strings/StringToolbox.h>

#include <odvdminiature/GeneratedHeaders_ODVDMiniature.h>

#include "IRDetection.h"

namespace opendlv {
namespace proxy {
namespace miniature {

IRDetection::IRDetection(const int &argc, char **argv)
    : TimeTriggeredConferenceClientModule(argc, argv, "proxy-miniature-irdetection")
    , m_conversionConst()
    , m_debug()
    , m_initialised()
    , m_path()
    , m_pins()
    , m_analogPins()
    , m_periodsNs()
    , m_dutyCyclesNs()
    , m_analogReadings()
    , m_numberOfReadings()
{
}

IRDetection::~IRDetection() 
{
}

void IRDetection::setUp()
{
  // Set up analog readings
  odcore::base::KeyValueConfiguration kv = getKeyValueConfiguration();

  m_numberOfReadings = 
      kv.getValue<uint16_t>("proxy-miniature-irdetection.numberOfReadings");

  m_conversionConst = 
      kv.getValue<float>("proxy-miniature-irdetection.conversion-constant");
  std::string analogPinsString = 
      kv.getValue<std::string>("proxy-miniature-irdetection.analogPins");
  std::vector<std::string> pinsVecString = 
      odcore::strings::StringToolbox::split(analogPinsString, ',');
  for(std::string const& str : pinsVecString) {
    m_analogPins.push_back(std::stoi(str));
  }

  // Set up pwm
  m_debug = (kv.getValue<int32_t>("proxy-miniature-irdetection.debug") == 1);

  m_path = kv.getValue<std::string>("proxy-miniature-irdetection.systemPath");

  std::string const pinsString = 
      kv.getValue<std::string>("proxy-miniature-irdetection.pins");
  std::vector<std::string> pinsVector = 
      odcore::strings::StringToolbox::split(pinsString, ',');

  std::string const periodNsString = 
      kv.getValue<std::string>("proxy-miniature-irdetection.periodsNs");
  std::vector<std::string> periodNsStringVector = 
      odcore::strings::StringToolbox::split(periodNsString, ',');
  
  std::string const dutyCycleNsString =
      kv.getValue<std::string>("proxy-miniature-irdetection.dutyCyclesNs");
  std::vector<std::string> dutyCycleNsStringVector =
      odcore::strings::StringToolbox::split(dutyCycleNsString, ',');

  if (pinsVector.size() == periodNsStringVector.size() 
      && pinsVector.size() == dutyCycleNsStringVector.size()) {
    for (uint32_t i = 0; i < pinsVector.size(); i++) {
      uint16_t pin = std::stoi(pinsVector.at(i));
      int32_t periodNs = std::stoi(periodNsStringVector.at(i));
      int32_t dutyCycleNs = std::stoi(dutyCycleNsStringVector.at(i));
      m_pins.push_back(pin);
      m_periodsNs.push_back(periodNs);
      m_dutyCyclesNs.push_back(dutyCycleNs);
    }
    if (m_debug) {
      std::cout << "[" << getName() << "] " << "Initialised pins:";
      for (uint32_t i = 0; i < pinsVector.size(); i++) {
        std:: cout << "|Pin " << m_pins.at(i) << " Period " << m_periodsNs.at(i) 
            << " Duty cycle" << m_dutyCyclesNs.at(i);
      }
      std::cout << "." << std::endl;
    }
  } else {
    cerr << "[" << getName() 
        << "] Number of pins do not equals to number of periods or duty cycles." 
        << std::endl;
  }

  OpenPwm();

  m_initialised = true;
}

void IRDetection::tearDown()
{
//  ClosePwm();
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode IRDetection::body()
{
  uint16_t count = 0;

  while (getModuleStateAndWaitForRemainingTimeInTimeslice() ==
      odcore::data::dmcp::ModuleStateMessage::RUNNING) {
    if(m_initialised) 
    {
      if(count == 0) 
      {
        // Turn on pwm pins
        for (uint32_t i = 0; i < m_pins.size(); i++) 
        {
          std:: cout << "Set Pwm Pin " << m_pins.at(i)
              << " Duty cycle: " << m_dutyCyclesNs.at(i);
          uint16_t pin = m_pins.at(i); 
          uint32_t dutyCycleNs = m_dutyCyclesNs.at(i);
          SetDutyCycleNs(pin, dutyCycleNs);
          
        }
      }
      else if(count == 1) 
      {
        // Turn off pwm pins
        for (uint32_t i = 0; i < m_pins.size(); i++) 
        {
          std:: cout << "Set Pwm Pin " << m_pins.at(i)
              << " Duty cycle: " << 0;
          uint16_t pin = m_pins.at(i); 
          uint32_t dutyCycleNs = 0;
          SetDutyCycleNs(pin, dutyCycleNs);
        }
      }

      // Read pins
      ReadAnalogPins();

      // Increase count
      count++;

      // Send readings to conference and reset
      if (count >= m_numberOfReadings-1)
      {
        count = 0;
        SendAnalogReadings();
        m_analogReadings = getReadings();
      }
    }
  }
  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

void IRDetection::ReadAnalogPins()
{
  // Get readings from analog pins
  std::vector<std::pair<uint16_t, float>> reading = getReadings();

  
  for (uint16_t i=0; i<m_analogReadings.size();i++) 
  {
    if(reading[i].second < m_analogReadings[i].second)
    {
      m_analogReadings[i].second = reading[i].second;
    }
  }
}

void IRDetection::SendAnalogReadings()
{
  // Send readings to conference
  for (std::pair<uint16_t, float> const& pair : m_analogReadings) 
  {
    opendlv::proxy::AnalogReading message(pair.first, pair.second);
    odcore::data::Container c(message);
    getConference().send(c);
  }
  if(m_debug) 
  {
    std::cout << "[" << getName() << "] ";
    for (std::pair<uint16_t, float> const& pair : m_analogReadings) 
    {
      std::cout << "Pin " << pair.first << ": " << pair.second << " ";
    }
    std::cout << std::endl;
  }
}

std::vector<std::pair<uint16_t, float>> IRDetection::getReadings() {
  std::vector<std::pair<uint16_t, float>> reading;
  for(uint16_t const pin : m_analogPins) {
    std::string filename = "/sys/bus/iio/devices/iio:device0/in_voltage" 
        + std::to_string(pin) + "_raw";
    std::ifstream file(filename, std::ifstream::in);
    std::string line;
    if(file.is_open()){
      std::getline(file, line);
      uint16_t rawReading = std::stoi(line);
      reading.push_back(std::make_pair(pin, rawReading*m_conversionConst));
    } else {
      std::cerr << "[" << getName() 
          << "] Could not read from analog input. (pin: " << pin 
          << ", filename: " << filename << ")" << std::endl;
      reading.push_back(std::make_pair(pin,std::nanf("")));
    }
    file.close();
  }
  return reading;
}

void IRDetection::OpenPwm()
{
  std::string filename = m_path + "/export";
  std::ofstream exportFile(filename, std::ofstream::out);
  
  if (exportFile.is_open()) {
    for (auto pin : m_pins) {
      exportFile << pin;
      exportFile.flush();
    }
    Reset();
  } else {
    cerr << "[" << getName() << "] Could not open " << filename << "." 
        << std::endl;
  }
  exportFile.close();
}

void IRDetection::ClosePwm()
{
  std::string filename = m_path + "/unexport";
  std::ofstream unexportFile(filename, std::ofstream::out);
  
  if (unexportFile.is_open()) {
    for (auto pin : m_pins) {
      SetEnabled(pin, 0);
      unexportFile << pin;
      unexportFile.flush();
    }
  } else {
    cerr << "[" << getName() << "] Could not open " << filename << "." 
        << std::endl;
  }
  unexportFile.close();
}

void IRDetection::Reset()
{
  for (uint8_t i = 0; i < m_pins.size(); i++) {
    SetEnabled(m_pins.at(i), false);
    SetPeriodNs(m_pins.at(i), m_periodsNs.at(i));
    SetDutyCycleNs(m_pins.at(i), m_dutyCyclesNs.at(i));
    SetEnabled(m_pins.at(i), true);
  }
}

void IRDetection::SetEnabled(uint16_t const a_pin, bool const a_value)
{
  std::string filename = m_path + "/pwm" + std::to_string(a_pin) + "/enable";
  std::ofstream file(filename, std::ofstream::out);
  if (file.is_open()) {
    file << std::to_string((static_cast<int32_t>(a_value)));
    file.flush();
  } else {
    cerr << "[" << getName() << "] Could not open " << filename 
        << "." << std::endl;
  }
  file.close();
}

bool IRDetection::GetEnabled(uint16_t const a_pin) const
{
  std::string filename = m_path + "/pwm" + std::to_string(a_pin) + "/enable";
  std::string line;

  std::ifstream file(filename, std::ifstream::in);
  if (file.is_open()) {
    std::getline(file, line);
    bool value = (line.compare("1") == 0);
    file.close();
    return value;
  } else {
    cerr << "[" << getName() << "] Could not open " << filename 
        << "." << std::endl;
    file.close();
    return NULL;
  }
}

void IRDetection::SetDutyCycleNs(uint16_t const a_pin, uint32_t const a_value)
{
  std::string filename = m_path + "/pwm" + std::to_string(a_pin) + "/duty_cycle";

  std::ofstream file(filename, std::ofstream::out);
  if (file.is_open()) {
    file << std::to_string(a_value);
    file.flush();
  } else {
    cerr << "[" << getName() << "] Could not open " << filename 
        << "." << std::endl;
  }

  file.close();
}

uint32_t IRDetection::GetDutyCycleNs(uint16_t const a_pin) const
{
  std::string filename = m_path + "/pwm" + std::to_string(a_pin) + "/duty_cycle";
  std::string line;

  std::ifstream file(filename, std::ifstream::in);
  if (file.is_open()) {
    std::getline(file, line);
    uint32_t value = std::stoi(line);
    file.close();
    return value;
  } else {
    cerr << "[" << getName() << "] Could not open " << filename 
        << "." << std::endl;
    file.close();
    return 0;
  }
}

void IRDetection::SetPeriodNs(uint16_t const a_pin, uint32_t const a_value)
{
  std::string filename = m_path + "/pwm" + std::to_string(a_pin) + "/period";

  std::ofstream file(filename, std::ofstream::out);
  if (file.is_open()) {
    file << std::to_string(a_value);
    file.flush();
  } else {
    cerr << "[" << getName() << "] Could not open " << filename 
        << "." << std::endl;
  }
  file.close();
}

uint32_t IRDetection::GetPeriodNs(uint16_t const a_pin) const
{
  std::string filename = m_path + "/pwm" + std::to_string(a_pin) + "/period";
  std::string line;

  std::ifstream file(filename, std::ifstream::in);
  if (file.is_open()) {
    std::getline(file, line);
    uint32_t value = std::stoi(line);
    file.close();
    return value;
  } else {
    cerr << "[" << getName() << "] Could not open " << filename 
        << "." << std::endl;
    file.close();
    return 0;
  }
}

}
}
}

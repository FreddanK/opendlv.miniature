/**
 * proxy-miniature-pwm - Interface to pwm.
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

#ifndef PROXY_MINIATURE_IRDETECTION_H
#define PROXY_MINIATURE_IRDETECTION_H


#include <memory>
#include <string>
#include <vector>
#include <utility>

#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>

namespace opendlv {
namespace proxy {
namespace miniature {

class IRDetection : public odcore::base::module::TimeTriggeredConferenceClientModule {
 public:
  IRDetection(const int &, char **);
  IRDetection(const IRDetection &) = delete;
  IRDetection &operator=(const IRDetection &) = delete;
  virtual ~IRDetection();

 private:
  void setUp();
  void tearDown();
  virtual odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();
  std::vector<std::pair<uint16_t, float>> getReadings();

  void OpenPwm();
  void ClosePwm();
  void Reset();
  void SetEnabled(uint16_t const, bool const);
  bool GetEnabled(uint16_t const) const;
  void SetDutyCycleNs(uint16_t const, uint32_t const);
  uint32_t GetDutyCycleNs(uint16_t const) const;
  void SetPeriodNs(uint16_t const, uint32_t const);
  uint32_t GetPeriodNs(uint16_t const) const;

  void ReadAnalogPins();
  void SendAnalogReadings();

  float m_conversionConst;
  bool m_debug;
  bool m_initialised;
  std::string m_path;
  std::vector<uint16_t> m_pins;
  std::vector<uint16_t> m_analogPins;
  std::vector<uint32_t> m_periodsNs;
  std::vector<uint32_t> m_dutyCyclesNs;

  std::vector<std::pair<uint16_t, float>> m_analogReadings;
  uint16_t m_numberOfReadings;
  uint16_t m_pwmOnSteps;
  uint16_t m_pwmCounterMax;

};

}
}
}

#endif

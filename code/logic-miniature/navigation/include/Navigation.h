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

#ifndef LOGIC_MINIATURE_NAVIGATION_H
#define LOGIC_MINIATURE_NAVIGATION_H

#include <map>
#include <memory>

#include <opendavinci/odcore/base/Mutex.h>
#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
#include "opendavinci/odcore/data/TimeStamp.h"

#include <opendlv/data/environment/Line.h>
#include <opendlv/data/environment/Point3.h>

#include "PID.h"

namespace opendlv {
namespace logic {
namespace miniature {

class Navigation : 
  public odcore::base::module::TimeTriggeredConferenceClientModule {
 public:
  Navigation(const int &, char **);
  Navigation(const Navigation &) = delete;
  Navigation &operator=(const Navigation &) = delete;
  virtual ~Navigation();
  virtual void nextContainer(odcore::data::Container &);

 private:
  void setUp();
  void tearDown();
  virtual odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();
  std::vector<data::environment::Point3> ReadPointString(std::string const &) const;
  std::vector<double> pathUpdateCurrentTarget(double, double, std::vector<std::vector<double>>);

  enum Direction {forward, backward, right, left};
  enum State {PathFollow, Avoid, Stop, Cruise};

  void sendMotorCommands(uint32_t leftMotorDutyCycle, uint32_t rightMotorDutyCycle);
  void sendGPIOCommands(Direction leftWheelDirection, Direction rightWheelDirection);

  odcore::base::Mutex m_mutex;
  std::vector<data::environment::Line> m_outerWalls;
  std::vector<data::environment::Line> m_innerWalls;
  std::vector<data::environment::Point3> m_pointsOfInterest;
  std::map<uint16_t, float> m_analogReadings;
  std::map<uint16_t, bool> m_gpioReadings;
  std::vector<uint16_t> m_gpioOutputPins;
  std::vector<uint16_t> m_pwmOutputPins;

  double m_pruReading;
  odcore::data::TimeStamp m_sonarDetectionTime;
  double m_xPositionLPS;
  double m_yPositionLPS;
  double m_yawLPS;

  uint32_t m_prevLeftMotorDutyCycle;
  uint32_t m_prevRightMotorDutyCycle;
  Direction m_prevLeftWheelDirection;
  Direction m_prevRightWheelDirection;
  
  PIDController m_PIDController;
  std::vector<std::vector<double>> m_path;
  uint16_t m_pathCurrentPointIndex;
  Direction m_followPathDirection;
  
  State m_currentState;
  double m_stateTimer;
  double m_stateTimeout;
  double m_deltaTime;
};

}
}
}

#endif

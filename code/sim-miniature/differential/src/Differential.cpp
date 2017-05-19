/**
 * Copyright (C) 2017 Chalmers Revere
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

#include <iostream>
#include <string>
#include <math.h>

#include <opendavinci/odcore/base/KeyValueConfiguration.h>
#include <opendavinci/odcore/base/Lock.h>
#include <opendavinci/odcore/data/Container.h>
#include <opendavinci/odcore/data/TimeStamp.h>

#include <opendlv/data/environment/Point3.h>

#include <odvdopendlvdata/GeneratedHeaders_ODVDOpenDLVData.h>
#include <odvdminiature/GeneratedHeaders_ODVDMiniature.h>

#include "Differential.h"

namespace opendlv {
namespace sim {
namespace miniature {

Differential::Differential(const int &argc, char **argv)
  : TimeTriggeredConferenceClientModule(
      argc, argv, "sim-miniature-differential")
  , m_mutex()
  , m_currentEgoState()
  , m_debug()
  , m_gpioIn30(false)
  , m_gpioIn31(false)
  , m_gpioIn51(false)
  , m_gpioIn60(false)
  , m_deltaTime()
  , m_leftMotorDutyCycleNs(0)
  , m_rightMotorDutyCycleNs(0)
{
}

Differential::~Differential()
{
}

void Differential::nextContainer(odcore::data::Container &a_c)
{
  odcore::base::Lock l(m_mutex);

  int32_t dataType = a_c.getDataType();

  if (dataType == automotive::miniature::SensorBoardData::ID()) {
    auto sensorBoardData = 
        a_c.getData<automotive::miniature::SensorBoardData>();
    if (m_debug) {
      std::cout << "[" << getName() << "] Received an SensorBoardData: " 
          << sensorBoardData.toString() << "." << std::endl;
    }
    ConvertBoardDataToSensorReading(sensorBoardData);
  } else if (dataType == opendlv::proxy::ToggleRequest::ID()) {
    auto request = a_c.getData<opendlv::proxy::ToggleRequest>();
    uint16_t pin = request.getPin();
    bool state = (request.getState() == opendlv::proxy::ToggleRequest::ToggleState::On);
    SetMotorControl(pin, state);
    if (m_debug) {
      std::cout << "[" << getName() << "] Received a ToggleRequest: "
          << request.toString() << "." << std::endl;
    }
  } else if (dataType == opendlv::proxy::PwmRequest::ID()) {
    auto request = a_c.getData<opendlv::proxy::PwmRequest>();
    if (m_debug) {
      std::cout << "[" << getName() << "] Received a PwmRequest: "
          << request.toString() << "." << std::endl;
    }
    uint16_t senderStamp = a_c.getSenderStamp();
    uint32_t dutyCycleNs = request.getDutyCycleNs();
    if (senderStamp == 1)
      m_leftMotorDutyCycleNs = dutyCycleNs;
    else if (senderStamp == 2)
      m_rightMotorDutyCycleNs = dutyCycleNs;
  } 
}

void Differential::setUp()
{
  odcore::base::KeyValueConfiguration kv = getKeyValueConfiguration();

  bool valueFound;
  m_debug = kv.getOptionalValue<bool>("sim-miniature-differential.debug", 
      valueFound);
  if (!valueFound) {
    m_debug = false;
  }

  m_deltaTime = 1 / getFrequency();
}

void Differential::tearDown()
{
}

odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Differential::body()
{
  while (getModuleStateAndWaitForRemainingTimeInTimeslice() == 
      odcore::data::dmcp::ModuleStateMessage::RUNNING) {
  
    odcore::base::Lock l(m_mutex);
 
//    std::cout << "gpio30: " << m_gpioIn30 << endl;
//    std::cout << "gpio31: " << m_gpioIn31 << endl;
//    std::cout << "gpio51: " << m_gpioIn51 << endl;
//    std::cout << "gpio60: " << m_gpioIn60 << endl;

    opendlv::data::environment::Point3 prevPosition = 
      m_currentEgoState.getPosition();
    opendlv::data::environment::Point3 prevRotation = 
      m_currentEgoState.getRotation();
    opendlv::data::environment::Point3 prevVelocity = 
      m_currentEgoState.getVelocity();

    double prevVelX = prevVelocity.getX();
    double prevVelY = prevVelocity.getY();
    
    // The division is needed due to a scaling problem, in order to convert into
    // meters. In your code below, everything will be meters.
    double prevPosX = prevPosition.getX() / 10.0;
    double prevPosY = prevPosition.getY() / 10.0;

    double prevYaw = atan2(prevRotation.getY(), prevRotation.getX());
    // NOTE: Do not change the code above.

    // Convert from duty cycle to angular velocity
    double leftWheelAngularVelocity = 
      ConvertDutyCycleNsToWheelAngularVelocity(m_leftMotorDutyCycleNs);
    double rightWheelAngularVelocity =
      ConvertDutyCycleNsToWheelAngularVelocity(m_rightMotorDutyCycleNs);
    
    // Set wheel directions based on gpio pins
    if (m_gpioIn30 && !m_gpioIn31) {
      // Forward
      leftWheelAngularVelocity = (leftWheelAngularVelocity < 0) ? 
       -leftWheelAngularVelocity : leftWheelAngularVelocity;
    } else if (!m_gpioIn30 && m_gpioIn31) {
      // Backward
      leftWheelAngularVelocity = (leftWheelAngularVelocity >= 0) ?
        -leftWheelAngularVelocity : leftWheelAngularVelocity;
    } else {
      leftWheelAngularVelocity = 0.0f;
    }
    if (m_gpioIn51 && !m_gpioIn60) {
      // Forward
      rightWheelAngularVelocity = (rightWheelAngularVelocity < 0) ?
        -rightWheelAngularVelocity : rightWheelAngularVelocity;
    } else if (!m_gpioIn51 && m_gpioIn60) {
      // Backward
      rightWheelAngularVelocity = (rightWheelAngularVelocity >= 0) ?
        -rightWheelAngularVelocity : rightWheelAngularVelocity;
    } else {
      rightWheelAngularVelocity = 0.0f;
    }

    // Constant definitions
    double const radiusBody = 0.12; // (m)
    double const radiusWheel = 0.06; // (m)

    // Kinematic equations below
    double bodyVelocity = radiusWheel*(leftWheelAngularVelocity 
                            + rightWheelAngularVelocity)/2;
    double yawRate = -radiusWheel*(leftWheelAngularVelocity 
                        - rightWheelAngularVelocity)/(2*radiusBody); 

    // Integrate yawRate so that yaw can be used to calculate velX and velY
    double yaw = prevYaw + yawRate*m_deltaTime; 

    // Kinematic equations using yaw
    double velX = bodyVelocity*cos(yaw); 
    double velY = bodyVelocity*sin(yaw); 
   
    // Integration step to get position
    double posX = prevPosX + velX*m_deltaTime; 
    double posY = prevPosY + velY*m_deltaTime; 
    
    // Due to a simulation scaling problem, the position is scaled. 
    // NOTE: Do not change the code below.
    posX = posX * 10.0;
    posY = posY * 10.0;

    double posZ = 0.0;
    double velZ = 0.0;
    double accZ = 0.0;

    double accX = (velX - prevVelX) / m_deltaTime;
    double accY = (velY - prevVelY) / m_deltaTime;

    opendlv::data::environment::Point3 position(posX, posY, posZ);
    opendlv::data::environment::Point3 rotation(1.0, 0.0, 0.0);
    opendlv::data::environment::Point3 velocity(velX, velY, velZ);
    opendlv::data::environment::Point3 acceleration(accX, accY, accZ);


    rotation.rotateZ(yaw);
    rotation.normalize();

    opendlv::data::environment::EgoState egoState(position, rotation, velocity,
        acceleration);

    m_currentEgoState = egoState;

    odcore::data::Container c(egoState);
    getConference().send(c);

    // Simulate LPS.
    opendlv::model::Cartesian3 lpsPosition(static_cast<float>(posX), static_cast<float>(posY), 0.0f);
    opendlv::model::Cartesian3 lpsOrientation(0.0f, 0.0f, static_cast<float>(yaw));
    opendlv::model::State lpsState(lpsPosition, lpsOrientation, 0);
    odcore::data::Container lpsContainer(lpsState);
    getConference().send(lpsContainer);
  }

  return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
}

double Differential::ConvertDutyCycleNsToWheelAngularVelocity(uint32_t a_dutyCycleNs)
{
    int32_t const minDutyCycleNs = 25000; 
    int32_t const maxDutyCycleNs = 50000; 

    double const maxAngularVelocity = 10.0;

    a_dutyCycleNs = (a_dutyCycleNs < minDutyCycleNs) ? minDutyCycleNs : a_dutyCycleNs; 
    a_dutyCycleNs = (a_dutyCycleNs > maxDutyCycleNs) ? maxDutyCycleNs : a_dutyCycleNs;
    
    return maxAngularVelocity * (a_dutyCycleNs - minDutyCycleNs) / 
      static_cast<double>(maxDutyCycleNs - minDutyCycleNs); 
}

void Differential::ConvertBoardDataToSensorReading(
  automotive::miniature::SensorBoardData const &a_sensorBoardData)
{
  std::map<uint32_t, double> mapOfDistances = 
      a_sensorBoardData.getMapOfDistances();
 
  double const maxDistance = 4.0f * 10;

  for (auto distanceReading : mapOfDistances) {
    uint32_t sensorId = distanceReading.first;
    double distance = distanceReading.second;
    
    std::cout << "Distance: " << distance << std::endl;

    float voltage = 850.0f;
    if (distance > 0.0 && distance < maxDistance) {
      voltage = 850.0f * static_cast<float>(distance / maxDistance);
    }

    if (sensorId == 3) { // Ping sensor
      opendlv::proxy::ProximityReading proximityReading(
          (distance < 0.0) ? -1 : distance*10);
      odcore::data::Container proximityContainer(proximityReading);
      getConference().send(proximityContainer);
    } else { // IR sensor
      opendlv::proxy::AnalogReading analogReading(sensorId, voltage);
      odcore::data::Container analogContainer(analogReading);
      getConference().send(analogContainer);
    }   
  }
}

void Differential::SetMotorControl(uint16_t a_pin, bool a_state)
{
  switch (a_pin) {
    case 30:
      {
        m_gpioIn30 = a_state;
        break;
      }
    case 31:
      {
        m_gpioIn31 = a_state;
        break;
      }
    case 51:
      {
        m_gpioIn51 = a_state;
        break;
      }
    case 60:
      {
        m_gpioIn60 = a_state;
        break;
      }
    default:
      {
      }
  }
}

}
}
} 

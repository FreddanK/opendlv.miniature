#include "PID.h"

PIDController::PIDController(double Kp, double Ki, double Kd)
  : m_Kp(Kp)
  , m_Ki(Ki)
  , m_Kd(Kd)
  , m_integratedInput(0.0)
  , m_lastInput(0.0)
{
}

double PIDController::update(double input, double dt)
{
  double output = m_Kp*input +
    m_Ki*m_integratedInput +
    m_Kd*(input - m_lastInput)/dt;

  m_lastInput += input*dt;
  
  return output;
}

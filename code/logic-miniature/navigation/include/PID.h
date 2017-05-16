class PIDController
{
 public:
  PIDController(double, double, double);
  double update(double, double);

 private:
  double m_Kp;
  double m_Ki;
  double m_Kd;

  double m_integratedInput;
  double m_lastInput;
};

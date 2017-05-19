#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(Eigen::Vector3d initState)
  : m_stateEstimate(initState)
  , m_Q(Eigen::MatrixXd::Identity(3, 3))
  , m_R(Eigen::MatrixXd::Identity(2, 2))
  , m_P(Eigen::MatrixXd::Identity(3, 3))
{
	
}


KalmanFilter::KalmanFilter(Eigen::Vector3d initState, Eigen::Matrix3d Q, Eigen::Matrix3d R)
  : m_stateEstimate(initState)
  , m_Q(Q)
  , m_R(R)
  , m_P(Eigen::MatrixXd::Identity(3, 3))
{
	
}


KalmanFilter::KalmanFilter(Eigen::Vector3d initState, Eigen::Matrix3d Q, Eigen::Matrix3d R, Eigen::Matrix3d P_0)
  : m_stateEstimate(initState)
  , m_Q(Q)
  , m_R(R)
  , m_P(P_0)
{
	
}


void KalmanFilter::setQ(Eigen::Matrix3d Q)
{
  m_Q = Q;
}


void KalmanFilter::setR(Eigen::Matrix3d R)
{
  m_R = R;
}


/*
Eigen::Vector3d KalmanFilter::updateEstimate(double sensorX, double sensorY, double sensorTheta,
  double speedLeftWheel, double speedRightWheel, double dt)
{
  // This function will update and return the state estimate from the Kalman filter
  // given the current sensor readings.

  Eigen::Matrix3d F = getModelJacobian(speedLeftWheel, speedRightWheel, dt);
  Eigen::MatrixXd H = getSensorJacobian();
  Eigen::MatrixXd K(3, 3); // Kalman gain
  Eigen::Vector3d y(sensorX, sensorY, sensorTheta);

  // Prediction step:
  Eigen::Vector3d predictedState = modelFunction(speedLeftWheel, speedRightWheel, dt);
  m_P = (F * m_P * F.transpose()) + m_Q;

  // Update step:
  K = m_P * H.transpose() * ((H * m_P * H.transpose()) + m_R).inverse();
  m_stateEstimate = predictedState + K*(y - sensorFunction());
  m_P = (Eigen::MatrixXd::Identity(3, 3) - (K * H)) * m_P;

  return m_stateEstimate;
}

Eigen::Vector3d KalmanFilter::updateEstimateNoSensorData(
  double speedLeftWheel, double speedRightWheel, double dt)
{
  // This function will update and return the state estimate from the Kalman filter,
  // without new sensor readings.
  
  m_stateEstimate = modelFunction(speedLeftWheel, speedRightWheel, dt);
  return m_stateEstimate;
}*/


Eigen::Vector3d KalmanFilter::getStateEstimate()
{
  return m_stateEstimate;
}


void KalmanFilter::doPredictionStep(double leftWheelSpeed, double rightWheelSpeed, double dt)
{
  Eigen::Matrix3d F = getModelJacobian(leftWheelSpeed, rightWheelSpeed, dt);

  // Prediction step:
  m_stateEstimate = modelFunction(leftWheelSpeed, rightWheelSpeed, dt);
  m_P = (F * m_P * F.transpose()) + m_Q;
}


void KalmanFilter::doUpdateStep(double sensorX, double sensorY, double sensorTheta)
{
  Eigen::MatrixXd H = getSensorJacobian();
  Eigen::MatrixXd K(3, 3); // Kalman gain
  Eigen::Vector3d y(sensorX, sensorY, sensorTheta);

  // Update step:
  K = m_P * H.transpose() * ((H * m_P * H.transpose()) + m_R).inverse();
  m_stateEstimate += K*(y - sensorFunction());
  m_P = (Eigen::MatrixXd::Identity(3, 3) - (K * H)) * m_P;
}


Eigen::Vector3d KalmanFilter::modelFunction(double speedLeftWheel, double speedRightWheel, double dt)
{
  // Model function describing the dynamics of the system.

  double x = m_stateEstimate(0);
  double y = m_stateEstimate(1);
  double theta = m_stateEstimate(2);

  x += dt*cos(theta)*((speedRightWheel+speedLeftWheel)/2);
  y += dt*sin(theta)*((speedRightWheel+speedLeftWheel)/2);
  theta += dt*(-(speedLeftWheel-speedRightWheel)/2);

  return Eigen::Vector3d(x, y, theta);
}


Eigen::Vector3d KalmanFilter::sensorFunction()
{
  // Sensor function describing the behavior of the sensors.

  return Eigen::Vector3d(m_stateEstimate(0), m_stateEstimate(1), m_stateEstimate(2));
}


Eigen::Matrix3d KalmanFilter::getModelJacobian(double speedLeftWheel, double speedRightWheel, double dt)
{
  // Returns the Jacobian of the model function evaluated at the current state
  // with the current inputs (given as arguments).

  double theta = m_stateEstimate(2);

  Eigen::Matrix3d jacobian;
  jacobian << 1.0, 0, -sin(theta)*dt*((speedLeftWheel+speedRightWheel)/2),
              0, 1.0,  cos(theta)*dt*((speedLeftWheel+speedRightWheel)/2),
              0,   0,  1.0;

  return jacobian;
}


Eigen::Matrix3d KalmanFilter::getSensorJacobian()
{
  // Returns the Jacobian of the sensor function.

  Eigen::MatrixXd jacobian(3, 3);
  jacobian << 1, 0, 0,
              0, 1, 0,
              0, 0, 1;

  return jacobian; 
}

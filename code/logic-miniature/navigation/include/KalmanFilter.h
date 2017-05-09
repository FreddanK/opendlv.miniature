#include <math.h>

#include <opendavinci/Eigen/Dense> 


// All units are expected to be in SI units.

class KalmanFilter {
 public:
  KalmanFilter(Eigen::Vector3d initState);
  KalmanFilter(Eigen::Vector3d initState, Eigen::Matrix3d Q, Eigen::Matrix2d R);
  KalmanFilter(Eigen::Vector3d initState, Eigen::Matrix3d Q, Eigen::Matrix2d R, Eigen::Matrix3d P_0);

  void setQ(Eigen::Matrix3d Q);
  void setR(Eigen::Matrix2d R);

  Eigen::Vector3d updateEstimate(double sensorX, double sensorY,
    double speedLeftWheel, double speedRightWheel, double dt);
  Eigen::Vector3d updateEstimateNoSensorData(
    double speedLeftWheel, double speedRightWheel, double dt);
  Eigen::Vector3d getStateEstimate();

 private:
  Eigen::Vector3d modelFunction(double speedLeftWheel, double speedRightWheel, double dt);
  Eigen::Vector2d sensorFunction();

  Eigen::Matrix3d getModelJacobian(double speedLeftWheel, double speedRightWheel, double dt);
  Eigen::Matrix2d getSensorJacobian();

  Eigen::Vector3d m_stateEstimate; 
  Eigen::Matrix3d m_Q;
  Eigen::Matrix2d m_R;
  Eigen::Matrix3d m_P;
};

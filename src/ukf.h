#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <memory>
#include <vector>
#include <map>
#include <string>
#include <fstream>
#include "ukf_update.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;



class UKF {
public:

  ///* State of object being track.
  ///  An array of these could be used to track multiple objects.
  TrackedObject tracked_object_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();


  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(const MeasurementPackage* meas_package);


private:
  //List of processors used to handle state updates for different
  //sensor types.

  using TUnscentedKalmanUpdatePtr = std::shared_ptr<UKF_update>;
  using TUpdaterList = std::map<MeasurementPackage::SensorType, TUnscentedKalmanUpdatePtr>;
  TUpdaterList updater_map_;

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Sigma point spreading parameter
  double lambda_;

};

#endif /* UKF_H */

#include "ukf.h"
#include "Eigen/Dense"
#include "radar_update.h"
#include "lidar_update.h"
#include "no_op_update.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() :
  is_initialized_(false),
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_(true),
  // if this is false, radar measurements will be ignored (except during init)
  use_radar_(true),
  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_(0.05),
  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_(0.062)
{

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  double lambda = 3 - aug_state_dimension;

  // Laser measurement noise standard deviation position1 in m
  auto std_laspx = 0.015;
  // Laser measurement noise standard deviation position2 in m
  auto std_laspy = 0.015;
  auto laserUpdate = make_shared<LaserUpdate>(std_a_, std_yawdd_, std_laspx, std_laspy, lambda);


  // Radar measurement noise standard deviation radius in m
  auto std_radr = 0.3;
  // Radar measurement noise standard deviation angle in rad
  auto std_radphi = 0.03;
  // Radar measurement noise standard deviation radius change in m/s
  auto std_radrd = 0.3;
  auto radarUpdate = make_shared<RadarUpdate>(std_a_, std_yawdd_, std_radr, std_radphi, std_radrd, lambda);

  //Add the supported sensor types to our lookup table.
  updater_map_[MeasurementPackage::LASER] = laserUpdate;
  updater_map_[MeasurementPackage::RADAR] = radarUpdate;
}

UKF::~UKF() 
{
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(const MeasurementPackage* meas_package) {

  UKF_update* updater = updater_map_[meas_package->GetSensorType()].get();

  if (!is_initialized_)
  {
    updater->InitialUpdate(&tracked_object_, *meas_package);

    tracked_object_.timestamp_ = meas_package->GetTimeStamp();

    //Remove update objects we aren't going to use.
    if (!use_laser_)
    {
      updater_map_[MeasurementPackage::LASER] = make_shared<NoOpUpdate>();
    }

    if (!use_radar_)
    {
      updater_map_[MeasurementPackage::RADAR] = make_shared<NoOpUpdate>();
    }

    is_initialized_ = true;
    return;
  }

  double delta_t = (meas_package->GetTimeStamp() - tracked_object_.timestamp_) / 1000000.0;
  tracked_object_.timestamp_ = meas_package->GetTimeStamp();
  if (delta_t > 0.0001)
  {
    updater->Prediction(&tracked_object_, delta_t);
  }
  updater->Update(&tracked_object_, *meas_package);
}



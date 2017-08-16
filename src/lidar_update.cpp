#include "lidar_update.h"

LaserUpdate::LaserUpdate(const TNoiseCovarianceMatrix& process_noise_covariance, double std_laspx, double std_laspy, double lambda) :
  UnscentedKalmanUpdateBase(process_noise_covariance, lambda),
  std_laspx_(std_laspx),
  std_laspy_(std_laspy)
{
}

LaserUpdate::~LaserUpdate() {}

void LaserUpdate::InitialUpdate(TrackedObject* tracked_object, const MeasurementPackage& meas_mackage)
{
  tracked_object->P_ = TrackedObject::TCovarianceMatrix::Identity();

  //Todo change this to real initilization.
  tracked_object->x_ = TrackedObject::TStateVector::Zero();
}

/**
* Updates the state and the state covariance matrix using a laser measurement.
* @param {MeasurementPackage} meas_package
*/
void LaserUpdate::Update(TrackedObject* tracked_object, const MeasurementPackage& meas_package)
{
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}


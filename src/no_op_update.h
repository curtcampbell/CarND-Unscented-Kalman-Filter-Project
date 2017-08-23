#pragma once

#include "ukf_update.h"

///
// This class has the lidar specific Unscented Kalman Filter update logic.
class NoOpUpdate : virtual public UKF_update
{
public:

  NoOpUpdate() {};
  virtual ~NoOpUpdate() {};

  virtual void InitialUpdate(TrackedObject* tracked_object, const MeasurementPackage& meas_mackage) {};
  virtual void Prediction(TrackedObject* tracked_object, double delta_t) {};
  virtual void Update(TrackedObject* tracked_object, const MeasurementPackage& meas_package) {};
};


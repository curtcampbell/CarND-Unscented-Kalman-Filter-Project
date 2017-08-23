#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

struct MeasurementPackage {
  enum SensorType{
    UNDEFINED = -1,
    LASER = 0,
    RADAR = 1
  };

  MeasurementPackage(SensorType sensor_type, long long timestamp):sensor_type_(sensor_type), timestamp_(timestamp)  {  }
  virtual ~MeasurementPackage() {}

  SensorType GetSensorType() const { return sensor_type_;  }
  long long GetTimeStamp() const { return timestamp_; }

  MeasurementPackage():sensor_type_(UNDEFINED) {}

private:
    long long timestamp_;
    SensorType sensor_type_;
};

struct LidarMeasurement: public MeasurementPackage
{
  static const int measurment_dim = 2;
  using TLidarMeasurementVector = Eigen::Matrix<double, measurment_dim, 1>;

    LidarMeasurement(long long timestamp, float x,float y): x_(x), y_(y), MeasurementPackage(MeasurementPackage::LASER, timestamp){    }
    virtual ~LidarMeasurement() {}

    float GetX() const { return x_; }
    float GetY() const { return y_; }

    TLidarMeasurementVector GetVector() const 
    { 
      TLidarMeasurementVector vector;
      vector << x_, y_;
      return vector;
    }

private:
    float x_;
    float y_;
};

struct RadarMeasurement: public MeasurementPackage
{
    static const int measurment_dim = 3;
    using TRadarMeasurementVector = Eigen::Matrix<double, measurment_dim, 1>;

    RadarMeasurement(long long timestamp, float rho, float theta, float rho_dot) : rho_(rho), theta_(theta), rho_dot_(rho_dot), MeasurementPackage(MeasurementPackage::RADAR, timestamp) {}
    virtual ~RadarMeasurement() {}

    float GetRho() const { return rho_; }
    float GetTheta() const { return theta_; }
    float GetRhoDot() const { return rho_dot_; }

    TRadarMeasurementVector GetVector() const 
    { 
      TRadarMeasurementVector vector;;
      vector << rho_, theta_, rho_dot_;
      return vector;
    }

private:
    float rho_;
    float theta_;
    float rho_dot_;
};

#endif /* MEASUREMENT_PACKAGE_H_ */


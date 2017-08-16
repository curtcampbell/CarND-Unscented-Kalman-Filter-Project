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
  long GetTimeStamp() const { return timestamp_; }

  MeasurementPackage():sensor_type_(UNDEFINED) {}

private:
    long long timestamp_;
    SensorType sensor_type_;
};

struct LidarMeasurement: public MeasurementPackage
{
    using TLidarMeasurementVector = Eigen::Matrix<double, 2, 1>;

    LidarMeasurement(long long timestamp, float x,float y): x_(x), y_(y), MeasurementPackage(MeasurementPackage::LASER, timestamp){    }
    virtual ~LidarMeasurement() {}

    float GetX() const { return x_; }
    float GetY() const { return x_; }

    TLidarMeasurementVector GetVector() const { TLidarMeasurementVector() << x_, y_; }

private:
    float x_;
    float y_;
};

struct RadarMeasurement: public MeasurementPackage
{
    using TRadarMeasurementVector = Eigen::Matrix<double, 3, 1>;

    RadarMeasurement(long long timestamp, float ro, float theta, float ro_dot) : ro_(ro), theta_(theta), ro_dot_(ro_dot), MeasurementPackage(MeasurementPackage::RADAR, timestamp) {}
    virtual ~RadarMeasurement() {}

    float GetRo() const { return ro_; }
    float GetTheta() const { return ro_; }
    float GetRoDot() const { return ro_dot_; }

    TRadarMeasurementVector GetVector() const { TRadarMeasurementVector() << ro_, theta_, ro_dot_; }

private:
    float ro_;
    float theta_;
    float ro_dot_;
};

#endif /* MEASUREMENT_PACKAGE_H_ */


#pragma once

#include "quadcopter_model/common/types.hpp"
#include "quadcopter_model/sensors/sensor_base.hpp"

namespace quadcopter {

class IMU : SensorBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  IMU();
  ~IMU();

 private:
};
}  // namespace quadcopter

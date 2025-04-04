/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#pragma once

#include <Eigen/Dense>

namespace dpth_sensor_simulator {

struct DpthSensorSimParams {
  double accuracy;     // Full scale accuracy of the pressure sensor (%)
  double resolution;   // Full scale resolution of the pressure sensor (%)
  double range;        // Measurement range of the pressure sensor (Pa)
  double static_bias;  // Static bias of the pressure sensor (Pa)
  double
      pressure_per_metre;  // Conversion factor from pressure to depth (Pa/mH2O)
  Eigen::Vector3d p_bs_b;  // Lever arm between body and sensor frame (m)
};

struct DpthSensorSimEnableParams {
  bool enable_white_noise;   // Enable white noise
  bool enable_static_bias;   // Enable static bias
  bool enable_quantization;  // Enable quantization
  bool enable_saturation;    // Enable saturation
};

struct DpthSensorMeasurement {
  double pressure;  // Pressure measurement (Pa)
  double depth;     // Depth measurement (m)
  double
      pressure_std_dev;  // Standard deviation of the pressure measurement (Pa)
  double depth_std_dev;  // Standard deviation of the depth measurement (m)
};

}  // namespace dpth_sensor_simulator

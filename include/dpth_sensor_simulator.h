/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#pragma once

#include <random>

#include "dpth_sensor_simulator_structures.h"

namespace dpth_sensor_simulator {

class DpthSensorSimulator {
 public:
  // Default Constructor
  DpthSensorSimulator();

  // Constructor with parameters
  DpthSensorSimulator(const DpthSensorSimParams dpthSensorSimParams,
                      const DpthSensorSimEnableParams dpthSensorSimEnableParams,
                      const double dt, const unsigned int seed,
                      const bool use_fixed_random_numbers);

  // Destructor
  ~DpthSensorSimulator();

  // Measurement generation function
  DpthSensorMeasurement generateMeasurement(const double true_depth);

  // Getter functions
  DpthSensorSimParams getSimParams() const;
  DpthSensorSimEnableParams getSimEnableParams() const;
  bool getEnableWhiteNoise() const;
  bool getEnableStaticBias() const;
  bool getEnableQuantization() const;
  bool getEnableSaturation() const;
  double getPressureToMetreConversionFactor() const;
  double getMetreToPressureConversionFactor() const;
  double getSampleTime() const;
  unsigned int getSeed() const;
  bool getEnableFixedRandomNumbers() const;

  // Setter functions
  void setSimParams(const DpthSensorSimParams dpthSensorSimParams);
  void setSimEnableParams(
      const DpthSensorSimEnableParams dpthSensorSimEnableParams);
  void setEnableWhiteNoise(const bool enable);
  void setEnableStaticBias(const bool enable);
  void setEnableQuantization(const bool enable);
  void setEnableSaturation(const bool enable);
  void setPressureToMetreConversionFactor(const double conversion_factor);
  void setSampleTime(const double dt);
  void setSeed(const unsigned int seed);
  void setEnableFixedRandomNumbers(const bool enable);

  // Print function
  std::stringstream printSimulatorParameters();

 private:
  // Depth sensor simulator parameters
  DpthSensorSimParams dpthSensorSimParams_;

  // Depth sensor simulator enable parameters
  DpthSensorSimEnableParams dpthSensorSimEnableParams_;

  // Pressure measurement standard deviation
  double pressure_standard_deviation_;

  // Sampling time
  double dt_;

  // Random number generator for normal distributiona and uniform distribution
  std::mt19937 randomNumberGenerator_;
  unsigned int seed_;
  std::normal_distribution<> normalDistribution_;
  std::uniform_real_distribution<> uniformDistribution_;

  // Flag indicating if fixed random numbers are be used (for testing/debugging)
  bool use_fixed_random_numbers_;

  // White noise model
  double calcNoisyMeasurement(const double pressure_measurement);

  // Static bias model
  double calcStaticBiasModel(const double pressure_measurement);

  // Quantization model
  double calcQuantizationModel(const double pressure_measurement);

  // Saturation model
  double calcSaturationModel(const double pressure_measurement);

  // Depth to pressure conversion
  double convertDepthToPressure(const double depth);

  // Pressure to depth conversion
  double convertPressureToDepth(const double pressure);

  // Random number generation function
  double drawRandNormalDistNum();
};

}  // namespace dpth_sensor_simulator

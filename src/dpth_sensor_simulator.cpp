/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#include <iomanip>

#include "dpth_sensor_simulator.h"

namespace dpth_sensor_simulator {
/**
 * @brief Default Constructor of the DpthSensorSimulator class.
 *
 * This constructor initializes the depth pressure sensor simulator with default parameters.
 *
 * @return An instance of the DpthSensorSimulator class initialized with default
 * parameters. The returned instance is ready for simulation and further
 * configuration.
 */
DpthSensorSimulator::DpthSensorSimulator()
    : dt_(0.5),
      randomNumberGenerator_(42),
      seed_(42),
      use_fixed_random_numbers_(false) {
  // Initialize the depth pressure sensor simulator parameters
  dpthSensorSimParams_.accuracy = 0.05;       // (%)
  dpthSensorSimParams_.resolution = 0.005;    // (%)
  dpthSensorSimParams_.range = 5000000.0;     // (Pa)
  dpthSensorSimParams_.static_bias = 101325;  // (Pa); 1 atm
  dpthSensorSimParams_.pressure_per_metre =
      10110.8212387044;  // (Pa/mH2O); NM III station
  dpthSensorSimParams_.p_bs_b = Eigen::Vector3d::Zero();  // (m)

  // Initialize the depth pressure sensor simulator enable parameters
  dpthSensorSimEnableParams_.enable_white_noise = true;
  dpthSensorSimEnableParams_.enable_static_bias = true;
  dpthSensorSimEnableParams_.enable_quantization = true;
  dpthSensorSimEnableParams_.enable_saturation = true;

  // Calculate the standard deviation of the pressure sensor
  pressure_standard_deviation_ = dpthSensorSimParams_.accuracy *
                                 dpthSensorSimParams_.range / 100.0;  // (Pa)
}

/**
 * @brief Constructor with parameters of the DpthSensorSimulator class.
 *
 * This constructor initializes the depth pressure sensor simulator with parameters.
 *
 * @param[in] dpthSensorSimParams Parameters of the depth pressure sensor simulator.
 * @param[in] dpthSensorSimEnableParams Enable parameters of the depth pressure sensor simulator.
 * @param[in] dt Sample time of the depth pressure sensor simulator.
 * @param[in] seed Seed for the random number generator.
 * @param[in] use_fixed_random_numbers Flag to enable fixed random numbers for testing/debugging.
 *
 * @return An instance of the DpthSensorSimulator class initialized with
 * parameters. The returned instance is ready for simulation and further
 * configuration.
 */
DpthSensorSimulator::DpthSensorSimulator(
    const DpthSensorSimParams dpthSensorSimParams,
    const DpthSensorSimEnableParams dpthSensorSimEnableParams, const double dt,
    const unsigned int seed, const bool use_fixed_random_numbers)
    : dpthSensorSimParams_(dpthSensorSimParams),
      dpthSensorSimEnableParams_(dpthSensorSimEnableParams),
      dt_(dt),
      randomNumberGenerator_(seed),
      seed_(seed),
      use_fixed_random_numbers_(use_fixed_random_numbers) {
  // Calculate the standard deviation of the pressure sensor
  pressure_standard_deviation_ = dpthSensorSimParams_.accuracy *
                                 dpthSensorSimParams_.range / 100.0;  // (Pa)
}

/**
 * @brief Destructor of the DpthSensorSimulator class.
 *
 * This destructor is responsible for cleaning up the resources used by the depth pressure sensor
 * simulator.
 *
 * @return None
 */
DpthSensorSimulator::~DpthSensorSimulator() {}

/**
 * @brief Generates a pressure measurement with noise, bias, quantization, and saturation.
 * 
 * This function generates a pressure measurement with noise, bias, quantization, and saturation.
 * The models for noise, bias, quantization, and saturation can be enabled or disabled.
 * The function needs the true depth as input to generate the pressure measurement.
 * The true depth is the z-component of the position vector p_nb_n in m.
 * The index n of p_nb_n denotes the navigation frame, and the index b denotes the body frame.
 * 
 * @param[in] true_depth True depth (z-component of p_nb_n) in m.
 * 
 * @return Depth sensor measurement with pressure (Pa) and depth (m).
*/
DpthSensorMeasurement DpthSensorSimulator::generateMeasurement(
    const double true_depth) {
  double pressure_measurement = convertDepthToPressure(true_depth);

  // Add white noise
  if (dpthSensorSimEnableParams_.enable_white_noise) {
    pressure_measurement = calcNoisyMeasurement(pressure_measurement);
  }

  // Add static bias
  if (dpthSensorSimEnableParams_.enable_static_bias) {
    pressure_measurement = calcStaticBiasModel(pressure_measurement);
  }

  // Quantize the measurement
  if (dpthSensorSimEnableParams_.enable_quantization) {
    pressure_measurement = calcQuantizationModel(pressure_measurement);
  }

  // Saturate the measurement
  if (dpthSensorSimEnableParams_.enable_saturation) {
    pressure_measurement = calcSaturationModel(pressure_measurement);
  }

  double depth_measurement = convertPressureToDepth(pressure_measurement);

  // Assign measurement to depth sensor measurement struct
  DpthSensorMeasurement dpthSensorMeasurement;
  dpthSensorMeasurement.pressure = pressure_measurement;
  dpthSensorMeasurement.depth = depth_measurement;
  dpthSensorMeasurement.pressure_std_dev = pressure_standard_deviation_;
  dpthSensorMeasurement.depth_std_dev =
      pressure_standard_deviation_ / dpthSensorSimParams_.pressure_per_metre;

  return dpthSensorMeasurement;
}

/**
 * @brief Getter function for the depth pressure sensor simulator parameters.
 * 
 * @return Depth pressure sensor simulator parameters.
*/
DpthSensorSimParams DpthSensorSimulator::getSimParams() const {
  return dpthSensorSimParams_;
}

/**
 * @brief Getter function for the depth pressure sensor simulator enable parameters.
 * 
 * @return Depth pressure sensor simulator enable parameters.
*/
DpthSensorSimEnableParams DpthSensorSimulator::getSimEnableParams() const {
  return dpthSensorSimEnableParams_;
}

/**
 * @brief Getter function for the white noise enable flag.
 * 
 * @return White noise enable flag.
*/
bool DpthSensorSimulator::getEnableWhiteNoise() const {
  return dpthSensorSimEnableParams_.enable_white_noise;
}

/**
 * @brief Getter function for the static bias enable flag.
 * 
 * @return Static bias enable flag.
*/
bool DpthSensorSimulator::getEnableStaticBias() const {
  return dpthSensorSimEnableParams_.enable_static_bias;
}

/**
 * @brief Getter function for the quantization enable flag.
 * 
 * @return Quantization enable flag.
*/
bool DpthSensorSimulator::getEnableQuantization() const {
  return dpthSensorSimEnableParams_.enable_quantization;
}

/**
 * @brief Getter function for the saturation enable flag.
 * 
 * @return Saturation enable flag.
*/
bool DpthSensorSimulator::getEnableSaturation() const {
  return dpthSensorSimEnableParams_.enable_saturation;
}

/**
 * @brief Getter function for the pressure to depth conversion factor.
 * 
 * @return Pressure to depth conversion factor in Pa/mH2O.
*/
double DpthSensorSimulator::getPressureToMetreConversionFactor() const {
  return dpthSensorSimParams_.pressure_per_metre;
}

/**
 * @brief Getter function for the depth to pressure conversion factor.
 * 
 * @return Depth to pressure conversion factor in mH2O/Pa.
*/
double DpthSensorSimulator::getMetreToPressureConversionFactor() const {
  return 1.0 / dpthSensorSimParams_.pressure_per_metre;
}

/**
 * @brief Getter function for the sample time.
 * 
 * @return Sample time in seconds.
*/
double DpthSensorSimulator::getSampleTime() const {
  return dt_;
}

/**
 * @brief Getter function for the seed of the random number generator.
 * 
 * @return Seed of the random number generator.
*/
unsigned int DpthSensorSimulator::getSeed() const {
  return seed_;
}

/**
 * @brief Getter function for the enable fixed random numbers flag.
 * 
 * @return Enable fixed random numbers flag.
*/
bool DpthSensorSimulator::getEnableFixedRandomNumbers() const {
  return use_fixed_random_numbers_;
}

/**
 * @brief Setter function for the depth pressure sensor simulator parameters.
 * 
 * @param[in] dpthSensorSimParams Depth pressure sensor simulator parameters.
 * 
 * @return None
*/
void DpthSensorSimulator::setSimParams(
    const DpthSensorSimParams dpthSensorSimParams) {
  dpthSensorSimParams_ = dpthSensorSimParams;

  // Re-calculate the standard deviation of the pressure sensor
  pressure_standard_deviation_ = dpthSensorSimParams_.accuracy *
                                 dpthSensorSimParams_.range / 100.0;  // (Pa)
}

/**
 * @brief Setter function for the depth pressure sensor simulator enable parameters.
 * 
 * @param[in] dpthSensorSimEnableParams Depth pressure sensor simulator enable parameters.
 * 
 * @return None
*/
void DpthSensorSimulator::setSimEnableParams(
    const DpthSensorSimEnableParams dpthSensorSimEnableParams) {
  dpthSensorSimEnableParams_ = dpthSensorSimEnableParams;
}

/**
 * @brief Setter function for the white noise enable flag.
 * 
 * @param[in] enable White noise enable flag.
 * 
 * @return None
*/
void DpthSensorSimulator::setEnableWhiteNoise(const bool enable) {
  dpthSensorSimEnableParams_.enable_white_noise = enable;
}

/**
 * @brief Setter function for the static bias enable flag.
 * 
 * @param[in] enable Static bias enable flag.
 * 
 * @return None
*/
void DpthSensorSimulator::setEnableStaticBias(const bool enable) {
  dpthSensorSimEnableParams_.enable_static_bias = enable;
}

/**
 * @brief Setter function for the quantization enable flag.
 * 
 * @param[in] enable Quantization enable flag.
 * 
 * @return None
*/
void DpthSensorSimulator::setEnableQuantization(const bool enable) {
  dpthSensorSimEnableParams_.enable_quantization = enable;
}

/**
 * @brief Setter function for the saturation enable flag.
 * 
 * @param[in] enable Saturation enable flag.
 * 
 * @return None
*/
void DpthSensorSimulator::setEnableSaturation(const bool enable) {
  dpthSensorSimEnableParams_.enable_saturation = enable;
}

void DpthSensorSimulator::setPressureToMetreConversionFactor(
    const double conversion_factor) {
  dpthSensorSimParams_.pressure_per_metre = conversion_factor;
}

/**
 * @brief Setter function for the sample time.
 * 
 * @param[in] dt Sample time in seconds.
 * 
 * @return None
*/
void DpthSensorSimulator::setSampleTime(const double dt) {
  dt_ = dt;
}

/**
 * @brief Set the seed of the random number generator.
 *
 * This function sets the seed of the random number generator used by the
 * simulator.
 *
 * @param[in] seed The seed of the random number generator.
 *
 * @return None
 */
void DpthSensorSimulator::setSeed(const unsigned int seed) {
  randomNumberGenerator_.seed(seed_);
  seed_ = seed;
}

/**
 * @brief Setter function for the enable fixed random numbers flag.
 * 
 * @param[in] enable Enable fixed random numbers flag.
 * 
 * @return None
*/
void DpthSensorSimulator::setEnableFixedRandomNumbers(const bool enable) {
  use_fixed_random_numbers_ = enable;
}

std::stringstream DpthSensorSimulator::printSimulatorParameters() {
  // Create stringstream to store the output
  std::stringstream ss;

  ss << "***************************************************************"
        "********************************************************************"
        "**"
        "*"
     << "\n";
  ss << std::left << std::setw(50) << "Starting DEPTH Simulator"
     << "\n";
  ss << "***************************************************************"
        "********************************************************************"
        "**"
        "*"
     << "\n";

  // Depth sensor simulation parameters
  ss << std::left << "Simulation parameters:\n";

  ss << std::fixed << std::setprecision(6);

  ss << std::left << std::setw(50)
     << "accuracy:" << dpthSensorSimParams_.accuracy << " %\n";

  ss << std::left << std::setw(50)
     << "resolution:" << dpthSensorSimParams_.resolution << " %\n";

  ss << std::left << std::setw(50) << "range:" << dpthSensorSimParams_.range
     << " Pa\n";

  ss << std::left << std::setw(50)
     << "pressure_standard_deviation:" << pressure_standard_deviation_
     << " Pa\n";

  ss << std::left << std::setw(50)
     << "static_bias:" << dpthSensorSimParams_.static_bias << " Pa\n";

  ss << std::left << std::setw(50)
     << "pressure_per_metre:" << dpthSensorSimParams_.pressure_per_metre
     << " Pa/mH2O\n";

  ss << std::left << std::setw(50)
     << "p_bs_b:" << dpthSensorSimParams_.p_bs_b.transpose() << " m\n";

  ss << "***************************************************************"
        "********************************************************************"
        "**"
        "*"
     << "\n";

  // Depth sensor model enable settings
  ss << std::left << "Model enable settings:\n";

  ss << std::left << std::setw(50)
     << "enable_white_noise:" << dpthSensorSimEnableParams_.enable_white_noise
     << "\n";

  ss << std::left << std::setw(50)
     << "enable_static_bias:" << dpthSensorSimEnableParams_.enable_static_bias
     << "\n";

  ss << std::left << std::setw(50)
     << "enable_quantization:" << dpthSensorSimEnableParams_.enable_quantization
     << "\n";

  ss << std::left << std::setw(50)
     << "enable_saturation:" << dpthSensorSimEnableParams_.enable_saturation
     << "\n";

  ss << "\n";

  // Sampling time and frequency
  ss << std::left << std::setw(50) << "Sampling time:" << std::fixed
     << std::setprecision(6) << dt_ << " s\n";

  ss << std::left << std::setw(50) << "Sampling frequency:" << std::fixed
     << std::setprecision(6) << 1.0 / dt_ << " Hz\n";

  ss << "***************************************************************"
        "********************************************************************"
        "**"
        "*"
     << "\n";

  return ss;
}

/**
 * @brief Generates a noisy pressure measurement.
 *
 * This function adds white noise to the pressure measurement.
 * The standard deviation of the white noise is calculated from the accuracy of the pressure sensor.
 * 
 * @param[in] pressure_measurement Pressure measurement in Pa.
 *
 * @return Noisy pressure measurement in Pa.
  */
double DpthSensorSimulator::calcNoisyMeasurement(
    const double pressure_measurement) {
  // Define random number
  double normal_dist_num;

  if (use_fixed_random_numbers_) {
    // Draw a fixed random number for testing/debugging
    normal_dist_num = 3.0;
  } else {
    // Draw a random number from a normal distribution
    normal_dist_num = drawRandNormalDistNum();
  }

  // Add white noise to the pressure measurement
  double noisy_measurement =
      pressure_measurement + pressure_standard_deviation_ * normal_dist_num;

  return noisy_measurement;
}

/**
 * @brief Adds a static bias to the pressure measurement.
 *
 * This function adds a static bias to the pressure measurement.
 * This typically models the atmospheric pressure, ice pressure, or other static pressure sources.
 * 
 * @param[in] pressure_measurement Pressure measurement in Pa.
 *
 * @return Pressure measurement with static bias in Pa.
  */
double DpthSensorSimulator::calcStaticBiasModel(
    const double pressure_measurement) {
  // Add static bias to the pressure measurement
  double biased_measurement =
      pressure_measurement + dpthSensorSimParams_.static_bias;

  return biased_measurement;
}

/**
 * @brief Quantizes the pressure measurement.
 *
 * This function quantizes the pressure measurement to the resolution of the pressure sensor.
 * 
 * @param[in] pressure_measurement Pressure measurement in Pa.
 *
 * @return Quantized pressure measurement in Pa.
  */
double DpthSensorSimulator::calcQuantizationModel(
    const double pressure_measurement) {
  // Quantize the pressure measurement
  double quantized_measurement =
      std::round(pressure_measurement / dpthSensorSimParams_.resolution) *
      dpthSensorSimParams_.resolution;

  return quantized_measurement;
}

/**
 * @brief Saturates the pressure measurement.
 *
 * This function saturates the pressure measurement to the measurement range of the pressure sensor.
 * 
 * @param[in] pressure_measurement Pressure measurement in Pa.
 *
 * @return Saturated pressure measurement in Pa.
  */
double DpthSensorSimulator::calcSaturationModel(
    const double pressure_measurement) {
  // Saturate the pressure measurement
  double saturated_measurement =
      std::min(std::abs(pressure_measurement), dpthSensorSimParams_.range) *
      (pressure_measurement >= 0 ? 1 : -1);

  return saturated_measurement;
}

/**
 * @brief Converts a pressure value to a depth value.
 *
 * This function converts a pressure value to a depth value using the given conversion
 * factor.
 * 
 * @param[in] depth Depth in m.
 *
 * @return Pressure in Pa.
  */
double DpthSensorSimulator::convertDepthToPressure(const double depth) {
  return depth * dpthSensorSimParams_.pressure_per_metre;
}

/**
 * @brief Converts a depth value to a pressure value.
 *
 * This function converts a pressure value to a depth value using the given conversion
 * factor.
 * 
 * @param[in] pressure Pressure in Pa.
 *
 * @return Depth in m.
 */
double DpthSensorSimulator::convertPressureToDepth(const double pressure) {
  return pressure / dpthSensorSimParams_.pressure_per_metre;
}

/**
 * @brief Draws a random number from a normal distribution.
 * 
 * This function draws a random number from a normal distribution with the given mean and standard deviation.
 * 
 * @return Single random number from the normal distribution.
*/
double DpthSensorSimulator::drawRandNormalDistNum() {
  // Generate normal distributed random number
  return normalDistribution_(randomNumberGenerator_);
}

}  // namespace dpth_sensor_simulator

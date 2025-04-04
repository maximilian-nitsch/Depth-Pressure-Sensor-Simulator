/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#include "dpth_sensor_simulator.h"

#include "gtest/gtest.h"

/**
 * @brief Test fixture for the DEPTH sensor simulator.
 * 
 * This test fixture class sets up the DEPTH sensor simulator with default
 * parameters.
 * 
*/
class DpthSensorSimulatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dpthSensorSimParams_.accuracy = 0.05;       // (%)
    dpthSensorSimParams_.resolution = 0.005;    // (%)
    dpthSensorSimParams_.range = 5000000.0;     // (Pa)
    dpthSensorSimParams_.static_bias = 101325;  // (Pa); 1 atm
    dpthSensorSimParams_.pressure_per_metre =
        10110.8212387044;  // (Pa/mH2O); NM III station
    dpthSensorSimParams_.p_bs_b = Eigen::Vector3d::Zero();  // (m)

    // Initialize the depth pressure sensor simulator enable parameters
    dpthSensorSimEnableParams_.enable_white_noise = false;
    dpthSensorSimEnableParams_.enable_static_bias = false;
    dpthSensorSimEnableParams_.enable_quantization = false;
    dpthSensorSimEnableParams_.enable_saturation = false;

    // Create the class under test
    pDepthSensorSimulator =
        std::make_unique<dpth_sensor_simulator::DpthSensorSimulator>(
            dpthSensorSimParams_, dpthSensorSimEnableParams_, 0.5, 42, false);
  }
  // Simulation parameters DEPTH sensor
  dpth_sensor_simulator::DpthSensorSimParams dpthSensorSimParams_;

  // DEPTH sensor model enable settings
  dpth_sensor_simulator::DpthSensorSimEnableParams dpthSensorSimEnableParams_;

  // Declare the class under test
  std::unique_ptr<dpth_sensor_simulator::DpthSensorSimulator>
      pDepthSensorSimulator;
};

/**
 * @brief Test the measurement generation function without errors.
 *
 * This test checks if the measurement generation function returns the ground
 * truth depth without errors.
 *
 * @param[in] DpthSensorSimulatorTest The test fixture class.
 * @param[in] GenerateMeasurementWithoutErrorsTest The test name.
 *
 * @return None
 */
TEST_F(DpthSensorSimulatorTest, GenerateMeasurementWithoutErrorsTest) {
  // Generate a measurement
  dpth_sensor_simulator::DpthSensorMeasurement measurement =
      pDepthSensorSimulator->generateMeasurement(10.0);

  // No white noise should be added to the measurement
  EXPECT_NEAR(measurement.pressure,
              10.0 * dpthSensorSimParams_.pressure_per_metre, 1e-10);
  EXPECT_NEAR(measurement.depth, 10.0, 1e-10);
}

/**
 * @brief Test the white noise model.
 * 
 * This test checks if the white noise model is correctly enabled and disabled.
 * Furthermore, it is checked if a measurement is corrupted by white noise for a
 * given standard deviation.
 * 
 * @param[in] DpthSensorSimulatorTest The test fixture class.
 * @param[in] WhiteNoiseModelTest The test name.
 * 
 * @return None
 */
TEST_F(DpthSensorSimulatorTest, WhiteNoiseModelTest) {
  // Enable white noise model
  pDepthSensorSimulator->setEnableWhiteNoise(true);

  // Enable fixed random numbers
  pDepthSensorSimulator->setEnableFixedRandomNumbers(true);

  // Set standard deviation
  dpthSensorSimParams_.accuracy = 0.25;
  dpthSensorSimParams_.range = 2500000.0;

  // Set the new parameters
  pDepthSensorSimulator->setSimParams(dpthSensorSimParams_);

  // Generate a measurement
  dpth_sensor_simulator::DpthSensorMeasurement measurement =
      pDepthSensorSimulator->generateMeasurement(20.0);

  // Calculate the expected pressure
  double pressure_expected = 20.0 * dpthSensorSimParams_.pressure_per_metre +
                             3.0 * dpthSensorSimParams_.accuracy *
                                 dpthSensorSimParams_.range * 1 / 100;

  // Calculate the expected depth
  double depth_expected =
      pressure_expected / dpthSensorSimParams_.pressure_per_metre;

  // White noise should be added to the measurement
  EXPECT_NEAR(measurement.pressure, pressure_expected, 1e-10);
  EXPECT_NEAR(measurement.depth, depth_expected, 1e-10);
}

/**
 * @brief Test the static bias model.
 *
 * This test checks if the static bias model is correctly enabled and disabled.
 * Furthermore, it is checked if a measurement is corrupted by a given static bias.
 *
 * @param[in] DpthSensorSimulatorTest The test fixture class.
 * @param[in] StaticBiasTest The test name.
 *
 * @return None
 */
TEST_F(DpthSensorSimulatorTest, StaticBiasTest) {
  // Enable static bias model
  pDepthSensorSimulator->setEnableStaticBias(true);

  // Set static bias
  dpthSensorSimParams_.static_bias = 20000.0;

  // Set the new parameters
  pDepthSensorSimulator->setSimParams(dpthSensorSimParams_);

  // Generate a measurement
  dpth_sensor_simulator::DpthSensorMeasurement measurement =
      pDepthSensorSimulator->generateMeasurement(30.0);

  // Calculate the expected pressure
  double pressure_expected = 30.0 * dpthSensorSimParams_.pressure_per_metre +
                             dpthSensorSimParams_.static_bias;

  // Calculate the expected depth
  double depth_expected =
      pressure_expected / dpthSensorSimParams_.pressure_per_metre;

  // Static bias should be added to the measurement
  EXPECT_NEAR(measurement.pressure, pressure_expected, 1e-10);
  EXPECT_NEAR(measurement.depth, depth_expected, 1e-10);
}

/**
 * @brief Test the quantization model.
 *
 * This test checks if the quantization model is correctly enabled and disabled.
 * Furthermore, it is checked if a measurement is quantized to a given resolution.
 *
 * @param[in] DpthSensorSimulatorTest The test fixture class.
 * @param[in] QuantizationModelTest The test name.
 *
 * @return None
 */
TEST_F(DpthSensorSimulatorTest, QuantizationModelTest) {
  // Enable quantization model
  pDepthSensorSimulator->setEnableQuantization(true);

  // Set resolution
  dpthSensorSimParams_.resolution = 0.1;

  // Set the new parameters
  pDepthSensorSimulator->setSimParams(dpthSensorSimParams_);

  // Generate a measurement
  dpth_sensor_simulator::DpthSensorMeasurement measurement =
      pDepthSensorSimulator->generateMeasurement(42.12345);

  // Calculate the expected pressure
  double pressure_expected = 42.12345 * dpthSensorSimParams_.pressure_per_metre;
  pressure_expected =
      std::round(pressure_expected / dpthSensorSimParams_.resolution) *
      dpthSensorSimParams_.resolution;

  // Calculate the expected depth
  double depth_expected =
      pressure_expected / dpthSensorSimParams_.pressure_per_metre;

  // Measurement should be quantized
  EXPECT_NEAR(measurement.pressure, pressure_expected, 1e-10);
  EXPECT_NEAR(measurement.depth, depth_expected, 1e-10);
}

/**
 * @brief Test the saturation model.
 *
 * This test checks if the saturation model is correctly enabled and disabled.
 * Furthermore, it is checked if a measurement is saturated to a given range.
 *
 * @param[in] DpthSensorSimulatorTest The test fixture class.
 * @param[in] SaturationModelTest The test name.
 *
 * @return None
 */
TEST_F(DpthSensorSimulatorTest, SaturationModelTest) {
  // Enable saturation model
  pDepthSensorSimulator->setEnableSaturation(true);

  // Set range
  dpthSensorSimParams_.range = 125000.0;

  // Set the new parameters
  pDepthSensorSimulator->setSimParams(dpthSensorSimParams_);

  // Generate a measurement
  dpth_sensor_simulator::DpthSensorMeasurement measurement =
      pDepthSensorSimulator->generateMeasurement(100.0);

  // Calculate the expected pressure
  double pressure_expected = 100.0 * dpthSensorSimParams_.pressure_per_metre;
  pressure_expected =
      std::min(std::max(pressure_expected, -dpthSensorSimParams_.range),
               dpthSensorSimParams_.range);

  // Calculate the expected depth
  double depth_expected =
      pressure_expected / dpthSensorSimParams_.pressure_per_metre;

  // Measurement should no be saturated
  EXPECT_NEAR(measurement.pressure, pressure_expected, 1e-10);
  EXPECT_NEAR(measurement.depth, depth_expected, 1e-10);

  // Generate a measurement
  measurement = pDepthSensorSimulator->generateMeasurement(150.0);

  // Calculate the expected pressure
  pressure_expected = 150.0 * dpthSensorSimParams_.pressure_per_metre;
  pressure_expected =
      std::min(std::max(pressure_expected, -dpthSensorSimParams_.range),
               dpthSensorSimParams_.range);

  // Calculate the expected depth
  depth_expected = pressure_expected / dpthSensorSimParams_.pressure_per_metre;

  // Measurement should be saturated
  EXPECT_NEAR(measurement.pressure, pressure_expected, 1e-10);
  EXPECT_NEAR(measurement.depth, depth_expected, 1e-10);
}

/**
 * @brief Test the pressure to depth conversion function.
 *
 * This test checks if the pressure to depth conversion function is correctly
 * implemented.
 *
 * @param[in] DpthSensorSimulatorTest The test fixture class.
 * @param[in] PressureToDepthConversionTest The test name.
 *
 * @return None
 */
TEST_F(DpthSensorSimulatorTest, PressureToDepthConversionTest) {
  // Set the pressure to depth conversion factor
  pDepthSensorSimulator->setPressureToMetreConversionFactor(9806.38);

  // Generate a measurement
  dpth_sensor_simulator::DpthSensorMeasurement measurement =
      pDepthSensorSimulator->generateMeasurement(10.0);

  // Pressure to depth conversion factor should be applied
  EXPECT_NEAR(measurement.pressure, 10.0 * 9806.38, 1e-10);
  EXPECT_NEAR(measurement.depth, 10.0, 1e-10);

  // Get the depth to pressure conversion factor
  double depth_to_pressure_conversion_factor =
      pDepthSensorSimulator->getMetreToPressureConversionFactor();

  // Pressure to depth conversion factor should be returned
  EXPECT_NEAR(depth_to_pressure_conversion_factor, 1 / 9806.38, 1e-10);
}

/**
 * @brief Test the setter and getter functions.
 *
 * This test checks if the setter and getter functions set and return the
 * correct values.
 *
 * @param[in] DpthSensorSimulatorTest The test fixture class.
 * @param[in] SetterAndGetterFunctionsTests The test name.
 *
 * @return None
 */
TEST_F(DpthSensorSimulatorTest, SetterAndGetterFunctionsTests) {
  // Set the simulation parameters
  dpth_sensor_simulator::DpthSensorSimParams dpthSensorSimParams;
  dpthSensorSimParams.accuracy = 0.12345;
  dpthSensorSimParams.resolution = 0.012345;
  dpthSensorSimParams.range = 1234567.8;
  dpthSensorSimParams.static_bias = 123.45;
  dpthSensorSimParams.pressure_per_metre = 7500.0;
  dpthSensorSimParams.p_bs_b = Eigen::Vector3d(1.0, 2.0, 3.0);

  pDepthSensorSimulator->setSimParams(dpthSensorSimParams);

  // Get the simulation parameters
  dpth_sensor_simulator::DpthSensorSimParams dpthSensorSimParamsReturned =
      pDepthSensorSimulator->getSimParams();

  // Simulation parameters should be returned
  EXPECT_NEAR(dpthSensorSimParams.accuracy,
              dpthSensorSimParamsReturned.accuracy, 1e-10);
  EXPECT_NEAR(dpthSensorSimParams.resolution,
              dpthSensorSimParamsReturned.resolution, 1e-10);
  EXPECT_NEAR(dpthSensorSimParams.range, dpthSensorSimParamsReturned.range,
              1e-10);
  EXPECT_NEAR(dpthSensorSimParams.static_bias,
              dpthSensorSimParamsReturned.static_bias, 1e-10);
  EXPECT_NEAR(dpthSensorSimParams.pressure_per_metre,
              dpthSensorSimParamsReturned.pressure_per_metre, 1e-10);
  EXPECT_NEAR(dpthSensorSimParams.p_bs_b(0),
              dpthSensorSimParamsReturned.p_bs_b(0), 1e-10);
  EXPECT_NEAR(dpthSensorSimParams.p_bs_b(1),
              dpthSensorSimParamsReturned.p_bs_b(1), 1e-10);
  EXPECT_NEAR(dpthSensorSimParams.p_bs_b(2),
              dpthSensorSimParamsReturned.p_bs_b(2), 1e-10);

  // Set the simulation enable parameters to true
  dpth_sensor_simulator::DpthSensorSimEnableParams dpthSensorSimEnableParams;
  dpthSensorSimEnableParams.enable_white_noise = true;
  dpthSensorSimEnableParams.enable_static_bias = true;
  dpthSensorSimEnableParams.enable_quantization = true;
  dpthSensorSimEnableParams.enable_saturation = true;

  pDepthSensorSimulator->setSimEnableParams(dpthSensorSimEnableParams);

  // Get the simulation enable parameters
  dpth_sensor_simulator::DpthSensorSimEnableParams
      dpthSensorSimEnableParamsReturned =
          pDepthSensorSimulator->getSimEnableParams();

  // Simulation enable parameters should be returned
  EXPECT_EQ(dpthSensorSimEnableParams.enable_white_noise,
            dpthSensorSimEnableParamsReturned.enable_white_noise);
  EXPECT_EQ(dpthSensorSimEnableParams.enable_static_bias,
            dpthSensorSimEnableParamsReturned.enable_static_bias);
  EXPECT_EQ(dpthSensorSimEnableParams.enable_quantization,
            dpthSensorSimEnableParamsReturned.enable_quantization);
  EXPECT_EQ(dpthSensorSimEnableParams.enable_saturation,
            dpthSensorSimEnableParamsReturned.enable_saturation);

  // Set the simulation enable parameters to false
  pDepthSensorSimulator->setEnableWhiteNoise(false);
  pDepthSensorSimulator->setEnableStaticBias(false);
  pDepthSensorSimulator->setEnableQuantization(false);
  pDepthSensorSimulator->setEnableSaturation(false);

  // Get the simulation enable parameters
  dpthSensorSimEnableParamsReturned =
      pDepthSensorSimulator->getSimEnableParams();

  // Simulation enable parameters should be returned
  EXPECT_EQ(dpthSensorSimEnableParamsReturned.enable_white_noise, false);
  EXPECT_EQ(dpthSensorSimEnableParamsReturned.enable_static_bias, false);
  EXPECT_EQ(dpthSensorSimEnableParamsReturned.enable_quantization, false);
  EXPECT_EQ(dpthSensorSimEnableParamsReturned.enable_saturation, false);
}

/**
 * @brief Test the print function.
 *
 * This test checks if the print function returns a non-empty stringstream.
 *
 * @param[in] DpthSensorSimulatorTest The test fixture class.
 * @param[in] PrintFunctionTest The test name.
 *
 * @return None
 */
TEST_F(DpthSensorSimulatorTest, PrintFunctionTest) {
  // Print the simulator parameters to a stringstream
  std::stringstream ss = pDepthSensorSimulator->printSimulatorParameters();

  // Check if the stringstream is not empty
  EXPECT_FALSE(ss.str().empty());
}

/**
 * @brief Test the measurement generation function.
 *
 * This test checks if the measurement generation function returns a correct
 * measurement.
 *
 * @param[in] DpthSensorSimulatorTest The test fixture class.
 * @param[in] GenerateMeasurementTest The test name.
 *
 * @return None
 */
TEST_F(DpthSensorSimulatorTest, GenerateMeasurementTest) {
  // Enable white noise model
  pDepthSensorSimulator->setEnableWhiteNoise(true);

  // Enable static bias model
  pDepthSensorSimulator->setEnableStaticBias(true);

  // Enable quantization model
  pDepthSensorSimulator->setEnableQuantization(true);

  // Enable saturation model
  pDepthSensorSimulator->setEnableSaturation(true);

  // Enable fixed random numbers
  pDepthSensorSimulator->setEnableFixedRandomNumbers(true);

  // Generate a measurement
  dpth_sensor_simulator::DpthSensorMeasurement measurement =
      pDepthSensorSimulator->generateMeasurement(10.0);

  // Print the simulator
  std::cout << pDepthSensorSimulator->printSimulatorParameters().str()
            << std::endl;

  // Calculate the expected pressure
  double pressure_expected = 10.0 * dpthSensorSimParams_.pressure_per_metre;

  // Add white noise to the expected pressure
  pressure_expected += 3.0 * dpthSensorSimParams_.accuracy *
                       dpthSensorSimParams_.range * 1 / 100;

  // Add static bias to the expected pressure
  pressure_expected += dpthSensorSimParams_.static_bias;

  // Quantize the expected pressure
  pressure_expected =
      std::round(pressure_expected / dpthSensorSimParams_.resolution) *
      dpthSensorSimParams_.resolution;

  // Saturate the expected pressure
  pressure_expected =
      std::min(std::max(pressure_expected, -dpthSensorSimParams_.range),
               dpthSensorSimParams_.range);

  // Calculate the expected depth
  double depth_expected =
      pressure_expected / dpthSensorSimParams_.pressure_per_metre;

  // Measurement should be corrupted by all errors sources
  EXPECT_NEAR(measurement.pressure, pressure_expected, 1e-10);
  EXPECT_NEAR(measurement.depth, depth_expected, 1e-10);
}

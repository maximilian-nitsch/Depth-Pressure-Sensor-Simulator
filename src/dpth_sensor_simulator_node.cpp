#include <cstdio>

#include "rclcpp/rclcpp.hpp"

#include "dpth_sensor_simulator_node.h"

namespace dpth_sensor_simulator {

/**
 * @brief Constructor for the depth sensor simulator node.
 * 
 * This function is called when the depth sensor simulator node is created.
 * The parameters are retrieved from the YAML file and loaded into the simulator.
 * The static bias parameter is set on the parameter server.
 * The odometry subscriber, fluid pressure publisher, depth publisher, diagnostic
 * publisher, action server, and tf2 broadcaster are initialized.
 * The action server for calibrating the pressure sensor is initialized.
 * The tf2 broadcaster is initialized and the static tf2 transformations are published.
 * The timers for the simulator loop callback function and the odometry timeout
 * callback function are created.
 *
 * @param[in] pDpthSensorSimulator Pointer to the depth sensor simulator class
 * 
 * @return None
*/
DpthSensorSimulatorNode::DpthSensorSimulatorNode(
    const std::shared_ptr<DpthSensorSimulator> pDpthSensorSimulator)
    : Node("depth_sensor_simulator_node"),
      pDpthSensorSimulator_(pDpthSensorSimulator),
      groundTruthOdomMsg_(nullptr),
      lastOdomTimestamp_(0, 0),
      last_pressure_measurement_(0.0),
      sample_time_(0.5),
      first_odometry_received_(false),
      odometry_timeout_(false),
      node_namespace_(this->get_namespace()) {
  RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
              "Configuring depth sensor simulator node...");

  // Declare and retrieve parameters and load them into the simulator
  declareAndRetrieveGeneralSettings();
  declareAndRetrieveSimParams();
  declareAndRetrievEnableSettings();

  RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
              "Parameters from YAML config loaded successfully.");

  // Print simulator parameters
  std::stringstream ss = pDpthSensorSimulator_->printSimulatorParameters();
  RCLCPP_INFO(rclcpp::get_logger(node_namespace_), "%s", ss.str().c_str());

  // Declare the value of the topic_name parameter
  this->declare_parameter("topic_name", rclcpp::PARAMETER_STRING);

  // Retrieve topic name from launch file or use default
  std::string gt_topic_name_str;
  this->get_parameter_or("topic_name", gt_topic_name_str,
                         std::string("/auv/odometry"));

  // Set the pressure bias parameter on the parameter server
  this->declare_parameter(node_namespace_ + "/pressure_bias",
                          rclcpp::PARAMETER_DOUBLE);
  this->set_parameter(
      rclcpp::Parameter(node_namespace_ + "/pressure_bias", 0.0));

  RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
              "Subscribing to ground truth odometry topic: %s",
              gt_topic_name_str.c_str());

  // Initialize the odometry subscriber
  pOdometrySubscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      gt_topic_name_str, 10,
      std::bind(&DpthSensorSimulatorNode::odometryCallback, this,
                std::placeholders::_1));

  // Initialize the depth pressure publisher
  pDepthPressurePublisher_ = this->create_publisher<
      nanoauv_sensor_driver_interfaces::msg::DepthPressure>(
      node_namespace_ + "/depth_pressure", 10);

  // Initialize the action server for calibrating the pressure sensor
  calibratePressureSensorActionServer_ = rclcpp_action::create_server<
      nanoauv_sensor_driver_interfaces::action::CalibratePressureSensor>(
      this->get_node_base_interface(), this->get_node_clock_interface(),
      this->get_node_logging_interface(), this->get_node_waitables_interface(),
      "calibrate_pressure_sensor",
      std::bind(&DpthSensorSimulatorNode::handleGoal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&DpthSensorSimulatorNode::handleCancel, this,
                std::placeholders::_1),
      std::bind(&DpthSensorSimulatorNode::handleAccepted, this,
                std::placeholders::_1));

  // Initialize the tf2 broadcaster
  pStaticTf2Broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // Publish static tf2 transformations
  publishStaticTf2Transforms();

  // Extract the sample time from the simulator
  sample_time_ = pDpthSensorSimulator_->getSampleTime();

  // Convert sample time to milliseconds and cast to int
  int sampleTimeInt = static_cast<int>(sample_time_ * 1e3);

  RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
              "DEPTH simulator node executing with %dms.", sampleTimeInt);

  // Create a timer to call the DEPTH simulator loop callback function
  pTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(sampleTimeInt),
      std::bind(&DpthSensorSimulatorNode::simulatorLoopCallback, this));

  // Create timer for timeout
  pOdometryTimeOutTimer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&DpthSensorSimulatorNode::odometryTimeOutCallback, this));

  RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
              "DEPTH simulator node initialized.");

  RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
              "DEPTH simulator node waiting for first odometry message...");
}

/**
 * @brief Destructor for the depth sensor simulator node.
 * 
 * This function is called when the depth sensor simulator node is destroyed.
 * 
 * @param[in] None
 * 
 * @return None
*/
DpthSensorSimulatorNode::~DpthSensorSimulatorNode() {}

/**
 * @brief Declare and retrieve general settings from the YAML file.
 * 
 * This function declares and retrieves the general settings for the depth sensor simulator.
 * The general settings are retrieved from the YAML file and loaded into the simulator.
 * 
 * @param[in] None
 * 
 * @return None
*/
void DpthSensorSimulatorNode::declareAndRetrieveGeneralSettings() {
  // Define parameters to be retrieved
  double dt;
  unsigned int seed;
  bool use_constant_seed;

  // Preceding hierarchies of YAML file
  std::string prev_hierarchies =
      "depth_pressure_sensor_simulator.general_settings.";

  // Declare parameters
  this->declare_parameter(prev_hierarchies + "sample_time",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(prev_hierarchies + "seed", rclcpp::PARAMETER_INTEGER);
  this->declare_parameter(prev_hierarchies + "use_constant_seed",
                          rclcpp::PARAMETER_BOOL);

  // Retrieve parameters
  dt = this->get_parameter(prev_hierarchies + "sample_time").as_double();
  seed = this->get_parameter(prev_hierarchies + "seed").as_int();
  use_constant_seed =
      this->get_parameter(prev_hierarchies + "use_constant_seed").as_bool();

  // Set parameters in the simulator
  pDpthSensorSimulator_->setSampleTime(dt);
  pDpthSensorSimulator_->setSeed(seed);
  pDpthSensorSimulator_->setEnableFixedRandomNumbers(use_constant_seed);

  // Set seed depending on the use_constant_seed flag
  if (use_constant_seed == false) {
    // Draw a random seed from the random device
    std::random_device rd;
    seed = rd();
    pDpthSensorSimulator_->setSeed(seed);
    RCLCPP_INFO(rclcpp::get_logger(node_namespace_), "Using random seed: %d",
                seed);
  } else {
    // Set the random number generator seed
    pDpthSensorSimulator_->setSeed(seed);
    RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
                "Using seed from config file: %d", seed);
  }
}

/**
 * @brief Declare and retrieve model parameters from the YAML file.
 * 
 * This function declares and retrieves the model parameters for the depth sensor simulator.
 * The model parameters are retrieved from the YAML file and loaded into the simulator.
 * 
 * @param[in] None
 * 
 * @return None
*/
void DpthSensorSimulatorNode::declareAndRetrieveSimParams() {
  // Define depth sensor parameters to be retrieved
  DpthSensorSimParams dpthSensorSimParams;

  // Preceding hierarchies of YAML file
  std::string prev_hierarchies =
      "depth_pressure_sensor_simulator.model_parameter_settings.";

  // Declare parameters
  this->declare_parameter(prev_hierarchies + "accuracy",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(prev_hierarchies + "resolution",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(prev_hierarchies + "range", rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(prev_hierarchies + "static_bias",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(prev_hierarchies + "pressure_per_metre",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter(prev_hierarchies + "lever_arm_body_to_sensor",
                          rclcpp::PARAMETER_DOUBLE_ARRAY);

  // Retrieve parameters
  dpthSensorSimParams.accuracy =
      this->get_parameter(prev_hierarchies + "accuracy").as_double();
  dpthSensorSimParams.resolution =
      this->get_parameter(prev_hierarchies + "resolution").as_double();
  dpthSensorSimParams.range =
      this->get_parameter(prev_hierarchies + "range").as_double();
  dpthSensorSimParams.static_bias =
      this->get_parameter(prev_hierarchies + "static_bias").as_double();
  dpthSensorSimParams.pressure_per_metre =
      this->get_parameter(prev_hierarchies + "pressure_per_metre").as_double();
  std::vector<double> p_bs_b =
      this->get_parameter(prev_hierarchies + "lever_arm_body_to_sensor")
          .as_double_array();
  dpthSensorSimParams.p_bs_b << p_bs_b[0], p_bs_b[1], p_bs_b[2];

  // Set parameters in the simulator
  pDpthSensorSimulator_->setSimParams(dpthSensorSimParams);
}

/**
 * @brief Declare and retrieve enable settings from the YAML file.
 * 
 * This function declares and retrieves the enable settings for the depth sensor simulator.
 * The enable settings are retrieved from the YAML file and loaded into the simulator.
 * 
 * @param[in] None
 * 
 * @return None
*/
void DpthSensorSimulatorNode::declareAndRetrievEnableSettings() {
  // Define depth sensor enable parameters to be retrieved
  DpthSensorSimEnableParams dpthSensorSimEnableParams;

  // Preceding hierarchies of YAML file
  std::string prev_hierarchies =
      "depth_pressure_sensor_simulator.model_enable_settings.";

  // Declare parameters
  this->declare_parameter(prev_hierarchies + "enable_white_noise",
                          rclcpp::PARAMETER_BOOL);
  this->declare_parameter(prev_hierarchies + "enable_static_bias",
                          rclcpp::PARAMETER_BOOL);
  this->declare_parameter(prev_hierarchies + "enable_quantization",
                          rclcpp::PARAMETER_BOOL);
  this->declare_parameter(prev_hierarchies + "enable_saturation",
                          rclcpp::PARAMETER_BOOL);

  // Retrieve parameters
  dpthSensorSimEnableParams.enable_white_noise =
      this->get_parameter(prev_hierarchies + "enable_white_noise").as_bool();
  dpthSensorSimEnableParams.enable_static_bias =
      this->get_parameter(prev_hierarchies + "enable_static_bias").as_bool();
  dpthSensorSimEnableParams.enable_quantization =
      this->get_parameter(prev_hierarchies + "enable_quantization").as_bool();
  dpthSensorSimEnableParams.enable_saturation =
      this->get_parameter(prev_hierarchies + "enable_saturation").as_bool();

  // Set parameters in the simulator
  pDpthSensorSimulator_->setSimEnableParams(dpthSensorSimEnableParams);
}

/**
 * @brief Simulator loop callback function.
 * 
 * This function is called when the simulator loop timer is triggered. The
 * depth sensor simulator generates a depth measurement and publishes the
 * custom depth pressure messages.
 * 
 * @param[in] None
 * 
 * @return None
 */
void DpthSensorSimulatorNode::simulatorLoopCallback() {
  // Get current timestamp
  rclcpp::Time currentTimestamp = now();

  // Create ROS messages to be published
  nanoauv_sensor_driver_interfaces::msg::DepthPressure depthPressureMsg;
  sensor_msgs::msg::FluidPressure fluidPressureMsg;
  diagnostic_msgs::msg::DiagnosticStatus diagnosticMsg;
  diagnostic_msgs::msg::DiagnosticArray diagnosticArrayMsg;

  // Initialize the custom depth pressure message
  depthPressureMsg.header.stamp = currentTimestamp;
  depthPressureMsg.header.frame_id = "world_ned";

  depthPressureMsg.pressure.fluid_pressure = 0.0;
  depthPressureMsg.depth.data = 0.0;
  depthPressureMsg.temperature.temperature =
      9999999.0;  // Initialize with invalid value

  depthPressureMsg.is_valid.data = false;

  // Check if ground truth odometry message is available
  if (groundTruthOdomMsg_ == nullptr) {
    // Print STALE diagnostic message when no ground truth odometry message
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    diagnosticMsg.name = "DEPTH simulator";
    diagnosticMsg.message = "Waiting for first ground truth odometry message!";

    // Add diagnostic message to diagnostic array message
    diagnosticArrayMsg.status.push_back(diagnosticMsg);
    diagnosticArrayMsg.header.stamp = currentTimestamp;

    // Add diagnostic message to diagnostic array message
    diagnosticArrayMsg.status.push_back(diagnosticMsg);
    diagnosticArrayMsg.header.stamp = currentTimestamp;

    // Add diagnostic array message to custom depth pressure message
    depthPressureMsg.diagnostic_array = diagnosticArrayMsg;

    // Publish the custom depth pressure message
    pDepthPressurePublisher_->publish(depthPressureMsg);

    // Reset odometry timeout timer since waiting for first message
    pOdometryTimeOutTimer_->cancel();
    pOdometryTimeOutTimer_->reset();

    return;
  } else {
    // Extract ground truth position from odometry message
    Eigen::Vector3d p_nb_n_true;

    p_nb_n_true.x() = groundTruthOdomMsg_.get()->pose.pose.position.x;
    p_nb_n_true.y() = groundTruthOdomMsg_.get()->pose.pose.position.y;
    p_nb_n_true.z() = groundTruthOdomMsg_.get()->pose.pose.position.z;

    // Generate depth measurement
    DpthSensorMeasurement dpthSensorMeasurement =
        pDpthSensorSimulator_->generateMeasurement(p_nb_n_true.z());

    // Scope for the mutex lock
    {
      // Lock the last_pressure_measurement_ with mutex
      std::lock_guard<std::mutex> lock(last_pressure_measurement_mutex_);

      // Assign the pressure measurement to internal variable
      last_pressure_measurement_ = dpthSensorMeasurement.pressure;
    }

    // Calibrate pressure sensor with static bias from parameter server
    double pressure_bias;

    // Get the pressure bias parameter from the parameter server
    this->get_parameter(node_namespace_ + "/pressure_bias", pressure_bias);

    // Remove the static bias from the pressure measurement
    dpthSensorMeasurement.pressure -= pressure_bias;
    dpthSensorMeasurement.depth -=
        pressure_bias /
        pDpthSensorSimulator_->getSimParams().pressure_per_metre;
    // Fill the fluid pressure message
    fluidPressureMsg.header.stamp = currentTimestamp;
    fluidPressureMsg.header.frame_id = "depth_sensor_link";
    fluidPressureMsg.fluid_pressure = dpthSensorMeasurement.pressure;
    fluidPressureMsg.variance = dpthSensorMeasurement.pressure_std_dev *
                                dpthSensorMeasurement.pressure_std_dev;

    // Fill the custom depth pressure message
    depthPressureMsg.pressure = fluidPressureMsg;

    // Fill the custom depth pressure message
    depthPressureMsg.depth.data = dpthSensorMeasurement.depth;

    // Fill the custom depth pressure message
    depthPressureMsg.is_valid.data = true;

    // Fill the diagnostic message
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnosticMsg.name = "DEPTH simulator";
    diagnosticMsg.message = "Depth pressure sensor simulator running nominal.";
  }

  // Calculate time since last odometry message
  rclcpp::Duration timeSinceLastOdom =
      rclcpp::Duration(currentTimestamp - lastOdomTimestamp_);

  // Check if the ground truth odometry message frequency is too slow
  if (timeSinceLastOdom.seconds() > sample_time_ &&
      odometry_timeout_ == false) {
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diagnosticMsg.name = "DEPTH Simulator";
    diagnosticMsg.message =
        "Ground truth odometry message frequency is too slow!"
        " DEPTH simulator ground truth frequency higher than odometry!"
        " Increase odometry message frequency!";

    RCLCPP_WARN(rclcpp::get_logger(node_namespace_),
                "Ground truth odometry message frequency is too slow!");
  }

  // Check if there is a timeout in the ground truth odometry message
  if (odometry_timeout_) {
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    diagnosticMsg.name = "DEPTH simulator";
    diagnosticMsg.message =
        "No ground truth odometry message received since than 5 seconds!"
        " DEPTH simulator stalling!";

    RCLCPP_WARN(rclcpp::get_logger(node_namespace_),
                "No ground truth odometry message since more than 5 seconds!");
  }

  // Add diagnostic message to diagnostic array message
  diagnosticArrayMsg.status.push_back(diagnosticMsg);
  diagnosticArrayMsg.header.stamp = currentTimestamp;

  // Add diagnostic array message to custom depth pressure message
  depthPressureMsg.diagnostic_array = diagnosticArrayMsg;

  // Publish the custom depth pressure message
  pDepthPressurePublisher_->publish(depthPressureMsg);
}

/**
 * @brief Odometry callback function.
 * 
 * This function is called when a new odometry message is received. The ground
 * truth odometry message is assigned to the ground truth odometry message
 * member variable of the node class.
 * 
 * @param[in] msg Pointer to the odometry message
 * 
 * @return None
*/
void DpthSensorSimulatorNode::odometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Reset odometry timeout timer
  pOdometryTimeOutTimer_->cancel();
  pOdometryTimeOutTimer_->reset();

  // Set first odometry received flag
  if (first_odometry_received_ == false) {
    first_odometry_received_ = true;

    RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
                "First ground truth odometry message received! DEPTH simulator "
                "now running nominal!");
  }
  // Reset odometry timeout flag
  if (odometry_timeout_ == true) {
    odometry_timeout_ = false;

    RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
                "Ground truth odometry message received after timeout! DEPTH "
                "simulator now running nominal!");
  }

  // Assign ground truth odometry message
  groundTruthOdomMsg_ = msg;

  // Assign last odometry timestamp
  lastOdomTimestamp_ = msg->header.stamp;
}

/**
 * @brief Odometry timeout callback function.
 * 
 * This function is called when no ground truth odometry message is received.
 * 
 * @param[in] None
 * 
 * @return None
*/
void DpthSensorSimulatorNode::odometryTimeOutCallback() {
  // Set odometry timeout flag
  odometry_timeout_ = true;

  RCLCPP_WARN(rclcpp::get_logger(node_namespace_),
              "No ground truth odometry message since more than 5 "
              "seconds! DEPTH simulator now starting to stale!");
}

/**
 * @brief Publish static tf2 transformations.
 * 
 * This function publishes the static tf2 transformations between the base_link
 * and the depth sensor link.
 * 
 * @param[in] None
 * 
 * @return None
*/
void DpthSensorSimulatorNode::publishStaticTf2Transforms() {
  // Get USBL simulator parameters
  DpthSensorSimParams dpthSensorSimParams =
      pDpthSensorSimulator_->getSimParams();

  // Fill tf2 transform message between base_link_sname and dvl_link
  geometry_msgs::msg::TransformStamped tfMsg;
  tfMsg.header.stamp = now();

  tfMsg.header.frame_id = "base_link_sname";
  tfMsg.child_frame_id = "depth_sensor_link";

  tfMsg.transform.translation.x = dpthSensorSimParams.p_bs_b[0];
  tfMsg.transform.translation.y = dpthSensorSimParams.p_bs_b[1];
  tfMsg.transform.translation.z = dpthSensorSimParams.p_bs_b[2];

  tfMsg.transform.rotation.w = 1.0;
  tfMsg.transform.rotation.x = 0.0;
  tfMsg.transform.rotation.y = 0.0;
  tfMsg.transform.rotation.z = 0.0;

  pStaticTf2Broadcaster_->sendTransform(tfMsg);
}

/**
 * @brief Goal callback function for the calibrate pressure sensor action server.
 * 
 * This function is called when a new goal is received for the calibrate pressure
 * sensor action server. The goal is accepted and executed.
 * 
 * @param[in] uuid UUID of the goal
 * @param[in] pGoal Pointer to the goal message
 * 
 * @return Goal response
*/
rclcpp_action::GoalResponse DpthSensorSimulatorNode::handleGoal(
    [[maybe_unused]] const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const nanoauv_sensor_driver_interfaces::action::
                        CalibratePressureSensor::Goal>
        pGoal) {
  RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
              "Received goal request with order %d seconds",
              pGoal->calibration_duration);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/**
 * @brief Cancel callback function for the calibrate pressure sensor action server.
 * 
 * This function is called when a cancel request is received for the calibrate
 * pressure sensor action server. The goal is canceled.
 * 
 * @param[in] pGoalHandle Pointer to the goal handle
 * 
 * @return Cancel response
*/
rclcpp_action::CancelResponse DpthSensorSimulatorNode::handleCancel(
    [[maybe_unused]] const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        nanoauv_sensor_driver_interfaces::action::CalibratePressureSensor>>
        pGoalHandle) {
  RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
              "Received request to cancel goal.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

/**
 * @brief Accepted callback function for the calibrate pressure sensor action server.
 * 
 * This function is called when a new goal is accepted for the calibrate pressure
 * sensor action server. The goal is executed in a new thread.
 * 
 * @param[in] pGoalHandle Pointer to the goal handle
 * 
 * @return None
*/
void DpthSensorSimulatorNode::handleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        nanoauv_sensor_driver_interfaces::action::CalibratePressureSensor>>
        pGoalHandle) {
  RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
              "Received new calibration request.");

  // Create a lambda function to execute the action in a new thread
  auto execute_in_thread = [this, pGoalHandle]() {
    return this->executeCalibratePressureSensorAction(pGoalHandle);
  };

  // Detach the lambda function to execute the action in a new thread
  std::thread{execute_in_thread}.detach();
}

/**
 * @brief Execute callback function for the calibrate pressure sensor action server.
 * 
 * This function is called when a new goal is executed for the calibrate pressure
 * sensor action server. The depth pressure sensor is calibrated for the duration
 * specified in the goal message. The average pressure is calculated and published
 * as feedback. The calibrated pressure is published as result.
 * 
 * @param[in] pGoalHandle Pointer to the goal handle
 * 
 * @return None
*/
void DpthSensorSimulatorNode::executeCalibratePressureSensorAction(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        nanoauv_sensor_driver_interfaces::action::CalibratePressureSensor>>
        pGoalHandle) {
  RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
              "Executing depth pressure sensor calibration action...");

  // Create a loop rate object to sleep for the sample time
  rclcpp::Rate loopRate(sample_time_);

  // Create a double variable to store the average pressure
  double average_pressure = 0.0;

  // Create feedback message
  auto feedback = std::make_shared<nanoauv_sensor_driver_interfaces::action::
                                       CalibratePressureSensor::Feedback>();

  // Create result message
  auto result = std::make_shared<nanoauv_sensor_driver_interfaces::action::
                                     CalibratePressureSensor::Result>();

  // Get the start time of the calibration action
  rclcpp::Time startTime = rclcpp::Time(rclcpp::Clock().now());

  // Declare a variable to store the calibration time
  builtin_interfaces::msg::Duration calibrationDuration;

  // Get the calibration time from the action goal
  calibrationDuration.sec = pGoalHandle->get_goal()->calibration_duration;

  // Print the calibration time
  RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
              "Calibrating depth pressure sensor for %d seconds",
              calibrationDuration.sec);

  // Create a loop that will run for the duration of the calibration time
  while (rclcpp::Time(rclcpp::Clock().now()) <
             startTime + calibrationDuration &&
         rclcpp::ok()) {

    // Check if the action is being canceled
    if (pGoalHandle->is_canceling()) {
      // Cancel the action
      pGoalHandle->canceled(result);

      RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
                  "Depth pressure sensor calibration canceled!");

      // Clear the pressure data vector
      pressure_data_.clear();

      return;
    }

    // // Scope for the mutex lock
    // {
    //   // Lock the last_pressure_measurement_ with mutex
    //   std::lock_guard<std::mutex> lock(last_pressure_measurement_mutex_);

    // Add the last pressure measurement to the pressure data vector
    pressure_data_.push_back(last_pressure_measurement_);
    // }

    // Average pressure values in vector
    if (pressure_data_.size() > 0) {
      for (size_t i = 0; i < pressure_data_.size(); i++) {
        average_pressure += pressure_data_[i];
      }
      average_pressure = average_pressure / pressure_data_.size();
    }

    // Publish the feedback
    feedback->average_pressure = average_pressure;
    pGoalHandle->publish_feedback(feedback);

    // Print the average pressure
    RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
                "Average pressure: %.2f Pa. Corresponding depth: %.2f m.",
                average_pressure,
                average_pressure /
                    pDpthSensorSimulator_->getSimParams().pressure_per_metre);

    // Sleep for the sample time
    loopRate.sleep();
  }

  // Give the result when action is finished
  if (rclcpp::ok()) {
    result->calibrated_pressure = average_pressure;

    RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
                "Depth pressure sensor calibration finished!");

    RCLCPP_INFO(rclcpp::get_logger(node_namespace_),
                "Calibrated pressure: %.2f Pa. Corresponding depth: %.2f m.",
                average_pressure,
                average_pressure /
                    pDpthSensorSimulator_->getSimParams().pressure_per_metre);

    // Set the pressure bias parameter on the parameter server
    this->set_parameter(rclcpp::Parameter(node_namespace_ + "/pressure_bias",
                                          average_pressure));

    pGoalHandle->succeed(result);
  }

  // Clear the pressure data vector
  pressure_data_.clear();
}

}  // namespace dpth_sensor_simulator

/**
 * @brief Main function for the depth sensor simulator node.
 * 
 * This function is the entry point for the depth sensor simulator node.
 * 
 * @param[in] argc Number of command line arguments
 * @param[in] argv Pointer to the command line arguments
 * 
 * @return 0
*/
int main(int argc, char** argv) {
  // Create depth pressure sensor simulator class instance
  std::shared_ptr<dpth_sensor_simulator::DpthSensorSimulator>
      pDpthSensorSimulator =
          std::make_shared<dpth_sensor_simulator::DpthSensorSimulator>();

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<dpth_sensor_simulator::DpthSensorSimulatorNode>(
      pDpthSensorSimulator));

  rclcpp::shutdown();

  return 0;
}

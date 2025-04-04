/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#pragma once

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <tf2_ros/static_transform_broadcaster.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

#include "nanoauv_sensor_driver_interfaces/action/calibrate_pressure_sensor.hpp"
#include "nanoauv_sensor_driver_interfaces/msg/depth_pressure.hpp"

#include "dpth_sensor_simulator.h"

namespace dpth_sensor_simulator {

class DpthSensorSimulatorNode : public rclcpp::Node {
 public:
  // Constructor
  explicit DpthSensorSimulatorNode(
      const std::shared_ptr<DpthSensorSimulator> pDpthSensorSimulator);

  // Destructor
  ~DpthSensorSimulatorNode();

 private:
  // Depth pressure sensor simulator class object
  std::shared_ptr<DpthSensorSimulator> pDpthSensorSimulator_;

  // Ground truth odometry message
  nav_msgs::msg::Odometry::SharedPtr groundTruthOdomMsg_;

  // Node publisher
  //   rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr
  //       pFluidPressurePublisher_;
  //   rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
  //       pDepthPublisher_;
  //   rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
  //       pDiagnosticPublisher_;

  rclcpp::Publisher<nanoauv_sensor_driver_interfaces::msg::DepthPressure>::
      SharedPtr pDepthPressurePublisher_;

  // Vehicle odometry subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pOdometrySubscriber_;

  // Static tf2 broadcaster
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> pStaticTf2Broadcaster_;

  // Action server for calibrating the pressure sensor
  rclcpp_action::Server<
      nanoauv_sensor_driver_interfaces::action::CalibratePressureSensor>::
      SharedPtr calibratePressureSensorActionServer_;

  // Timers
  rclcpp::TimerBase::SharedPtr pTimer_;
  rclcpp::TimerBase::SharedPtr pOdometryTimeOutTimer_;

  // Last odometry timestamp
  rclcpp::Time lastOdomTimestamp_;

  // Last pressure sensor measurement
  double last_pressure_measurement_;

  // Pressure data vector for calibration
  std::vector<double> pressure_data_;

  // Sample time
  double sample_time_;

  // Odometry flags
  bool first_odometry_received_;
  bool odometry_timeout_;

  // Mutex for pressure data vector
  std::mutex last_pressure_measurement_mutex_;

  // Node namespace
  std::string node_namespace_;

  // Declaration and retrieval for parameters from YAML file
  void declareAndRetrieveGeneralSettings();
  void declareAndRetrieveSimParams();
  void declareAndRetrievEnableSettings();

  // Timer callback function
  void simulatorLoopCallback();

  // Odometry callback function
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr pOdometryMsg);
  void odometryTimeOutCallback();

  // tf2 static broadcaster callback function
  void publishStaticTf2Transforms();

  // Action server callback functions
  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const nanoauv_sensor_driver_interfaces::action::
                          CalibratePressureSensor::Goal>
          pGoal);

  rclcpp_action::CancelResponse handleCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<
          nanoauv_sensor_driver_interfaces::action::CalibratePressureSensor>>
          pGoalHandle);

  void handleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<
          nanoauv_sensor_driver_interfaces::action::CalibratePressureSensor>>
          pGoalHandle);

  void executeCalibratePressureSensorAction(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<
          nanoauv_sensor_driver_interfaces::action::CalibratePressureSensor>>
          pGoalHandle);
};

}  // namespace dpth_sensor_simulator

// Copyright 2026
// Apache License, Version 2.0

#ifndef ROBSTRIDE_HARDWARE_INTERFACE__ROBSTRIDE_HARDWARE_INTERFACE_HPP_
#define ROBSTRIDE_HARDWARE_INTERFACE__ROBSTRIDE_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "robstride_sdk/can_bus.hpp"
#include "robstride_sdk/robstride_motor.hpp"
#include "robstride_sdk/robstride_protocol.hpp"

#include "robstride_interfaces/msg/robstride_state.hpp"
#include "robstride_interfaces/srv/get_data_from_robstride.hpp"
#include "robstride_interfaces/srv/set_data_to_robstride.hpp"
#include "robstride_interfaces/srv/set_zero_robstride.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "realtime_tools/realtime_publisher.hpp"

namespace robstride_hardware_interface
{

struct JointHandle
{
  std::string joint_name;
  uint8_t id;
  std::string can_interface;
  robstride_sdk::ActuatorType actuator_type;
  double kp;
  double kd;
  robstride_sdk::RobstrideMotor * motor;  // owned by RobstrideHardware::motors_
};

/// @brief ros2_control SystemInterface for RobStride actuators over
/// SocketCAN. Owns one robstride_sdk::CanBus per distinct `can_interface`
/// used across the hardware's joints, so multi-bus setups (e.g. one bus per
/// quadruped leg) work by giving joints different `can_interface` params.
class RobstrideHardware : public hardware_interface::SystemInterface, public rclcpp::Node
{
public:
  RobstrideHardware();
  ~RobstrideHardware() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  rclcpp::Logger logger_;

  std::vector<JointHandle> joints_;
  std::unordered_map<std::string, std::unique_ptr<robstride_sdk::CanBus>> buses_;
  std::vector<std::unique_ptr<robstride_sdk::RobstrideMotor>> motors_;

  std::vector<double> hw_state_position_;
  std::vector<double> hw_state_velocity_;
  std::vector<double> hw_state_effort_;
  std::vector<double> hw_cmd_position_;
  std::vector<double> hw_cmd_velocity_;
  std::vector<double> hw_cmd_effort_;

  uint8_t master_id_ = 0xFD;
  double error_timeout_ms_ = 4000.0;

  using RobstrideStateMsg = robstride_interfaces::msg::RobstrideState;
  using StatePublisher = realtime_tools::RealtimePublisher<RobstrideStateMsg>;
  rclcpp::Publisher<RobstrideStateMsg>::SharedPtr state_pub_;
  std::unique_ptr<StatePublisher> state_pub_uni_ptr_;

  rclcpp::Service<robstride_interfaces::srv::GetDataFromRobstride>::SharedPtr get_data_srv_;
  rclcpp::Service<robstride_interfaces::srv::SetDataToRobstride>::SharedPtr set_data_srv_;
  rclcpp::Service<robstride_interfaces::srv::SetZeroRobstride>::SharedPtr set_zero_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_torque_srv_;

  void GetDataCallback(
    const std::shared_ptr<robstride_interfaces::srv::GetDataFromRobstride::Request> request,
    std::shared_ptr<robstride_interfaces::srv::GetDataFromRobstride::Response> response);
  void SetDataCallback(
    const std::shared_ptr<robstride_interfaces::srv::SetDataToRobstride::Request> request,
    std::shared_ptr<robstride_interfaces::srv::SetDataToRobstride::Response> response);
  void SetZeroCallback(
    const std::shared_ptr<robstride_interfaces::srv::SetZeroRobstride::Request> request,
    std::shared_ptr<robstride_interfaces::srv::SetZeroRobstride::Response> response);
  void SetTorqueCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  JointHandle * FindJointById(uint8_t id);
};

}  // namespace robstride_hardware_interface

#endif  // ROBSTRIDE_HARDWARE_INTERFACE__ROBSTRIDE_HARDWARE_INTERFACE_HPP_

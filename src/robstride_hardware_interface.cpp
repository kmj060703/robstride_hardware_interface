// Copyright 2026
// Apache License, Version 2.0

#include "robstride_hardware_interface/robstride_hardware_interface.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <set>
#include <thread>

namespace robstride_hardware_interface
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

RobstrideHardware::RobstrideHardware()
: rclcpp::Node("robstride_hardware_interface"),
  logger_(rclcpp::get_logger("robstride_hardware_interface"))
{
}

RobstrideHardware::~RobstrideHardware()
{
  for (auto & kv : buses_) {
    kv.second->Stop();
    kv.second->Close();
  }
}

namespace
{
std::string GetJointParam(
  const hardware_interface::ComponentInfo & joint, const std::string & key,
  const std::string & default_value = "")
{
  const auto it = joint.parameters.find(key);
  return it == joint.parameters.end() ? default_value : it->second;
}

// now_ns and last_ns are independent steady_clock snapshots taken from
// different threads (this control-loop call vs. the CAN read thread), so
// last_ns can legitimately land a few ns *after* now_ns if a fresh frame
// arrives mid-call. A plain unsigned subtraction would wrap around to a
// huge value in that case and falsely report staleness -- clamp to 0
// instead.
double ElapsedMsSinceUpdate(uint64_t now_ns, uint64_t last_ns)
{
  if (last_ns == 0) {return -1.0;}
  if (last_ns >= now_ns) {return 0.0;}
  return static_cast<double>(now_ns - last_ns) / 1.0e6;
}
}  // namespace

CallbackReturn RobstrideHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    if (info_.hardware_parameters.count("master_id")) {
      master_id_ = static_cast<uint8_t>(std::stoi(info_.hardware_parameters.at("master_id")));
    }
    if (info_.hardware_parameters.count("error_timeout_ms")) {
      error_timeout_ms_ = std::stod(info_.hardware_parameters.at("error_timeout_ms"));
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(
      logger_, "Malformed hardware parameter (master_id/error_timeout_ms): " << e.what());
    return CallbackReturn::ERROR;
  }

  const size_t n = info_.joints.size();
  joints_.resize(n);
  hw_state_position_.assign(n, 0.0);
  hw_state_velocity_.assign(n, 0.0);
  hw_state_effort_.assign(n, 0.0);
  hw_cmd_position_.assign(n, 0.0);
  hw_cmd_velocity_.assign(n, 0.0);
  hw_cmd_effort_.assign(n, 0.0);

  motors_.reserve(n);

  for (size_t i = 0; i < n; ++i) {
    const hardware_interface::ComponentInfo & joint = info_.joints[i];

    const std::string id_str = GetJointParam(joint, "id");
    const std::string type_str = GetJointParam(joint, "actuator_type");
    const std::string can_if = GetJointParam(joint, "can_interface");
    if (id_str.empty() || type_str.empty() || can_if.empty()) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Joint '" << joint.name <<
          "' is missing a required param (id / actuator_type / can_interface)");
      return CallbackReturn::ERROR;
    }

    JointHandle & jh = joints_[i];
    jh.joint_name = joint.name;
    try {
      jh.id = static_cast<uint8_t>(std::stoi(id_str));
      jh.actuator_type = robstride_sdk::ActuatorTypeFromString(type_str);
      jh.kp = std::stod(GetJointParam(joint, "kp", "0.0"));
      jh.kd = std::stod(GetJointParam(joint, "kd", "0.0"));
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(logger_, "Joint '" << joint.name << "': " << e.what());
      return CallbackReturn::ERROR;
    }
    jh.can_interface = can_if;

    motors_.push_back(
      std::make_unique<robstride_sdk::RobstrideMotor>(jh.id, master_id_, jh.actuator_type));
    jh.motor = motors_.back().get();

    if (buses_.find(can_if) == buses_.end()) {
      buses_[can_if] = std::make_unique<robstride_sdk::CanBus>(can_if);
    }
    buses_[can_if]->RegisterMotor(jh.motor);
  }

  state_pub_ = this->create_publisher<RobstrideStateMsg>(
    "~/robstride_state", rclcpp::SystemDefaultsQoS());
  state_pub_uni_ptr_ = std::make_unique<StatePublisher>(state_pub_);
  state_pub_uni_ptr_->msg_.id.resize(n);
  state_pub_uni_ptr_->msg_.enabled.resize(n);
  state_pub_uni_ptr_->msg_.run_state.resize(n);
  state_pub_uni_ptr_->msg_.fault_bits.resize(n);

  get_data_srv_ = this->create_service<robstride_interfaces::srv::GetDataFromRobstride>(
    "~/get_data_from_robstride",
    std::bind(
      &RobstrideHardware::GetDataCallback, this, std::placeholders::_1, std::placeholders::_2));
  set_data_srv_ = this->create_service<robstride_interfaces::srv::SetDataToRobstride>(
    "~/set_data_to_robstride",
    std::bind(
      &RobstrideHardware::SetDataCallback, this, std::placeholders::_1, std::placeholders::_2));
  set_zero_srv_ = this->create_service<robstride_interfaces::srv::SetZeroRobstride>(
    "~/set_zero_robstride",
    std::bind(
      &RobstrideHardware::SetZeroCallback, this, std::placeholders::_1, std::placeholders::_2));
  set_torque_srv_ = this->create_service<std_srvs::srv::SetBool>(
    "~/set_torque",
    std::bind(
      &RobstrideHardware::SetTorqueCallback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO_STREAM(
    logger_,
    "Initialized " << n << " RobStride joint(s) across " << buses_.size() << " CAN bus(es)");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobstrideHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joints_.size(); ++i) {
    state_interfaces.emplace_back(
      joints_[i].joint_name, hardware_interface::HW_IF_POSITION, &hw_state_position_[i]);
    state_interfaces.emplace_back(
      joints_[i].joint_name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocity_[i]);
    state_interfaces.emplace_back(
      joints_[i].joint_name, hardware_interface::HW_IF_EFFORT, &hw_state_effort_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobstrideHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joints_.size(); ++i) {
    command_interfaces.emplace_back(
      joints_[i].joint_name, hardware_interface::HW_IF_POSITION, &hw_cmd_position_[i]);
    command_interfaces.emplace_back(
      joints_[i].joint_name, hardware_interface::HW_IF_VELOCITY, &hw_cmd_velocity_[i]);
    command_interfaces.emplace_back(
      joints_[i].joint_name, hardware_interface::HW_IF_EFFORT, &hw_cmd_effort_[i]);
  }
  return command_interfaces;
}

CallbackReturn RobstrideHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & kv : buses_) {
    if (!kv.second->Open()) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to open CAN interface '" << kv.first << "'");
      return CallbackReturn::ERROR;
    }
    kv.second->Start();
  }

  // Arm each motor's own CAN watchdog (param 0x7028, units of 1/20000 s per
  // the RobStride manual) so the motor auto-resets if it stops hearing from
  // us, as a second line of defense alongside our host-side staleness check
  // in read()/write(). This param lives in RAM (lost on power-cycle), so it
  // never permanently changes the motor's saved configuration.
  const uint32_t can_timeout_raw =
    std::max(1u, static_cast<uint32_t>(std::min(error_timeout_ms_ * 20.0, 100000.0)));
  for (auto & jh : joints_) {
    buses_[jh.can_interface]->SendFrame(
      jh.motor->EncodeParamWriteU32(robstride_sdk::ParamIndex::CAN_TIMEOUT, can_timeout_raw));
  }

  for (auto & jh : joints_) {
    buses_[jh.can_interface]->SendFrame(jh.motor->EncodeEnable());
  }

  // Wait for each motor to actually confirm it is enabled (Type-2 feedback
  // with run_state == MOTOR) before trusting its position, rather than
  // blindly commanding position 0 -- a motor sitting far from its zero
  // point would otherwise snap toward 0 the instant write() starts sending
  // motion-control frames with real kp/kd.
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(static_cast<int64_t>(error_timeout_ms_));
  std::vector<bool> confirmed(joints_.size(), false);
  size_t num_confirmed = 0;
  auto next_resend = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
  while (num_confirmed < joints_.size() && std::chrono::steady_clock::now() < deadline) {
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (confirmed[i]) {continue;}
      const robstride_sdk::MotorState & st = joints_[i].motor->state();
      if (st.last_update_ns.load(std::memory_order_relaxed) != 0 &&
        st.run_state.load(std::memory_order_relaxed) ==
        static_cast<uint8_t>(robstride_sdk::RunState::MOTOR))
      {
        confirmed[i] = true;
        ++num_confirmed;
      }
    }
    if (num_confirmed < joints_.size()) {
      // CAN enable frames are fire-and-forget -- resend periodically in case
      // the original frame was dropped, rather than relying on one attempt.
      if (std::chrono::steady_clock::now() >= next_resend) {
        for (size_t i = 0; i < joints_.size(); ++i) {
          if (!confirmed[i]) {
            buses_[joints_[i].can_interface]->SendFrame(joints_[i].motor->EncodeEnable());
          }
        }
        next_resend += std::chrono::milliseconds(200);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  if (num_confirmed < joints_.size()) {
    for (size_t i = 0; i < joints_.size(); ++i) {
      if (!confirmed[i]) {
        RCLCPP_ERROR_STREAM(
          logger_,
          "Joint '" << joints_[i].joint_name << "' (id=" <<
            static_cast<int>(joints_[i].id) << ") did not confirm enable within " <<
            error_timeout_ms_ << " ms");
      }
    }
    for (auto & jh : joints_) {
      buses_[jh.can_interface]->SendFrame(jh.motor->EncodeDisable(false));
    }
    for (auto & kv : buses_) {
      kv.second->Stop();
      kv.second->Close();
    }
    return CallbackReturn::ERROR;
  }

  // All motors confirmed enabled and have reported a real position: seed
  // the command interfaces from measured state so write()'s first cycle
  // holds position instead of snapping to 0.
  for (size_t i = 0; i < joints_.size(); ++i) {
    hw_state_position_[i] = joints_[i].motor->state().position.load(std::memory_order_relaxed);
    hw_cmd_position_[i] = hw_state_position_[i];
    hw_cmd_velocity_[i] = 0.0;
    hw_cmd_effort_[i] = 0.0;
  }

  RCLCPP_INFO(logger_, "RobStride hardware activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RobstrideHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & jh : joints_) {
    const auto it = buses_.find(jh.can_interface);
    if (it != buses_.end()) {
      it->second->SendFrame(jh.motor->EncodeDisable(false));
    }
  }
  for (auto & kv : buses_) {
    kv.second->Stop();
    kv.second->Close();
  }

  RCLCPP_INFO(logger_, "RobStride hardware deactivated");
  return CallbackReturn::SUCCESS;
}

return_type RobstrideHardware::read(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  bool any_fault = false;
  bool any_stale = false;
  const uint64_t now_ns = robstride_sdk::NowNs();
  for (size_t i = 0; i < joints_.size(); ++i) {
    const robstride_sdk::MotorState & st = joints_[i].motor->state();
    hw_state_position_[i] = st.position.load(std::memory_order_relaxed);
    hw_state_velocity_[i] = st.velocity.load(std::memory_order_relaxed);
    hw_state_effort_[i] = st.torque.load(std::memory_order_relaxed);
    any_fault |= st.fault_present.load(std::memory_order_relaxed);

    const uint64_t last_ns = st.last_update_ns.load(std::memory_order_relaxed);
    const double elapsed_ms = ElapsedMsSinceUpdate(now_ns, last_ns);
    if (elapsed_ms < 0.0 || elapsed_ms > error_timeout_ms_) {
      any_stale = true;
      RCLCPP_ERROR_STREAM_THROTTLE(
        logger_, *this->rclcpp::Node::get_clock(), 1000,
        "Joint '" << joints_[i].joint_name << "' (id=" <<
          static_cast<int>(joints_[i].id) << ") feedback stale (" << elapsed_ms <<
          " ms since last update)");
    }
  }

  if (state_pub_uni_ptr_ && state_pub_uni_ptr_->trylock()) {
    auto & msg = state_pub_uni_ptr_->msg_;
    msg.header.stamp = time;
    msg.comm_state = any_fault ? -1 : 0;
    for (size_t i = 0; i < joints_.size(); ++i) {
      const robstride_sdk::MotorState & st = joints_[i].motor->state();
      msg.id[i] = joints_[i].id;
      msg.enabled[i] =
        st.run_state.load(std::memory_order_relaxed) ==
        static_cast<uint8_t>(robstride_sdk::RunState::MOTOR);
      msg.run_state[i] = st.run_state.load(std::memory_order_relaxed);
      msg.fault_bits[i] = st.fault_bits.load(std::memory_order_relaxed);
    }
    state_pub_uni_ptr_->unlockAndPublish();
  }

  if (rclcpp::ok()) {
    rclcpp::spin_some(this->get_node_base_interface());
  }

  if (any_stale) {
    return return_type::ERROR;
  }
  return return_type::OK;
}

return_type RobstrideHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const uint64_t now_ns = robstride_sdk::NowNs();

  std::unordered_map<std::string, std::vector<can_frame>> frames_by_bus;
  for (size_t i = 0; i < joints_.size(); ++i) {
    const JointHandle & jh = joints_[i];
    const robstride_sdk::MotorState & st = jh.motor->state();
    const uint64_t last_ns = st.last_update_ns.load(std::memory_order_relaxed);
    const double elapsed_ms = ElapsedMsSinceUpdate(now_ns, last_ns);
    const bool stale = elapsed_ms < 0.0 || elapsed_ms > error_timeout_ms_;
    const bool faulted = st.fault_present.load(std::memory_order_relaxed);
    if (stale || faulted) {
      // Don't keep driving a motor we haven't heard from recently (or that
      // is reporting a fault) toward a possibly stale command target --
      // send Disable instead of a motion-control frame for this joint.
      frames_by_bus[jh.can_interface].push_back(jh.motor->EncodeDisable(false));
      continue;
    }
    const can_frame frame = jh.motor->EncodeMotionCommand(
      static_cast<float>(hw_cmd_position_[i]),
      static_cast<float>(hw_cmd_velocity_[i]),
      static_cast<float>(jh.kp),
      static_cast<float>(jh.kd),
      static_cast<float>(hw_cmd_effort_[i]));
    frames_by_bus[jh.can_interface].push_back(frame);
  }

  for (auto & kv : frames_by_bus) {
    buses_[kv.first]->SendFrames(kv.second);
  }
  return return_type::OK;
}

JointHandle * RobstrideHardware::FindJointById(uint8_t id)
{
  for (auto & jh : joints_) {
    if (jh.id == id) {return &jh;}
  }
  return nullptr;
}

void RobstrideHardware::GetDataCallback(
  const std::shared_ptr<robstride_interfaces::srv::GetDataFromRobstride::Request> request,
  std::shared_ptr<robstride_interfaces::srv::GetDataFromRobstride::Response> response)
{
  JointHandle * jh = FindJointById(request->id);
  if (!jh) {
    response->result = false;
    return;
  }
  const robstride_sdk::MotorState & st = jh->motor->state();
  if (request->item_name == "position") {
    response->item_data = st.position.load();
  } else if (request->item_name == "velocity") {
    response->item_data = st.velocity.load();
  } else if (request->item_name == "torque" || request->item_name == "effort") {
    response->item_data = st.torque.load();
  } else if (request->item_name == "temperature") {
    response->item_data = st.temperature.load();
  } else if (request->item_name == "run_state") {
    response->item_data = static_cast<float>(st.run_state.load());
  } else if (request->item_name == "fault_bits") {
    response->item_data = static_cast<float>(st.fault_bits.load());
  } else {
    response->result = false;
    return;
  }
  response->result = true;
}

void RobstrideHardware::SetDataCallback(
  const std::shared_ptr<robstride_interfaces::srv::SetDataToRobstride::Request> request,
  std::shared_ptr<robstride_interfaces::srv::SetDataToRobstride::Response> response)
{
  JointHandle * jh = FindJointById(request->id);
  if (!jh) {
    response->result = false;
    return;
  }

  if (request->item_name == "kp") {
    jh->kp = request->item_data;
    response->result = true;
    return;
  }
  if (request->item_name == "kd") {
    jh->kd = request->item_data;
    response->result = true;
    return;
  }

  // Otherwise treat item_name as a RobStride parameter index (e.g. "0x7018")
  // and fire-and-forget a parameter write frame; the ack arrives async as a
  // Type-2 feedback frame that is already handled generically.
  try {
    const uint16_t index = static_cast<uint16_t>(std::stoul(request->item_name, nullptr, 0));
    const auto it = buses_.find(jh->can_interface);
    if (it == buses_.end()) {
      response->result = false;
      return;
    }
    response->result =
      it->second->SendFrame(jh->motor->EncodeParamWriteFloat(index, request->item_data));
  } catch (const std::exception &) {
    response->result = false;
  }
}

void RobstrideHardware::SetZeroCallback(
  const std::shared_ptr<robstride_interfaces::srv::SetZeroRobstride::Request> request,
  std::shared_ptr<robstride_interfaces::srv::SetZeroRobstride::Response> response)
{
  JointHandle * jh = FindJointById(request->id);
  if (!jh) {
    response->result = false;
    return;
  }
  const auto it = buses_.find(jh->can_interface);
  response->result = it != buses_.end() && it->second->SendFrame(jh->motor->EncodeSetZero());
}

void RobstrideHardware::SetTorqueCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  bool ok = true;
  for (auto & jh : joints_) {
    const auto it = buses_.find(jh.can_interface);
    if (it == buses_.end()) {
      ok = false;
      continue;
    }
    const can_frame frame =
      request->data ? jh.motor->EncodeEnable() : jh.motor->EncodeDisable(false);
    ok &= it->second->SendFrame(frame);
  }
  response->success = ok;
}

}  // namespace robstride_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robstride_hardware_interface::RobstrideHardware, hardware_interface::SystemInterface)

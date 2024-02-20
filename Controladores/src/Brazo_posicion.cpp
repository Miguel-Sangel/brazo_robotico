// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Authors: Subhas Das, Denis Stogl
//
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "mis_controladores_hardware/Brazo_posicion.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"


using publi = rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr;
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ref_subscriber_ = nullptr;
std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("prueba_cmd");
//rclcpp::Node node = rclcpp::Node("prueba_cmd");
publi s_publisher_ = node->create_publisher<sensor_msgs::msg::JointState>("/mi_mensaje_escribe", rclcpp::QoS(1).best_effort().durability_volatile());

namespace ros2_control_hardware
{
   
CallbackReturn BrazoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  hw_sensor_change_ = stod(info_.hardware_parameters["example_param_max_sensor_change"]);

  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  control_level_.resize(info_.joints.size(), integration_level_t::POSITION);
  //hw_sensor_states_.resize(info_.sensors[0].state_interfaces.size(), 0); HAY QUE DOTAR SENSOR EN URDF
  mensaje_env.position.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  mensaje_env.velocity.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  mensaje_env.effort.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints) // PARA CADA JOINT
  {
    // Brazo has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() < 1 or joint.command_interfaces.size() > 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BrazoHardware"),
        "Joint '%s' has %zu command interfaces found. Esperadas de 1 a 3.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BrazoHardware"),
        "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() < 1 or joint.state_interfaces.size() > 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BrazoHardware"),
        "Joint '%s' has %zu command interfaces found. Esperadas de 1 a 3.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("BrazoHardware"),
        "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION);
      return CallbackReturn::ERROR;
    }
  }
  
//  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("prueba_cmd");  // SOLA FUNCIONA
//    rclcpp::Node node = rclcpp::Node("prueba_cmd");
//  ref_subscriber_ = node->create_subscription<sensor_msgs::msg::JointState>(
//    "/mi_mensaje_lee", 10, std::bind(&BrazoHardware::reference_callback, this, std::placeholders::_1));
//  s_publisher_ = node.create_publisher<sensor_msgs::msg::JointState>("/mi_mensaje_escribe", 10);
     
  RCLCPP_FATAL( rclcpp::get_logger("BrazoHardware"),"  INICIO CORRECTO.");
  return CallbackReturn::SUCCESS;
}

//CallbackReturn BrazoHardware::on_configure(
//  const rclcpp_lifecycle::State & /*previous_state*/)
//{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
//  RCLCPP_INFO(rclcpp::get_logger("SystemWithSensorHardware"), "Configuring ...please wait...");

//  for (int i = 0; i < hw_start_sec_; i++)
//  {
//    rclcpp::sleep_for(std::chrono::seconds(1));
//    RCLCPP_INFO(
//      rclcpp::get_logger("SystemWithSensorHardware"), "%.1f seconds left...",
//      hw_start_sec_ - i);
//  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
//  for (uint i = 0; i < hw_joint_states_.size(); i++)
//  {
//    hw_joint_states_[i] = 0;
//    hw_joint_commands_[i] = 0;
//  }

//  RCLCPP_INFO(rclcpp::get_logger("SystemWithSensorHardware"), "Successfully configured!");

//  return CallbackReturn::SUCCESS;
//}

std::vector<hardware_interface::StateInterface>
BrazoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++){
   for (uint j = 0; j < info_.joints[i].state_interfaces.size(); j++)
   { if (info_.joints[i].state_interfaces[j].name == hardware_interface::HW_IF_POSITION)
         state_interfaces.emplace_back(hardware_interface::StateInterface(
         info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
     else if (info_.joints[i].state_interfaces[j].name == hardware_interface::HW_IF_VELOCITY)
         state_interfaces.emplace_back(hardware_interface::StateInterface(
         info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
     else if (info_.joints[i].state_interfaces[j].name == hardware_interface::HW_IF_ACCELERATION)
         state_interfaces.emplace_back(hardware_interface::StateInterface(
         info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_states_accelerations_[i]));
   }
  }

  // export sensor state interface
//  for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
//  {
//    state_interfaces.emplace_back(hardware_interface::StateInterface(
//      info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
//  }
  RCLCPP_FATAL(rclcpp::get_logger("BrazoHardware"), "ESTADOS INTERFACE");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
BrazoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++){
    for (uint j = 0; j < info_.joints[i].command_interfaces.size(); j++)
   { if (info_.joints[i].command_interfaces[j].name == hardware_interface::HW_IF_POSITION)
         command_interfaces.emplace_back(hardware_interface::CommandInterface(
         info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
     else if (info_.joints[i].command_interfaces[j].name == hardware_interface::HW_IF_VELOCITY)
         command_interfaces.emplace_back(hardware_interface::CommandInterface(
         info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
     else if (info_.joints[i].command_interfaces[j].name == hardware_interface::HW_IF_VELOCITY)
         command_interfaces.emplace_back(hardware_interface::CommandInterface(
         info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_commands_accelerations_[i]));
   }
  }
  RCLCPP_FATAL(rclcpp::get_logger("BrazoHardware"), "INTERFACE COMANDOS");
  return command_interfaces;
}




CallbackReturn BrazoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("BrazoHardware"), "Activating ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("BrazoHardware"), "%.1f seconds left...",
      hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (std::size_t i = 0; i < hw_states_positions_.size(); i++)
  {
    if (std::isnan(hw_states_positions_[i]))
    {
      hw_states_positions_[i] = 0;
    }
    if (std::isnan(hw_states_velocities_[i]))
    {
      hw_states_velocities_[i] = 0;
    }
    if (std::isnan(hw_states_accelerations_[i]))
    {
      hw_states_accelerations_[i] = 0;
    }
    if (std::isnan(hw_commands_positions_[i]))
    {
      hw_commands_positions_[i] = 0;
    }
    if (std::isnan(hw_commands_velocities_[i]))
    {
      hw_commands_velocities_[i] = 0;
    }
    if (std::isnan(hw_commands_accelerations_[i]))
    {
      hw_commands_accelerations_[i] = 0;
    }
    control_level_[i] = integration_level_t::POSITION;
  }

  last_timestamp_ = clock_.now();

  // set default value for sensor
//  if (std::isnan(hw_sensor_states_[0]))
//  {
//    hw_sensor_states_[0] = 0;
//  }

  RCLCPP_INFO(rclcpp::get_logger("BrazoHardware"), "ACTIVACION CORRECTA!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn BrazoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("BrazoHardware"), "Deactivating ...please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("BrazoHardware"), "%.1f seconds left...",
      hw_stop_sec_ - i);
  }

  RCLCPP_INFO(rclcpp::get_logger("BrazoHardware"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type BrazoHardware::read()
{
  current_timestamp = clock_.now();
  rclcpp::Duration duration = current_timestamp - last_timestamp_;
  last_timestamp_ = current_timestamp;

  for (std::size_t i = 0; i < hw_states_positions_.size(); i++)
  {
    switch (control_level_[i])
    {
      case BrazoHardware::integration_level_t::UNDEFINED:
        RCLCPP_INFO(
          rclcpp::get_logger("BrazoHardware"), "NO HAY DEFINIDA UNA INTEFAZ!");
        return hardware_interface::return_type::OK;
        break;
      case BrazoHardware::integration_level_t::POSITION:
        hw_states_accelerations_[i] = 0;
        hw_states_velocities_[i] = 0;
        hw_states_positions_[i] += (hw_commands_positions_[i] - hw_states_positions_[i]) / hw_slowdown_;
        break;
      case BrazoHardware::integration_level_t::VELOCITY:
        hw_states_accelerations_[i] = 0;
        hw_states_velocities_[i] = hw_commands_velocities_[i];
        hw_states_positions_[i] += (hw_states_velocities_[i] * duration.seconds()) / hw_slowdown_;
        break;
      case BrazoHardware::integration_level_t::ACCELERATION:
        hw_states_accelerations_[i] = hw_commands_accelerations_[i];
        hw_states_velocities_[i] += (hw_states_accelerations_[i] * duration.seconds()) / hw_slowdown_;
        hw_states_positions_[i] += (hw_states_velocities_[i] * duration.seconds()) / hw_slowdown_;
        break;
    }
  }

//  for (uint i = 0; i < hw_sensor_states_.size(); i++)
//  {
    // Simulate RRBot's sensor data
//    unsigned int seed = time(NULL) + i;
//    hw_sensor_states_[i] =
//      static_cast<float>(rand_r(&seed)) / (static_cast<float>(RAND_MAX / hw_sensor_change_));
//    RCLCPP_INFO(
//      rclcpp::get_logger("BrazoHardware"), "Got state %e for interface %s!",
//      hw_sensor_states_[i], info_.sensors[0].state_interfaces[i].name.c_str());
//  }
//  RCLCPP_INFO(rclcpp::get_logger("BrazoHardware"), "Sensors successfully read!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BrazoHardware::write()
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code

  for (std::size_t i = 0; i < hw_commands_positions_.size(); i++) // SIEMPRE LEE Y EJECUTA
  {
    switch (control_level_[i])
    {
      case BrazoHardware::integration_level_t::UNDEFINED:
        RCLCPP_INFO(
          rclcpp::get_logger("BrazoHardware"), "Nothing is using the hardware interface!");
        return hardware_interface::return_type::OK;
        break;
      case BrazoHardware::integration_level_t::POSITION:
        mensaje_env.position[i] = hw_commands_positions_[i];
        break;
      case BrazoHardware::integration_level_t::VELOCITY:
      mensaje_env.velocity[i] = hw_commands_velocities_[i];
        break;
      case BrazoHardware::integration_level_t::ACCELERATION:
        mensaje_env.effort[i] = hw_commands_accelerations_[i];
        break;
     }
  }
  s_publisher_->publish(mensaje_env);

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_hardware::BrazoHardware, hardware_interface::SystemInterface)

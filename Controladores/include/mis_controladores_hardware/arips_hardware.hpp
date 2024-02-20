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

#ifndef ROS2_CONTROL_HARDWARE_BRAZO_
#define ROS2_CONTROL_HARDWARE_BRAZO_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <serial/serial.h> //En CMklist.txt del pkg serial_ros  RESUELVE PROBLEMA "relocation R_X86_64_PC32" usando -fPIC #######

//#include <sensor_msgs/msg/joint_state.hpp>
//#include <trajectory_msgs/msg/JointTrajectory>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


namespace ros2_control_hardware
{
class Mi_Brazo_Hardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Mi_Brazo_Hardware);


  CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;


//  CallbackReturn on_configure(
//    const rclcpp_lifecycle::State & previous_state) override;


  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;


  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;


  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;


  hardware_interface::return_type read() override;


  hardware_interface::return_type write() override;
  
  double conv_serv_a_grad(int valor);
  int conv_grad_a_serv(double valor);
  
private:
  // Parameters for the RRBot simulation
//  double hw_start_sec_;
//  double hw_stop_sec_;
//  double hw_slowdown_;
//  double hw_sensor_change_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
//  std::vector<double> hw_commands_accelerations_;
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
//  std::vector<double> hw_states_accelerations_;
  
//  std::vector<double> hw_sensor_states_;

  
  
    // Store time between update loops
//  rclcpp::Clock clock_;
//  rclcpp::Time last_timestamp_;
//  rclcpp::Time current_timestamp;  // PARA CALCULAR VELOCIDAD Y ACELERACION
  
  std::unique_ptr<serial::Serial> pSerial;
  
};

}  // namespace ros2_control_hardware

#endif  // ROS2_CONTROL_HARDWARE_BRAZO_

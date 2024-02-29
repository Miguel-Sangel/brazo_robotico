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

#include "mis_controladores_hardware/arips_hardware.hpp"
//#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

//---------------ESTO LO USE EN UNA PRUEBA PARA MICRO-ROS----------
//using publi = rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr;
//rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr ref_subscriber_ = nullptr;
//std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("prueba_cmd");
//rclcpp::Node node = rclcpp::Node("prueba_cmd");
//publi s_publisher_ = node->create_publisher<sensor_msgs::msg::JointState>("/mi_mensaje_escribe", rclcpp::QoS(1).best_effort().durability_volatile());

namespace ros2_control_hardware
{
   
CallbackReturn Mi_Brazo_Hardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
//  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
//  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
//  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
//  hw_sensor_change_ = stod(info_.hardware_parameters["example_param_max_sensor_change"]);

  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
//  hw_states_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
//  hw_commands_accelerations_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());


  for (const hardware_interface::ComponentInfo & joint : info_.joints) // PARA CADA JOINT
  {
    // COMPROBAR QUE HAY DE UNA A TRES INTERFASES DE COMANDO
    if (joint.command_interfaces.size() < 1 or joint.command_interfaces.size() > 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Mi_Brazo_Hardware"),
        "Joint '%s' has %zu iNTERFACES ENCONTRADAS. Esperadas de 1 a 3.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }
    // COMPROBAR QUE NOMBRES INTERFASES DE COMANDO CORRECTOS
    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Mi_Brazo_Hardware"),
        "Joint '%s' has %s command interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION);
      return CallbackReturn::ERROR;
    }
    
    // COMPROBAR QUE HAY DE UNA A TRES INTERFASES DE ESTADO
    if (joint.state_interfaces.size() < 1 or joint.state_interfaces.size() > 3)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Mi_Brazo_Hardware"),
        "Joint '%s' has %zu command interfaces found. Esperadas de 1 a 3.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }
    
    // COMPROBAR QUE NOMBRES DE INTERFACES DE ESTADO SON CORRECTOS
    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION))
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Mi_Brazo_Hardware"),
        "Joint '%s' has %s state interface. Expected %s, %s, or %s.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
        hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_ACCELERATION);
      return CallbackReturn::ERROR;
    }
  }
  
//  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("prueba_cmd");  // SOLA FUNCIONA
//    rclcpp::Node node = rclcpp::Node("prueba_cmd");
//  ref_subscriber_ = node->create_subscription<sensor_msgs::msg::JointState>(
//    "/mi_mensaje_lee", 10, std::bind(&Mi_Brazo_Hardware::reference_callback, this, std::placeholders::_1));
//  s_publisher_ = node.create_publisher<sensor_msgs::msg::JointState>("/mi_mensaje_escribe", 10);

  pSerial = std::make_unique<serial::Serial>();

  pSerial->setPort("/dev/ttyUSB0");  
   
  pSerial->setBaudrate(1000000);
  pSerial->setBytesize(serial::eightbits);
  pSerial->setParity(serial::parity_none);
  pSerial->setStopbits(serial::stopbits_one);
  pSerial->setTimeout(1, 0, 1, 0, 1);
  pSerial->open();

  
  if (pSerial->isOpen()) printf("\n CONECTADO ");
     else { printf("\n NO conectado");
            return CallbackReturn::ERROR;
          }
     
  RCLCPP_INFO( rclcpp::get_logger("Mi_Brazo_Hardware")," INICIO CORRECTO. N de juntas = %ld", info_.joints.size());
  return CallbackReturn::SUCCESS;
}

//CallbackReturn Mi_Brazo_Hardware::on_configure(
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
Mi_Brazo_Hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++){ // PARA CADA JOINT
   for (uint j = 0; j < info_.joints[i].state_interfaces.size(); j++) // PARA CADA TIPO ESTADO
   { if (info_.joints[i].state_interfaces[j].name == hardware_interface::HW_IF_POSITION) // SI ESTADO POSICION
         state_interfaces.emplace_back(hardware_interface::StateInterface(
         info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
     else if (info_.joints[i].state_interfaces[j].name == hardware_interface::HW_IF_VELOCITY)
         state_interfaces.emplace_back(hardware_interface::StateInterface(
         info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
//     else if (info_.joints[i].state_interfaces[j].name == hardware_interface::HW_IF_ACCELERATION)
//         state_interfaces.emplace_back(hardware_interface::StateInterface(
//         info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_states_accelerations_[i]));
   }
  }

  // export sensor state interface
//  for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
//  {
//    state_interfaces.emplace_back(hardware_interface::StateInterface(
//      info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
//  }
  RCLCPP_INFO(rclcpp::get_logger("Mi_Brazo_Hardware"), "ESTADOS INTERFACE");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
Mi_Brazo_Hardware::export_command_interfaces()
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
//     else if (info_.joints[i].command_interfaces[j].name == hardware_interface::HW_IF_VELOCITY)
//         command_interfaces.emplace_back(hardware_interface::CommandInterface(
//         info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &hw_commands_accelerations_[i]));
   }
  }
  RCLCPP_INFO(rclcpp::get_logger("Mi_Brazo_Hardware"), "INTERFACE COMANDOS");
  return command_interfaces;
}




CallbackReturn Mi_Brazo_Hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("Mi_Brazo_Hardware"), "Activating ...please wait...");

//  for (int i = 0; i < hw_start_sec_; i++)
  for (int i = 0; i < 3; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("Mi_Brazo_Hardware"), "%.1d seconds left...",
///      hw_start_sec_ - i);
      3-i);
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
//    if (std::isnan(hw_states_accelerations_[i]))
//    {
//      hw_states_accelerations_[i] = 0;
//    }
    if (std::isnan(hw_commands_positions_[i]))
    {
      hw_commands_positions_[i] = 0;
    }
    if (std::isnan(hw_commands_velocities_[i]))
    {
      hw_commands_velocities_[i] = 0;
    }
//    if (std::isnan(hw_commands_accelerations_[i]))
//    {
//      hw_commands_accelerations_[i] = 0;
//    }
  }
  
// set default value for sensor
//  if (std::isnan(hw_sensor_states_[0]))
//  {
//    hw_sensor_states_[0] = 0;
//  }

  RCLCPP_INFO(rclcpp::get_logger("Mi_Brazo_Hardware"), "ACTIVACION CORRECTA!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn Mi_Brazo_Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(
    rclcpp::get_logger("Mi_Brazo_Hardware"), "Deactivating ...please wait...");

  for (int i = 0; i < 2; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("Mi_Brazo_Hardware"), "%.1d seconds left...",
      2 - i);
  }

  RCLCPP_INFO(rclcpp::get_logger("Mi_Brazo_Hardware"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return CallbackReturn::SUCCESS;
}

double Mi_Brazo_Hardware::conv_serv_a_grad(int valor, int servo){
   if (servo == 2332)
      return ((valor-50) * 0.0044) - 2.09; // (2.09--2.09=4.18) / (1000-50=950) = 0.0044
   else if (servo == 3032)
           return ((valor-20) * 0.001538462) - 3.1; // (3.1--3.1=6.2) / (4050-20=4030) = 0.001538462
   else if (servo == 215) return 0.0;
   else return 0;
}
int Mi_Brazo_Hardware::conv_grad_a_serv(double valor, int servo){
   if (servo == 2332)
      return ((valor+2.09) * 227.27) + 50; // (1000-50=950) / (2.09--2.09=4.18) = 227.27
   else if (servo == 3032)
           return ((valor+3.1) * 650) + 20; // (4050-20) / (3.1--3.1=6.2) = 650
   else if (servo == 215) return 0;
   else return 0;
}

hardware_interface::return_type Mi_Brazo_Hardware::read()
{
  int posi_leida, velo_leida;
  std::vector<uint8_t> buf_l(8);
  uint8_t Dat_lei[10];
  int chec_sum = 0;
//  RCLCPP_INFO(rclcpp::get_logger("Buscando"), "Probando... = %ld!", hw_states_positions_.size());
  for (std::size_t i = 0; i < hw_states_positions_.size(); i++)
  { //if (i < 4) {
    chec_sum = 0;  
    pSerial->flush ();
    buf_l[0] = 0xFF;
    buf_l[1] = 0xFF;
    buf_l[2] = i+1; //ID
    buf_l[3] = 0x04; // Longitud
    buf_l[4] = 0x02; // Orden escribo para lectura
    buf_l[5] = 0x38; // Dirección principal del segmento de datos leído, si "posicion" = 0x38 ó 56 en binario
    buf_l[6] = 4; // si orden es lectura Longitud bytes a leer
  
    for (int j = 2; j < 7; j++) {
      chec_sum += buf_l[j];
    }
    if (chec_sum > 255){
       chec_sum = chec_sum - 256; ///////////////////// OJO 256
    }
    chec_sum = 0xFF - chec_sum;
    buf_l[7] = chec_sum; // Chec_sum byte de control
//    printf("\n %d", chec_sum);

//    printf("\n ENVIO %s - %d %d %d %d %d %d %d %d", cadena.c_str(), buf_l[0], buf_l[1], buf_l[2], buf_l[3], buf_l[4], buf_l[5], buf_l[6], buf_l[7]);
  
    pSerial->write(buf_l);

    pSerial->read(Dat_lei, 10);
    
    if (i < 4){
       posi_leida = Dat_lei[5];
       velo_leida = Dat_lei[7];
       posi_leida<<=8;
       posi_leida |= Dat_lei[6];    
       velo_leida<<=8;
       velo_leida |= Dat_lei[8];
       hw_states_positions_[i] = conv_serv_a_grad(posi_leida, 2332);
       hw_states_velocities_[i] = velo_leida;
    }
    else if (i == 4){
       posi_leida = Dat_lei[6];
       velo_leida = Dat_lei[8];
       posi_leida<<=8;
       posi_leida |= Dat_lei[5];    
       velo_leida<<=8;
       velo_leida |= Dat_lei[7];
       hw_states_positions_[i] = conv_serv_a_grad(posi_leida, 3032);
       hw_states_velocities_[i] = velo_leida;
    }
    else { 
       hw_states_positions_[i] = 0;
       hw_states_velocities_[i] = 0;
    }
  }
  //else {hw_states_positions_[i] = 0; ////////////////    SOLO POR QUE TENGO 2 MOTORES
  //      hw_states_velocities_[i] = 0;}
  //RCLCPP_INFO(rclcpp::get_logger("relleno "), "Posicion = %f, del servo %ld", hw_states_positions_[i], i);
  //}

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Mi_Brazo_Hardware::write()
{
  int chec_sum = 0;
  std::vector<uint8_t> buf_p(13);
  int pos, vel;
  uint8_t Dat_lei[6];
  size_t k = 6;

  for (std::size_t i = 0; i < hw_commands_positions_.size(); i++) // SIEMPRE LEE Y EJECUTA
  {
    chec_sum = 0;
    pSerial->flush ();
    if (i != 4)
        pos = conv_grad_a_serv(hw_commands_positions_[i], 2332);
    else
        pos = conv_grad_a_serv(hw_commands_positions_[i], 3032);
    //pos = hw_commands_positions_[i];
    vel = hw_commands_velocities_[i];
    //RCLCPP_INFO(rclcpp::get_logger("Mandar "), "Posicion = %d, del servo %ld", pos, i);
    buf_p[0] = 0xFF;
    buf_p[1] = 0xFF;
    buf_p[2] = i+1; //ID
    buf_p[3] = 0x09; // Longitud
    buf_p[4] = 0x03; // Orden escribir
    buf_p[5] = 0x2A; // Dirección principal del segmento de datos escrito "posicion"
    if (i < 4){ 
        buf_p[6] = (pos>>8); // Byte alto (pos>>8)
        buf_p[7] = (pos&0xff); // Byte bajo - 0800 (2048) "posicion" (pos&0xff)
        buf_p[10] = (vel>>8); // Byte alto
        buf_p[11] = (vel&0xff); // Byte bajo - 03E8 (1000) "velocidad" // 0064 (100 velocida en decimal)
        }
    else if (i == 4){
          buf_p[7] = (pos>>8); // Byte alto (pos>>8)
          buf_p[6] = (pos&0xff); // Byte bajo - 0800 (2048) "posicion" (pos&0xff)
          buf_p[11] = (vel>>8); // Byte alto
          buf_p[10] = (vel&0xff); // Byte bajo - 03E8 (1000) "velocidad" // 0064 (100 velocida en decimal)
         }
    else {
          buf_p[6] = 0; // Byte alto (pos>>8)
          buf_p[7] = 0; // Byte bajo - 0800 (2048) "posicion" (pos&0xff)
          buf_p[10] = 0; // Byte alto
          buf_p[11] = 0; // Byte bajo - 03E8 (1000) "velocidad" // 0064 (100 velocida en decimal)
         }
    buf_p[8] = 0x00; // "tiempo"
    buf_p[9] = 0x00; //
    for (int j = 2; j < 12; j++) {
       chec_sum += buf_p[j];
    }
    if (chec_sum > 255){
       chec_sum = chec_sum - 256; ///////////////////// OJO 256
    }
    chec_sum = 255 - chec_sum;
    buf_p[12] = chec_sum; // Chec summ
//    printf("%d", chec_sum);
//    printf("\n ENVIO MOVER - %d %d %d %d %d %d %d %d %d %d %d %d %d", buf_p[0], buf_p[1], buf_p[2], buf_p[3], buf_p[4], buf_p[5], buf_p[6], buf_p[7], buf_p[8], buf_p[9], buf_p[10], buf_p[11], buf_p[12]);

    //if ( i < 4)               // porque tengo dos motores
       pSerial->write(buf_p);

    pSerial->read(Dat_lei, k);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_hardware::Mi_Brazo_Hardware, hardware_interface::SystemInterface)

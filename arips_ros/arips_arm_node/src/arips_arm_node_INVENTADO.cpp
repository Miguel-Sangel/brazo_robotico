#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

#include <rclcpp_serial/rclcpp_serial.hpp>
#include <iostream>



typedef  unsigned char	u8;	
typedef  unsigned short u16;


  
int main(int argc, char** argv)
{
  // Set up ROS.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("arips_arm_node");

  
  //rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr brazo_pub = node->create_publisher<geometry_msgs::msg::Pose>("tema_pose", 1);
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr brazo_pub = node->create_publisher<std_msgs::msg::Int32>("pose", 1);
  
  std::string serial_port = "/dev/ttyUSB0";
  int baud_rate = 1000000;
  rclcpp_serial::SerialOptions serial_options(serial_port, baud_rate);
  
  auto serial_comm = std::make_shared<rclcpp_serial::SerialComm>(serial_options);
  
  // Configurar la recepción de datos del puerto serie
  serial_comm->setSerialDataCallback([](uint8_t *data, size_t len) {
    // Procesar los datos recibidos aquí
    // Por ejemplo, publicarlos en un topic de ROS2
  });
  
  // Iniciar la comunicación serie
  if (serial_comm->init()) {
    RCLCPP_INFO(node->get_logger(), "Comunicación serie iniciada con éxito");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Error al iniciar la comunicación serie");
    return -1;
  }
}

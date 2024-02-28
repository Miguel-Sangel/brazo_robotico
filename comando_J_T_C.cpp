#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;

class ObjetoComando : public rclcpp::Node
{
  public:
    ObjetoComando()
    : Node("comando_J_T_C")
    {
      publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/Mi_posicion_controler/joint_trajectory", 5);
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/xxxx/joint_states", 10, std::bind(&ObjetoComando::callback, this, _1));

    }

  private:
    void callback(const sensor_msgs::msg::JointState & message)
    {
      trajectory_msgs::msg::JointTrajectory data;
      trajectory_msgs::msg::JointTrajectoryPoint punto;
      punto.positions = message.position;
      data.header.frame_id = "prueba";
      data.joint_names = message.name;
      data.points.push_back(punto);
      publisher_->publish(data);
    }
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjetoComando>());
  rclcpp::shutdown();
  return 0;
}

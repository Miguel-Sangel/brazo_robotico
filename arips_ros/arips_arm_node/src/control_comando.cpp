//#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>

#include <iostream>
#include <stdio.h>
#include <std_msgs/msg/int16.h>


class Transformar
{
public:
  Transformar(rclcpp::Clock::SharedPtr momento, std::string cadena1="world", std::string cadena2="mano")
  :
  origen(cadena1),
  destino(cadena2)
  {
//    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(momento);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
  
  std::string origen;
  std::string destino;
  geometry_msgs::msg::Pose resultado;
  
  void calcula () {
    geometry_msgs::msg::TransformStamped t;
    tf2::Quaternion q_new;
    
    try {
          t = tf_buffer_->lookupTransform(origen, destino, tf2::TimePointZero);
          
          resultado.position.x = t.transform.translation.x; // - 0.2; // posicion efector final
          resultado.position.y = t.transform.translation.y;
          resultado.position.z = t.transform.translation.z;
          tf2::convert(t.transform.rotation, q_new);
          q_new.normalize();
          resultado.orientation.x = q_new.x();  //  
          resultado.orientation.y = q_new.y();   //  
          resultado.orientation.z = q_new.z();   // 
          resultado.orientation.w = q_new.w();
        } catch (const tf2::TransformException & ex) {
//          RCLCPP_INFO(this->get_logger(), "No puedo transformar %s to %s: %s", destino.c_str(), origen.c_str(), ex.what());
          printf("No puedo transformar %s to %s: %s", destino.c_str(), origen.c_str(), ex.what());
          return;
        }
  }
  
private:
  
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
//  rclcpp::
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("control_comando");
    
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr brazo_pub = node->create_publisher<geometry_msgs::msg::Pose>("tema_pose", 1);
    
    
    Transformar transformar(node->get_clock(), "world", "mano_link");
//    Transformar transformar(node->now());
//    transformar.origen = "base_footprint";
//    transformar.destino = "mano_link";
    transformar.calcula();
    
    geometry_msgs::msg::Pose pose;
    int control = -1;
    while (rclcpp::ok()){
       if (control == -1)
       {
        transformar.calcula();
        printf("\n\n POSICION MUNECA : X = %f - Y = %f - Z = %f \n", transformar.resultado.position.x, transformar.resultado.position.y, transformar.resultado.position.z);
        std::cout << "\n\t 0 - Recuperar Posicion Guardada";
        std::cout << "\n\t 1 - Guarda Posicion";
        std::cout << "\n\t 2 - Posicion Anterior";
        std::cout << "\n\t 3 - ELEGIR POSE";
        std::cout << "\n\t 4 - RECARGAR COORDENADAS";
        std::cout << "\n\t 5 - SALIR";
        std::cout << "\n\t introduce opcion: ";
        std::cin >> control;
        if (control == 5) break;
       }
       else{
        if (control == 0)
         { pose.position.x = 10; // envio mensaje recuperar guardada
           std::cout << "\n\t Elige del 1 al 5 para RECUPERAR esta posicion : ";
           std::cin >> pose.position.y;
           brazo_pub->publish(pose);
//           continuar.data = 10;
           control = -1;
         }
         else if (control == 1) 
                 { pose.position.x = 11; // envio mensaje guarda posicion
                   std::cout << "\n\t Elige del 1 al 5 para ALMACENAR esta posicion : ";
                   std::cin >> pose.position.y;
                   brazo_pub->publish(pose);
                   std::cout << "\n\t\t  POSICION ALMACENADA \n\n";
//                   continuar.data = 0;
                   control = -1;
                 }
                 else if (control == 2)
                      { pose.position.x = 12; // envio mensaje recuperar previo
                        brazo_pub->publish(pose);
//                        continuar.data = 10;
                        control = -1;
                      }
                      else if (control == 3)
                          {float A;
                           std::cout << "\n\t\t INTRODUCE VALOR -X-: ";
                           scanf("%f", &A);
                           pose.position.x = A;
                           std::cout << "\n\t\t INTRODUCE VALOR -Y-: ";
                           scanf("%f", &A);
                           pose.position.y = A;
                           std::cout << "\n\t\t INTRODUCE VALOR -Z-: ";
                           scanf("%f", &A);
                           pose.position.z = A;
                           
                           float ang_y = asin((pose.position.z-0.35) / sqrt (pow(pose.position.y, 2) + pow((pose.position.z-0.35), 2)));
                           float ang_z = asin(pose.position.y / sqrt (pow(pose.position.y, 2) + pow(pose.position.x, 2)));
                           if (pose.position.x < 0) ang_z = ang_z * -1;
                             else ang_z = (3.14 - ang_z) * -1;
                           //std::cout << "\n\n A. Deseado y= " << ang_y*180/3.14 << " - z= " <<  ang_z*180/3.14;
                           tf2::Quaternion q;
                           q.setRPY(-3.14, ang_y, ang_z);
                           q.normalize();
                           
                           tf2::convert(q, pose.orientation);
                           
                           //pose.orientation.x = q.x();
                           //pose.orientation.y = q.y();
                           //pose.orientation.z = q.z();
                           //pose.orientation.w = q.w();
                           brazo_pub->publish(pose);
//                           continuar.data = 10;
                           control = -1;
                          }
                          else
                                { control = -1;
//                                  continuar.data = 0;
                                }

       }
    }

    rclcpp::shutdown();
}


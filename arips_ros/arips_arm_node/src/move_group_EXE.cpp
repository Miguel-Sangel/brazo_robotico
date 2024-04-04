#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
//#include <moveit_error_code.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include "geometry_msgs/msg/pose.h"
//#include "geometry_msgs/msg/transform_stamped.hpp"


static const rclcpp::Logger LOGGER = rclcpp::get_logger("Nodo_mi_moveit");

geometry_msgs::msg::Pose pose;
int mensaje = 0;
char posicion[10];

void recibir_pose(const geometry_msgs::msg::Pose& nueva_pose) // desde teclado
{
  //const geometry_msgs::Pose& 
  pose = nueva_pose;
  if (pose.position.x == 10)
     { mensaje = 10;
       switch(int(pose.position.y))
       { case 1: strcpy (posicion, "uno");
         break;
         case 2: strcpy (posicion, "dos");
         break;
         case 3: strcpy (posicion, "tres");
         break;
         case 4: strcpy (posicion, "cuatro");
         break;
         case 5: strcpy (posicion, "cinco");
         break;
       }
     }
  else if (pose.position.x == 11)
     { mensaje = 11;
       switch(int(pose.position.y))
       { case 1: strcpy (posicion, "uno");
         break;
         case 2: strcpy (posicion, "dos");
         break;
         case 3: strcpy (posicion, "tres");
         break;
         case 4: strcpy (posicion, "cuatro");
         break;
         case 5: strcpy (posicion, "cinco");
         break;
       }
     }
  else if (pose.position.x == 12)
     mensaje = 12;
  else mensaje = 13;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("mi_moveit", node_options);  // En CMakelists lo llamo "calculator"
  
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr escucha_pose = move_group_node->create_subscription<geometry_msgs::msg::Pose>("tema_pose", 1, &recibir_pose);
  rclcpp::Publisher<moveit_msgs::msg::MoveItErrorCodes>::SharedPtr respuesta_pub = move_group_node->create_publisher<moveit_msgs::msg::MoveItErrorCodes>("resultado_moveit", 1);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_brazo = "gru_brazo";

//  moveit::planning_interface::MoveGroupInterface::Options opcion(PLANNING_brazo, "robot_description");
  moveit::planning_interface::MoveGroupInterface move_brazo(move_group_node, PLANNING_brazo);
//  move_brazo.setWorkspace(0.0, 0.0, 0.0, 5.0, 5.0, 5.0); no veo como funciona
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const moveit::core::JointModelGroup* joint_model_group = move_brazo.getCurrentState()->getJointModelGroup(PLANNING_brazo);
  RCLCPP_INFO(LOGGER, "QUE EFECTOR FINAL TENGO: %s", move_brazo.getEndEffectorLink().c_str());
  
  
  robot_model_loader::RobotModelLoader robot_model_loader(move_group_node, "robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  RCLCPP_INFO_STREAM(LOGGER, "EL ESTADO ES " << (collision_result.collision ? "SI COLISIONA" : "NO COLISIONA"));
  
  
  moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  collision_request.group_name = "gru_mano";
  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  RCLCPP_INFO_STREAM(LOGGER, "gru_mano " << (collision_result.collision ? "SISISISI" : "NONONONO"));
  
  

//  moveit::core::RobotState start_state(*move_brazo.getCurrentState());
//  move_brazo.setStartState(start_state);      
  moveit_msgs::msg::MoveItErrorCodes dev; 
  bool previo_bool = false;   
  while (rclcpp::ok()){
//  	rclcpp::spin_some(); No hace falta, ya gira con el executor
  	
  	if (mensaje > 0) {
//  	   move_brazo.setStartState(start_state); // creo que no hace falta
           move_brazo.setPlanningTime(3);
           if ( mensaje == 13)
              { move_brazo.rememberJointValues("previo");
                previo_bool = false;
                move_brazo.setPoseTarget(pose);
           }  
           else if (mensaje == 10)
                   { move_brazo.rememberJointValues("previo");
                     previo_bool = false;
                     move_brazo.setNamedTarget(posicion); // recuperar guardado
                   }
                else if (mensaje == 11)
                        move_brazo.rememberJointValues(posicion); // guardar
                     else if (mensaje == 12)  // recuperar previo
                             { if (previo_bool)
                                  { previo_bool = false;
                                    move_brazo.rememberJointValues("previo");
                                    move_brazo.setNamedTarget("previo_2"); // recuperar previo_2
                                  }
                                else { previo_bool = true;
                                       move_brazo.rememberJointValues("previo_2");
                                       move_brazo.setNamedTarget("previo"); // recuperar previo
                                     }
                             }
         if (mensaje != 11)
            {  //move_brazo.setGoalJointTolerance(0.01);
               //move_brazo.setGoalTolerance(0.5);
               move_brazo.setGoalOrientationTolerance(0.5); // mejor solucion, solo varia orientacion eeff.
               dev = move_brazo.move();
               respuesta_pub->publish(dev);
            }
         mensaje = 0;
       }
  
  } // end while
  rclcpp::shutdown();
}

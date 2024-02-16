# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
import xacro
import yaml


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

####### FUNCIONES LEER FICHEROS ########################

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

############################################################################



def generate_launch_description():

######## EJEMPLO PARA TRABAJAR CON ARGUMENTOS ############
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "simulado",
            default_value="true",
            description="iniciar simulacion",
        )
    )

    simulado = LaunchConfiguration("simulado")
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("arips_arm_description"), "urdf", "arips_arm.xacro"]),
            " ",
            "use_sim_time:=",
            simulado,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
#    cargar_modelo = Node(
#        package='ros_ign_gazebo',
#        executable='create',
#        output='screen',
#        arguments=['-string', robot_description_content,
#                   '-name', 'arips_arm',
#                   '-allow_renaming', 'true'],
##        parameters=[{"use_sim_time": simulado}],    ##     creo que no sirve
#        condition=IfCondition(simulado),
#    )
    

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("arips_arm_description"),
            "config",
            "brazo_controllers.yaml",
        ]
    )
    
    control_node = Node(   ####### NO ES NECESARIO POR QUE EL COMPLEMENTO "ign_ros2_control" CARGA UN controller_manager
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
#        condition=UnlessCondition(simulado),
    )
    
#    puente = Node(
#        package="ros_ign_bridge",
#        executable="parameter_bridge",
#        output={
#            "stdout": "screen",
#            "stderr": "screen",
#        },
#        arguments=[
#                "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
#                "--ros-args",
#                "--log-level",
#                "warn",
#        ],
#        parameters=[{"use_sim_time": simulado}],
#        condition=IfCondition(simulado),
#     )
    
#    load_joint_state_broadcaster_spawner = ExecuteProcess(
#        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
#             'joint_state_broadcaster'],
#        output='screen'
#    )
    
    joint_state_broadcaster_spawner = Node(
        name="j_s_b_sp",
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager",],
#        parameters=[{"use_sim_time": simulado}],
    )
    
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["Mi_posicion_controler", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": simulado}],
    )
    
#    robot_controller_spawner_hand = Node(
#        package="controller_manager",
#        executable="spawner",
#        arguments=["moveit_hand_controller", "-c", "/controller_manager"],
#        parameters=[{"use_sim_time": simulado}],
#    )
    
    
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
#        remappings=[(
#         "/joint_states", "/xxxx/joint_states",
#        ),
#        ],
    )
    
#    static_tf = Node(
#        package="tf2_ros",
#        executable="static_transform_publisher",
#        name="static_transform_publisher",
#        output="log",
##        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "peana_link"],
#    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": simulado}],
#        remappings=[(
#         "/joint_states", "/joint_state_broadcaster/joint_states",
#        ),
#        ],
    )
    
#    comando = Node(
#        package="descripcion_robot",
##        namespace='ostras',
#        executable="comando_J_T_C",
#        name="comando_J_T_C",
#        output="screen",
#    )
        
#   CONEXION PUERTO SERIE PARA MICROROS (puente)        
#    agente = ExecuteProcess(
#        cmd=['MicroXRCEAgent', 'serial', '--dev', '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',],
#        output='screen'
#    )    
    
######################################################## MOVEIT ##############################################################  

  
#    robot_description_semantic_load = load_file(
#        "descripcion_robot", "moveit/brazo.srdf"
#    )
#    robot_description_semantic = {
#        "robot_description_semantic": robot_description_semantic_load
#    }
    
#    kinematics_yaml = load_yaml(
#        "descripcion_robot", "moveit/kinematics.yaml"
#    )
    
#    joint_limits_yaml = load_yaml(
#        "descripcion_robot", "moveit/joint_limits.yaml"
#    )
    
#    planning_plugin = {"planning_plugin": "ompl_interface/OMPLPlanner"}
#    ompl_planning_pipeline_config = {
#        "move_group": {
#            "planning_plugin": "ompl_interface/OMPLPlanner",
#            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
#            "start_state_max_bounds_error": 0.1,
#        }
#    }
#    ompl_planning_yaml = load_yaml(
#        "descripcion_robot", "moveit/ompl_planning.yaml"
#    )
#    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)
    
#    moveit_simple_controllers_yaml = load_yaml(
#        "descripcion_robot", "moveit/moveit_controllers.yaml"
#    )
#    moveit_controllers = {
#        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
#        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
#    }

#    trajectory_execution = {
#        "moveit_manager_controllers": True,
#        "trajectory_execution.allowed_execution_duration_scaling": 1.2,   # tambien definidos en "moveit_controllers.yaml"
#        "trajectory_execution.allowed_goal_duration_margin": 0.5,         # tambien definidos en "moveit_controllers.yaml"
#        "trajectory_execution.allowed_start_tolerance": 0.01,             # tambien definidos en "moveit_controllers.yaml"
#    }

#    planning_scene_monitor_parameters = {
#        "publish_planning_scene": True,
#        "publish_geometry_updates": True,
#        "publish_state_updates": True,
#        "publish_transforms_updates": True,
#    }
    
    
#    run_move_group_node = Node(
#        package="moveit_ros_move_group",
#        executable="move_group",
#        output="screen",
#        parameters=[
#            robot_description,
#            robot_description_semantic,
#            kinematics_yaml,
##            joint_limits_yaml,
#            planning_plugin,
#            ompl_planning_pipeline_config,
##            trajectory_execution,
#            moveit_controllers,
#            moveit_simple_controllers_yaml,
#            planning_scene_monitor_parameters,
#            {"use_sim_time": simulado},
#        ],
#        remappings=[(
#         "/joint_states", "/joint_state_broadcaster/joint_states",
#        ),
#        ],
#    )
    
#############################################################  FIN MOVEIT  #########################################################  


    rviz_config_file = (
        get_package_share_directory("arips_arm_description") + "/rviz/rrbot.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
#            robot_description_semantic,
#            kinematics_yaml,
#            ompl_planning_pipeline_config,
#            {"use_sim_time": simulado},   # No me permite usar interactive flechas
#            joint_limits,
        ],
    )


#    calcular = Node(
#        package="descripcion_robot",
#        executable="calculator",
#        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": simulado},],
#        name="calculator",
#        output="screen",
#        remappings=[(
#         "/joint_states", "/joint_state_broadcaster/joint_states",
#        ),
#        ],
#    )
    
##########  ESPERAS    

    
    robot_controller_espera_cargar_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
    
#    robot_hand_espera_cargar_joint_state_broadcaster = RegisterEventHandler(
#        event_handler=OnProcessExit(
#            target_action=joint_state_broadcaster_spawner,
#            on_exit=[robot_controller_spawner_hand],
#        )
#    )
    
    rviz_node_espera_cargar_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )
#    calcular_espera_cargar_joint_state_broadcaster = RegisterEventHandler(
#        event_handler=OnProcessExit(
#            target_action=joint_state_broadcaster_spawner,
#            on_exit=[calcular],
#        )
#    )
    

    nodes = [
#    	IncludeLaunchDescription(
#            PythonLaunchDescriptionSource( [os.path.join(get_package_share_directory('ros_ign_gazebo'),
#                              'launch', 'ign_gazebo.launch.py')]),
#            launch_arguments=[('ign_args', [' -r empty.sdf --force-version 5.4.0'])],
#            condition=IfCondition(simulado),
#        ),
#        cargar_modelo,
        control_node,    ####### NO ES NECESARIO POR QUE EL COMPLEMENTO "ign_ros2_control" CARGA UN controller_manager
#        static_tf, ## si especificamos origen en la etiqueta "world" del URDF, no es necesario
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
#        joint_state_publisher_node,
#        robot_controller_espera_cargar_joint_state_broadcaster,
#        robot_hand_espera_cargar_joint_state_broadcaster,
#        rviz_node_espera_cargar_joint_state_broadcaster,
#	rviz_node,
#	comando,
#	puente,
#	run_move_group_node,
#	calcular_espera_cargar_joint_state_broadcaster,
#	agente,
#	DeclareLaunchArgument(
#            'use_sim_time',
#            default_value=simulado,
#            description='If true, use simulated clock'),
    ]

    return LaunchDescription(declared_arguments + nodes)
#    return LaunchDescription(nodes)
#

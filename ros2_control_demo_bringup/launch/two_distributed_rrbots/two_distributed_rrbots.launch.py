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


from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_complete_namespace(prefix, controller_manager_name):
    return "/" + prefix + "/" + controller_manager_name

def generate_launch_description():
    # TODO(Manuel): find a way to propagate to controllers.yaml
    # otherwise we have to define there two
    satellite_1_namespace_name = "sub_1"
    satellite_2_namespace_name = "sub_2"
    satellite_1_controller_manager_name = generate_complete_namespace(satellite_1_namespace_name, "controller_manager")
    satellite_2_controller_manager_name = generate_complete_namespace(satellite_2_namespace_name, "controller_manager")

    # Get URDF via xacro
    main_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("two_distributed_rrbots_description"),
                    "urdf",
                    "main.urdf.xacro",
                ]
            ),
        ]
    )

    robot_satellite_1_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("two_distributed_rrbots_description"),
                    "urdf",
                    "two_distributed_rrbots_one.urdf.xacro",
                ]
            ),
        ]
    )

    robot_satellite_2_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("two_distributed_rrbots_description"),
                    "urdf",
                    "two_distributed_rrbots_two.urdf.xacro",
                ]
            ),
        ]
    )
    
    # create parameter from description content
    main_robot_description = {"robot_description": main_robot_description_content}
    robot_satellite_1_description = {"robot_description": robot_satellite_1_description_content}
    robot_satellite_2_description = {"robot_description": robot_satellite_2_description_content}

    # Get controller manager settings
    main_robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_demo_bringup"),
            "config/two_distributed_rrbots",
            "main_controllers.yaml",
        ]
    )

    robot_controllers_satellite_1 = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_demo_bringup"),
            "config/two_distributed_rrbots",
            "rrbot_controllers_satellite_1.yaml",
        ]
    )
    
    robot_controllers_satellite_2 = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_demo_bringup"),
            "config/two_distributed_rrbots",
            "rrbot_controllers_satellite_2.yaml",
        ]
    )

    # MAIN CONTROLLER MANAGER
    # Main controller manager node
    main_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[main_robot_description, main_robot_controllers],
        remappings=[
            (
                "/forward_position_controller/commands",
                "/position_commands",
            ),
        ],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[main_robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager" ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/controller_manager"],
    )

    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    # RVIZ
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("two_distributed_rrbots_description"), "config", "two_distributed_rrbots.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # SUBSYSTEMS
    # subsystem 1, satellite controller
    sub_1_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=satellite_1_namespace_name,
        parameters=[robot_satellite_1_description, robot_controllers_satellite_1],
        remappings=[
            (
                "/forward_position_controller/commands",
                "/position_commands",
            ),
        ],
        output="both",
    )

    robot_state_pub_node_1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=satellite_1_namespace_name,
        output="both",
        parameters=[robot_satellite_1_description],
    )

    joint_state_broadcaster_spawner_1 = Node(
        package="controller_manager",
        executable="spawner",
        namespace=satellite_1_namespace_name,
        arguments=["joint_state_broadcaster", "-c", satellite_1_controller_manager_name ],
    )

    robot_controller_spawner_1 = Node(
        package="controller_manager",
        executable="spawner",
        namespace=satellite_1_namespace_name,
        arguments=["forward_position_controller", "-c", satellite_1_controller_manager_name],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner_1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner_1,
            on_exit=[robot_controller_spawner_1],
        )
    )

    # RVIZ
    rviz_config_file_1 = PathJoinSubstitution(
        [FindPackageShare("two_distributed_rrbots_description"), "config", "two_distributed_rrbots_1.rviz"]
    )

    rviz_node_1 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=satellite_1_namespace_name,
        output="log",
        arguments=["-d", rviz_config_file_1],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner_1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner_1,
            on_exit=[rviz_node_1],
        )
    )

    # SUBSYSTEMS
    # subsystem 1, satellite controller

    sub_2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=satellite_2_namespace_name,
        parameters=[robot_satellite_2_description, robot_controllers_satellite_2],
        remappings=[
            (
                "/forward_position_controller/commands",
                "/position_commands",
            ),
        ],
        output="both",
    )

    robot_state_pub_node_2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=satellite_2_namespace_name,
        output="both",
        parameters=[robot_satellite_2_description],
    )

    joint_state_broadcaster_spawner_2 = Node(
        package="controller_manager",
        executable="spawner",
        namespace=satellite_2_namespace_name,
        arguments=["joint_state_broadcaster", "-c", satellite_2_controller_manager_name ],
    )

    robot_controller_spawner_2 = Node(
        package="controller_manager",
        executable="spawner",
        namespace=satellite_2_namespace_name,
        arguments=["forward_position_controller", "-c", satellite_2_controller_manager_name],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner_2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner_2,
            on_exit=[robot_controller_spawner_2],
        )
    )

    # RVIZ  
    rviz_config_file_2 = PathJoinSubstitution(
        [FindPackageShare("two_distributed_rrbots_description"), "config", "two_distributed_rrbots_2.rviz"]
    )

    rviz_node_2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=satellite_2_namespace_name,
        output="log",
        arguments=["-d", rviz_config_file_2],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner_2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner_2,
            on_exit=[rviz_node_2],
        )
    )

    nodes = [
        main_control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        sub_1_control_node,
        robot_state_pub_node_1,
        joint_state_broadcaster_spawner_1,
        delay_rviz_after_joint_state_broadcaster_spawner_1,
        #delay_robot_controller_spawner_after_joint_state_broadcaster_spawner_1,
        sub_2_control_node,
        robot_state_pub_node_2,
        joint_state_broadcaster_spawner_2,
        delay_rviz_after_joint_state_broadcaster_spawner_2,
        #delay_robot_controller_spawner_after_joint_state_broadcaster_spawner_2,
    ]

    return LaunchDescription(nodes)

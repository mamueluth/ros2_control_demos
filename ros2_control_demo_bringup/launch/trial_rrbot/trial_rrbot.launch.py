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

def delete_direct_slash_duplicate(t):
    if(not (t[0] == "/" and t[1] == "/")):
        return t[0]

def prepend_slash_if_not_null(prefix):
    if not prefix:
        return ""
    ns = "/" + prefix
    # remove all occurrences of slashes that directly follow each other ("//Prefix/////Namespace//" -> "/Prefix/Namespace/")
    return ''.join(filter(lambda item: item is not None, map(delete_direct_slash_duplicate , zip(ns,ns[1:] + " ")))) 

def generate_launch_description():
    controller_manager_name = "controller_manager"
    slash_controller_manager_name = prepend_slash_if_not_null(controller_manager_name)
    satellite_1_ns_name = "sub_1"
    slash_satellite_1_ns_name = prepend_slash_if_not_null(satellite_1_ns_name)
    satellite_1_controller_manager_name = slash_satellite_1_ns_name + slash_controller_manager_name

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("trial_rrbot_description"),
                    "urdf",
                    "trial_rrbot.urdf.xacro",
                ]
            ),
            " ",
            "prefix:=sub_1_",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("ros2_control_demo_bringup"),
            "config/trial_rrbot",
            "trial_rrbot_controllers.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("trial_rrbot_description"), "config", "trial_rrbot.rviz"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=satellite_1_ns_name,
        parameters=[robot_description, robot_controllers],
        remappings=[
            (
                "/forward_position_controller/commands",
                "/position_commands",
            ),
            (
                "/joint_states",
                slash_satellite_1_ns_name + "/joint_states"
            ),
            (
                "/dynamic_joint_states",
                slash_satellite_1_ns_name + "/dynamic_joint_states"
            ),
        ],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        namespace=satellite_1_ns_name,
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=satellite_1_ns_name,
        arguments=["joint_state_broadcaster", "-c", satellite_1_controller_manager_name],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=satellite_1_ns_name,
        arguments=["forward_position_controller", "-c", satellite_1_controller_manager_name],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)

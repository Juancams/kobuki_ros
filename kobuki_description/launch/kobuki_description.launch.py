# Copyright 2022 Intelligent Robotics Lab
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
import yaml
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, SetRemap, SetParameter, PushROSNamespace
from launch_ros.descriptions import ParameterValue
from launch.substitutions import (
    AndSubstitution,
    AnySubstitution,
    Command,
    IfElseSubstitution,
    NotSubstitution
)
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, EqualsSubstitution


def modify_yaml_with_namespace(original_yaml_path, namespace, tf_namespace):
    """ Replace all instances of <robot_namespace> in the yaml file
        with the corresponding namespace value.
        This creates a temp file with the result and returns its path.
    """
    with tempfile.NamedTemporaryFile(delete=False, mode='w', encoding='utf-8') as temp_output_yaml:
        temp_yaml_path = temp_output_yaml.name
        with open(original_yaml_path, 'r') as yaml_file:
            key_ns = '<robot_namespace>'
            key_tf = '<tf_namespace>'
            namespace_prefix = f'/{namespace}' if namespace != '' else ''
            tf_prefix = f'/{tf_namespace}' if tf_namespace != '' else ''
            for line in yaml_file:
                if key_ns in line:
                    line = line.replace(key_ns, namespace_prefix)
                if key_tf in line:
                    line = line.replace(key_tf, tf_prefix)
                temp_output_yaml.write(line)
            yaml_data = yaml.safe_load(yaml_file)
    return temp_yaml_path

def start_bridge(context):
    if LaunchConfiguration('gazebo').perform(context) == 'true':
        kobuki_pkg = get_package_share_directory('kobuki_description')

        do_tf_remapping = LaunchConfiguration('do_tf_remapping')

        namespace = LaunchConfiguration('namespace').perform(context)
        tf_namespace = ''
        if namespace != '' and do_tf_remapping.perform(context) == 'true':
            tf_namespace = namespace
        original_yaml_path = os.path.join(
            kobuki_pkg, 'config/bridge', 'kobuki_bridge.yaml'
        )
        modified_yaml_path = modify_yaml_with_namespace(
            original_yaml_path,
            namespace,
            tf_namespace
        )

        bridge = GroupAction([
            SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')),
            PushROSNamespace(namespace=LaunchConfiguration('namespace')),
            SetRemap(condition=IfCondition(do_tf_remapping), src='/tf', dst='tf'),
            SetRemap(condition=IfCondition(do_tf_remapping), src='/tf_static', dst='tf_static'),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='bridge_ros_gz',
                parameters=[
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'config_file': modified_yaml_path,
                        'expand_gz_topic_names': True,
                    }
                ],
                output='screen',
            )
        ])
        return [bridge]

    return []


def start_camera(context):
    if LaunchConfiguration('camera').perform(context) == 'true' and LaunchConfiguration('gazebo').perform(context) == 'true':

        do_tf_remapping = LaunchConfiguration('do_tf_remapping')
        namespace = LaunchConfiguration('namespace').perform(context)

        # Bridge topics must be passed with manual namespaces
        namespace_prefix = f'/{namespace}' if namespace != '' else ''
        image_topic = f'{namespace_prefix}/rgbd_camera/image'
        depth_topic = f'{namespace_prefix}/rgbd_camera/depth_image'
        camera_nodes = GroupAction([
            SetParameter('use_sim_time', LaunchConfiguration('use_sim_time')),
            PushROSNamespace(namespace=LaunchConfiguration('namespace')),
            SetRemap(condition=IfCondition(do_tf_remapping), src='/tf', dst='tf'),
            SetRemap(condition=IfCondition(do_tf_remapping), src='/tf_static', dst='tf_static'),
            Node(
                package='ros_gz_image',
                executable='image_bridge',
                name='bridge_gz_ros_camera_image',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }],
                arguments=[image_topic]
            ),
            Node(
                package='ros_gz_image',
                executable='image_bridge',
                name='bridge_gz_ros_camera_depth',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }],
                arguments=[depth_topic]
            )
        ])

        return [camera_nodes]

    return []

def generate_launch_description():

    kobuki_pkg = get_package_share_directory('kobuki_description')

    lidar_arg = DeclareLaunchArgument(
        'lidar', default_value='false',
        description='Enable lidar sensor')

    lidar_position_x_arg = DeclareLaunchArgument(
        'lidar_position_x', default_value='0',
        description='Lidar X position relative to base_link')

    lidar_position_y_arg = DeclareLaunchArgument(
        'lidar_position_y', default_value='0',
        description='Lidar Y position relative to base_link')

    lidar_position_z_arg = DeclareLaunchArgument(
        'lidar_position_z', default_value='0.37',
        description='Lidar Z position relative to base_link')

    lidar_orientation_roll_arg = DeclareLaunchArgument(
        'lidar_orientation_roll', default_value='0',
        description='Lidar roll orientation')

    lidar_orientation_pitch_arg = DeclareLaunchArgument(
        'lidar_orientation_pitch', default_value='0',
        description='Lidar pitch orientation')

    lidar_orientation_yaw_arg = DeclareLaunchArgument(
        'lidar_orientation_yaw', default_value='0',
        description='Lidar yaw orientation')

    camera_arg = DeclareLaunchArgument(
        'camera', default_value='false',
        description='Enable camera sensor')

    structure_arg = DeclareLaunchArgument(
        'structure', default_value='true',
        description='Enable structure elements')

    gazebo_arg = DeclareLaunchArgument(
        'gazebo', default_value='false',
        description='Enable gazebo plugins')

    description_file = DeclareLaunchArgument(
        'description_file',
        default_value=os.path.join(kobuki_pkg, 'urdf', 'kobuki.urdf.xacro'),
        description='Absolute path to the robot description file'
    )

    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')

    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Namespace to apply to the nodes'
    )

    declare_do_tf_remapping_arg = DeclareLaunchArgument(
        'do_tf_remapping',
        default_value='False',
        description='Whether to remap the tf topic to independent namespaces (/tf -> tf)',
    )

    # Check if the namespace is set and generate the corresponding URDF.
    # If a namespace is used, a trailing '/' is added.
    # Otherwise the node is launched without namespace
    is_empty_namespace = EqualsSubstitution(LaunchConfiguration('namespace'), '')
    do_tf_remapping = LaunchConfiguration('do_tf_remapping')
    tf_namespace = IfElseSubstitution(
        AndSubstitution(AnySubstitution(do_tf_remapping), NotSubstitution(is_empty_namespace)),
        LaunchConfiguration('namespace'),  # Value if true
        ''                                 # Value if false
    )
    robot_model = GroupAction([
        SetRemap(condition=IfCondition(do_tf_remapping), src='/tf', dst='tf'),
        SetRemap(condition=IfCondition(do_tf_remapping), src='/tf_static', dst='tf_static'),
        Node(
            condition=IfCondition(is_empty_namespace),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': ParameterValue(
                    Command([
                        'xacro ', LaunchConfiguration('description_file'),
                        ' lidar:=', LaunchConfiguration('lidar'),
                        ' lidar_position_x:=', LaunchConfiguration('lidar_position_x'),
                        ' lidar_position_y:=', LaunchConfiguration('lidar_position_y'),
                        ' lidar_position_z:=', LaunchConfiguration('lidar_position_z'),
                        ' lidar_orientation_roll:=',
                        LaunchConfiguration('lidar_orientation_roll'),
                        ' lidar_orientation_pitch:=',
                        LaunchConfiguration('lidar_orientation_pitch'),
                        ' lidar_orientation_yaw:=',
                        LaunchConfiguration('lidar_orientation_yaw'),
                        ' camera:=', LaunchConfiguration('camera'),
                        ' structure:=', LaunchConfiguration('structure'),
                        ' gazebo:=', LaunchConfiguration('gazebo')
                    ]), value_type=str),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
        ),
        Node(
            condition=UnlessCondition(is_empty_namespace),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'robot_description': ParameterValue(
                    Command([
                        'xacro ', LaunchConfiguration('description_file'),
                        ' lidar:=', LaunchConfiguration('lidar'),
                        ' lidar_position_x:=', LaunchConfiguration('lidar_position_x'),
                        ' lidar_position_y:=', LaunchConfiguration('lidar_position_y'),
                        ' lidar_position_z:=', LaunchConfiguration('lidar_position_z'),
                        ' lidar_orientation_roll:=',
                        LaunchConfiguration('lidar_orientation_roll'),
                        ' lidar_orientation_pitch:=',
                        LaunchConfiguration('lidar_orientation_pitch'),
                        ' lidar_orientation_yaw:=',
                        LaunchConfiguration('lidar_orientation_yaw'),
                        ' camera:=', LaunchConfiguration('camera'),
                        ' structure:=', LaunchConfiguration('structure'),
                        # Must append the trailing slash when a namespace is active
                        ' namespace:=', LaunchConfiguration('namespace'), '/',
                        ' tf_namespace:=', tf_namespace, '/',
                        ' gazebo:=', LaunchConfiguration('gazebo')
                    ]), value_type=str),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
        ),
        # TF Tree (joint_state_publisher)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        )
    ])

    ld = LaunchDescription()
    ld.add_action(lidar_arg)
    ld.add_action(lidar_position_x_arg)
    ld.add_action(lidar_position_y_arg)
    ld.add_action(lidar_position_z_arg)
    ld.add_action(lidar_orientation_roll_arg)
    ld.add_action(lidar_orientation_pitch_arg)
    ld.add_action(lidar_orientation_yaw_arg)
    ld.add_action(camera_arg)
    ld.add_action(structure_arg)
    ld.add_action(gazebo_arg)
    ld.add_action(description_file)
    ld.add_action(namespace_arg)
    ld.add_action(declare_do_tf_remapping_arg)
    ld.add_action(use_sim_time)
    ld.add_action(robot_model)
    ld.add_action(OpaqueFunction(function=start_bridge))
    ld.add_action(OpaqueFunction(function=start_camera))

    return ld

# Copyright 2021 Clearpath Robotics, Inc.
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
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import os
import shutil
import tempfile

import yaml

from clearpath_config.clearpath_config import ClearpathConfig

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'],
                          description='Start rviz.'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='use_sim_time'),
    DeclareLaunchArgument('world', default_value='warehouse',
                          description='Gazebo World'),
    DeclareLaunchArgument('setup_path',
                          default_value=[EnvironmentVariable('HOME'), '/clearpath/'],
                          description='Clearpath setup path'),
    DeclareLaunchArgument('arm_mode',
                          default_value='dual',
                          choices=['single', 'dual'],
                          description='Launch single arm or dual arm configuration'),
    DeclareLaunchArgument('generate',
                          default_value='true',
                          choices=['true', 'false'],
                          description='Generate parameters and launch files')
]

for pose_element in ['x', 'y', 'yaw']:
    ARGUMENTS.append(DeclareLaunchArgument(pose_element, default_value='0.0',
                     description=f'{pose_element} component of the robot pose.'))

ARGUMENTS.append(DeclareLaunchArgument('z', default_value='0.15',
                 description='z component of the robot pose.'))


def _is_secondary_arm(name: str) -> bool:
    """Return True for arm controllers beyond arm_0."""
    return name.startswith('arm_') and not name.startswith('arm_0_')


def _sanitize_single_arm_control_yaml(setup_path: str) -> None:
    """Remove secondary arm controllers from control.yaml in single-arm mode."""
    control_yaml_path = os.path.join(setup_path, 'platform', 'config', 'control.yaml')
    if not os.path.exists(control_yaml_path):
        return

    with open(control_yaml_path, 'r', encoding='utf-8') as handle:
        control_config = yaml.safe_load(handle) or {}

    updated = False
    for _, namespace_config in control_config.items():
        if not isinstance(namespace_config, dict):
            continue

        # Remove full controller parameter blocks (e.g. arm_1_joint_trajectory_controller).
        for key in list(namespace_config.keys()):
            if _is_secondary_arm(key):
                namespace_config.pop(key, None)
                updated = True

        # Remove controller_manager type declarations for secondary arm controllers.
        controller_manager = namespace_config.get('controller_manager')
        if not isinstance(controller_manager, dict):
            continue
        ros_params = controller_manager.get('ros__parameters')
        if not isinstance(ros_params, dict):
            continue

        for key in list(ros_params.keys()):
            controller_name = key[:-5] if key.endswith('.type') else key
            if _is_secondary_arm(controller_name):
                ros_params.pop(key, None)
                updated = True

    if updated:
        with open(control_yaml_path, 'w', encoding='utf-8') as handle:
            yaml.safe_dump(control_config, handle, sort_keys=False)


def _sanitize_single_arm_setup(setup_path: str) -> None:
    """Apply single-arm cleanup to generated/copied setup artifacts."""
    _sanitize_single_arm_control_yaml(setup_path)


def _sanitize_single_arm_setup_action(context, setup_path: str, *args, **kwargs):
    _sanitize_single_arm_setup(setup_path)
    return []


def _resolve_setup_path(setup_path: str, arm_mode: str) -> str:
    """Build a temporary setup when launching in single-arm mode."""
    if arm_mode != 'single':
        return setup_path

    robot_yaml_path = os.path.join(setup_path, 'robot.yaml')
    if not os.path.exists(robot_yaml_path):
        raise FileNotFoundError(f'robot.yaml not found at {robot_yaml_path}')

    single_arm_setup_path = tempfile.mkdtemp(prefix='clearpath_single_arm_setup_')
    shutil.copytree(setup_path, single_arm_setup_path, dirs_exist_ok=True)

    single_arm_robot_yaml_path = os.path.join(single_arm_setup_path, 'robot.yaml')
    with open(single_arm_robot_yaml_path, 'r', encoding='utf-8') as handle:
        robot_config = yaml.safe_load(handle) or {}

    manipulators = robot_config.get('manipulators') or {}
    arms = manipulators.get('arms') or []
    if arms:
        # Keep only one arm in single mode and center it laterally.
        # Preserve x/z from robot.yaml and only force y to 0.0.
        single_arm = arms[0]
        xyz = single_arm.get('xyz', [0.25, 0.0, 0.005])
        single_arm['xyz'] = [xyz[0], 0.0, xyz[2]]
        manipulators['arms'] = [single_arm]
        robot_config['manipulators'] = manipulators
        with open(single_arm_robot_yaml_path, 'w', encoding='utf-8') as handle:
            yaml.safe_dump(robot_config, handle, sort_keys=False)

    _sanitize_single_arm_setup(single_arm_setup_path)

    return single_arm_setup_path


def launch_setup(context, *args, **kwargs):
    setup_path = LaunchConfiguration('setup_path')
    setup_path_value = str(setup_path.perform(context))
    arm_mode = LaunchConfiguration('arm_mode').perform(context)
    resolved_setup_path = _resolve_setup_path(setup_path_value, arm_mode)
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x, y, z = LaunchConfiguration('x'), LaunchConfiguration('y'), LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    generate = LaunchConfiguration('generate')
    generate_enabled = generate.perform(context).lower() == 'true'

    if arm_mode == 'single' and not generate_enabled:
        raise RuntimeError('arm_mode:=single requires generate:=true')

    # Parse robot YAML into config
    clearpath_config = ClearpathConfig(os.path.join(
        resolved_setup_path, 'robot.yaml'))

    namespace = clearpath_config.system.namespace
    if namespace in ('', '/'):
        robot_name = 'robot'
    else:
        robot_name = namespace + '/robot'

    # Directories
    pkg_clearpath_viz = FindPackageShare('clearpath_viz')

    # Paths
    rviz_launch = PathJoinSubstitution(
        [pkg_clearpath_viz, 'launch', 'view_robot.launch.py'])
    launch_file_platform_service = os.path.join(
        resolved_setup_path, 'platform/launch/platform-service.launch.py')
    launch_file_sensors_service = os.path.join(
        resolved_setup_path, 'sensors/launch/sensors-service.launch.py')

    group_action_spawn_robot = GroupAction([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_platform_service]),
            launch_arguments=[
              ('prefix', ['/world/', world, '/model/', robot_name, '/link/base_link/sensor/'])]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_sensors_service]),
            launch_arguments=[
              ('prefix', ['/world/', world, '/model/', robot_name, '/link/base_link/sensor/'])]
        ),

        # Spawn robot
        Node(
            package='ros_gz_sim',
            executable='create',
            namespace=namespace,
            arguments=['-name', robot_name,
                       '-x', x,
                       '-y', y,
                       '-z', z,
                       '-Y', yaw,
                       '-topic', 'robot_description'],
            output='screen'
        ),
    ])

    node_generate_description = Node(
        package='clearpath_generator_common',
        executable='generate_description',
        name='generate_description',
        output='screen',
        condition=IfCondition(generate),
        arguments=['-s', resolved_setup_path]
    )

    node_generate_semantic_description = Node(
        package='clearpath_generator_common',
        executable='generate_semantic_description',
        name='generate_semantic_description',
        output='screen',
        condition=IfCondition(generate),
        arguments=['-s', resolved_setup_path]
    )

    node_generate_launch = Node(
        package='clearpath_generator_gz',
        executable='generate_launch',
        name='generate_launch',
        output='screen',
        condition=IfCondition(generate),
        arguments=['-s', resolved_setup_path]
    )

    node_generate_param = Node(
        package='clearpath_generator_gz',
        executable='generate_param',
        name='generate_param',
        output='screen',
        condition=IfCondition(generate),
        arguments=['-s', resolved_setup_path]
    )

    event_generate_description = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_generate_description,
            on_exit=[node_generate_semantic_description]
        )
    )

    event_generate_semantic_description = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_generate_semantic_description,
            on_exit=[node_generate_launch]
        )
    )

    event_generate_launch = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_generate_launch,
            on_exit=[node_generate_param]
        )
    )

    spawn_on_exit = [group_action_spawn_robot]
    if arm_mode == 'single':
        spawn_on_exit = [
            OpaqueFunction(
                function=_sanitize_single_arm_setup_action,
                args=[resolved_setup_path],
            ),
            group_action_spawn_robot,
        ]

    event_generate_param = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_generate_param,
            on_exit=spawn_on_exit
        )
    )

    # RViz
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]),
        launch_arguments=[
            ('namespace', namespace),
            ('use_sim_time', use_sim_time)],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    actions = [
        node_generate_description,
        event_generate_description,
        event_generate_semantic_description,
        event_generate_launch,
        event_generate_param,
        rviz
    ]

    if not generate_enabled:
        actions.append(group_action_spawn_robot)

    return actions


def generate_launch_description():
    # Define LaunchDescription variable
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld

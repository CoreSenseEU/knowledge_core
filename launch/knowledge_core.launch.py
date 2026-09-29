# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = 'knowledge_core'

    ld = LaunchDescription()

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace to prepend before /kb (e.g. drone0 → /drone0/kb)')
    ld.add_action(namespace_arg)

    no_socket_arg = DeclareLaunchArgument(
        'no_socket',
        default_value='true',
        description='Disable the socket server mode')
    ld.add_action(no_socket_arg)

    viewer_arg = DeclareLaunchArgument(
        'viewer',
        default_value='false',
        description='Launch the knowledge_viewer web UI and rosbridge (default: false)')
    ld.add_action(viewer_arg)

    namespace = PythonExpression(
        ["'", LaunchConfiguration('namespace'), "/kb' if '",
         LaunchConfiguration('namespace'), "' else 'kb'"])

    try:
        from launch_pal import get_pal_configuration
        config = get_pal_configuration(
            pkg='knowledge_core',
            node='knowledge_core',
            ld=ld)
    except ImportError:
        config = {"parameters": [], "remappings": [], "arguments": []}

    base_args = []
    if config["arguments"]:
        base_args += config["arguments"]

    no_socket_args = base_args + ['--no-socket']
    socket_args = base_args

    knowledge_core_no_socket = Node(
        package='knowledge_core',
        executable='knowledge_core',
        namespace=namespace,
        name='knowledge_core',
        parameters=config["parameters"],
        remappings=config["remappings"],
        arguments=no_socket_args,
        output='both',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('no_socket')),
    )
    ld.add_action(knowledge_core_no_socket)

    knowledge_core_socket = Node(
        package='knowledge_core',
        executable='knowledge_core',
        namespace=namespace,
        name='knowledge_core',
        parameters=config["parameters"],
        remappings=config["remappings"],
        arguments=socket_args,
        output='both',
        emulate_tty=True,
        condition=UnlessCondition(LaunchConfiguration('no_socket')),
    )
    ld.add_action(knowledge_core_socket)

    try:
        from launch_pal import get_pal_configuration
        config = get_pal_configuration(
            pkg='knowledge_core',
            node='knowledge_viewer',
            ld=ld)
    except ImportError:
        config = {"parameters": [], "remappings": [], "arguments": []}

    drone_digits = PythonExpression(
        ["''.join(filter(str.isdigit, '", LaunchConfiguration('namespace'), "')) or '0'"])

    viewer_port = PythonExpression(
        ["str(8010 + int(''.join(filter(str.isdigit, '", LaunchConfiguration('namespace'), "')) or '0'))"])

    rosbridge_port = PythonExpression(
        ["str(9090 + int(''.join(filter(str.isdigit, '", LaunchConfiguration('namespace'), "')) or '0'))"])

    try:
        FindPackageShare('rosbridge_server').perform(None)
        rosbridge_launch = IncludeLaunchDescription(
            XMLLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rosbridge_server'),
                    'launch',
                    'rosbridge_websocket_launch.xml'])
            ]),
            launch_arguments={'port': rosbridge_port}.items(),
            condition=IfCondition(LaunchConfiguration('viewer')),
        )
        ld.add_action(rosbridge_launch)
    except Exception:
        import logging
        logging.getLogger('launch').warning(
            'rosbridge_server not found — knowledge_viewer web UI will not be available. '
            'Install it with: sudo apt install ros-humble-rosbridge-server'
        )

    viewer_args = ['--port', viewer_port, '--rosbridge-port', rosbridge_port]
    if config["arguments"]:
        viewer_args += config["arguments"]

    knowledge_viewer_node = Node(
        package='knowledge_core',
        executable='knowledge_viewer',
        namespace=namespace,
        name='knowledge_viewer',
        parameters=config["parameters"],
        remappings=config["remappings"],
        arguments=viewer_args,
        output='both',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('viewer')),
    )

    ld.add_action(knowledge_viewer_node)

    return ld

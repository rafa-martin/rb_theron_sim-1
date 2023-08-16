# Copyright (c) 2023, Robotnik Automation S.L.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Robotnik Automation S.L.L. nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL Robotnik Automation S.L.L. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from robotnik_common.launch import ExtendedArgument
from robotnik_common.launch import AddArgumentParser


def generate_launch_description():

    ld = LaunchDescription()
    add_to_launcher = AddArgumentParser(ld)

    arg = ExtendedArgument(
        name='gpu',
        description='Set use of GPU',
        default_value='true',
        use_env=True,
        environment='GPU',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='robot_id',
        description='Robot ID',
        default_value='robot',
        use_env=True,
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='robot_description_file',
        description='URDF file to load',
        default_value='default.urdf.xacro',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='pos_x',
        description='X position of the robot',
        default_value='0.0',
        use_env=True,
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='pos_y',
        description='Y position of the robot',
        default_value='0.0',
        use_env=True,
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='pos_z',
        description='Z position of the robot',
        default_value='0.1',
        use_env=True,
    )
    add_to_launcher.add_arg(arg)

    params = add_to_launcher.process_arg()

    # Node to spawn the robot in Gazebo
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('rb_theron_gazebo'),
                    'launch',
                    'description.launch.py'
                )
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'gpu': params['gpu'],
                'robot_id': params['robot_id'],
                'robot_description_file': params['robot_description_file'],
            }.items(),
        )
    )

    ld.add_action(
        Node(
            namespace=params['robot_id'],
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', params['robot_id'],
                '-topic', 'robot_description',
                '-x', params['pos_x'],
                '-y', params['pos_y'],
                '-z', params['pos_z'],
            ],
        )
    )

    ld.add_action(
        Node(
            namespace=params['robot_id'],
            package="controller_manager",
            executable="spawner",
            arguments=["robotnik_base_control"]
        )
    )

    ld.add_action(
        Node(
            namespace=params['robot_id'],
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"]
        )
    )

    return ld

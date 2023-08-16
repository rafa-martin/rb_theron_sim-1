# Copyright (c) 2023, Robotnik Automation S.L.L.
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

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from robotnik_common.launch import ExtendedArgument
from robotnik_common.launch import AddArgumentParser


def generate_launch_description():

    ld = LaunchDescription()
    add_to_launcher = AddArgumentParser(ld)

    arg = ExtendedArgument(
        name='gui',
        description='Set to "false" to run headless.',
        default_value='true',
        use_env=True,
        environment='GUI',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='gpu',
        description='Set use of GPU',
        default_value='true',
        use_env=True,
        environment='GPU',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='verbose',
        description='Enable verbose output',
        default_value='false',
        use_env=True,
        environment='VERBOSE',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='package_gazebo',
        description='Package name of the gazebo world',
        default_value='rb_theron_gazebo',
        use_env=True,
        environment='PACKAGE_GAZEBO',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='gazebo_world',
        description='Name of the gazebo world',
        default_value='default',
        use_env=True,
        environment='GAZEBO_WORLD',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='robot_id',
        description='Robot ID',
        default_value='robot',
        use_env=True,
        environment='ROBOT_ID',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='robot_description_file',
        description='URDF file to load',
        default_value='default.urdf.xacro',
        use_env=True,
        environment='ROBOT_DESCRIPTION_FILE',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='pos_x',
        description='X position of the robot',
        default_value='0.0',
        use_env=True,
        environment='POS_X',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='pos_y',
        description='Y position of the robot',
        default_value='0.0',
        use_env=True,
        environment='POS_Y',
    )
    add_to_launcher.add_arg(arg)

    arg = ExtendedArgument(
        name='pos_z',
        description='Z position of the robot',
        default_value='0.1',
        use_env=True,
        environment='POS_Z',
    )
    add_to_launcher.add_arg(arg)

    params = add_to_launcher.process_arg()

    # Launch gazebo with the world
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    FindPackageShare(
                        'rb_theron_gazebo'
                    ),
                    '/launch/gazebo.launch.py'
                ]
            ),
            launch_arguments={
                'gui': params['gui'],
                'gpu': params['gpu'],
                'verbose': params['verbose'],
                'world_name': params['gazebo_world'],
            }.items()
        )
    )

    # Spawn the robot a
    ld.add_action(
        GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        [
                            FindPackageShare(
                                'rb_theron_gazebo'
                            ),
                            '/launch/spawn.launch.py'
                        ]
                    ),
                    launch_arguments={
                        'robot_id': params['robot_id'],
                        'robot_description_file': params[
                            'robot_description_file'
                        ],
                        'pos_x': params['pos_x'],
                        'pos_y': params['pos_y'],
                        'pos_z': params['pos_z'],
                    }.items(),
                )
            ]
        )
    )

    return ld

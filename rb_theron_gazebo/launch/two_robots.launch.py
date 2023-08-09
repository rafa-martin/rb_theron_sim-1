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

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from lauch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from robotnik_common.launch import add_launch_args
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    ld = LaunchDescription()
    gui_arg = DeclareLaunchArgument(
        name='gui',
        description='Set to "false" to run headless.',
        default_value='true',
    ),
    verbose_arg = DeclareLaunchArgument(
        name='verbose',
        description='Enable verbose output',
        default_value='false',
    )
    gazebo_pkg_arg = DeclareLaunchArgument(
        name='package_gazebo',
        description='Package name of the gazebo world',
        default_value='rb_theron_gazebo'
    )
    world_arg = DeclareLaunchArgument(
        name='gazebo_world',
        description='Name of the gazebo world',
        default_value='default',
    )
    robot_id_arg = DeclareLaunchArgument(
        name='robot_id',
        description='Robot ID',
        default_value='robot',
    )
    robot_des_arg = DeclareLaunchArgument(
        name='robot_description_file',
        description='URDF file to load',
        default_value='default.urdf.xacro',
    )
    pos_x_arg = DeclareLaunchArgument(
        name='pos_x',
        description='X position of the robot',
        default_value='0.0'
    )
    pos_y_arg = DeclareLaunchArgument(
        name='pos_y',
        description='Y position of the robot',
        default_value='0.0'
    )
    pos_z_arg = DeclareLaunchArgument(
        name='pos_z',
        description='Z position of the robot',
        default_value='0.1'
    )
    p = [
        gui_arg,
        verbose_arg,
        gazebo_pkg_arg,
        world_arg,
        # First robot to spawn
        robot_id_arg,
        robot_des_arg,
        pos_x_arg,
        pos_y_arg,
        pos_z_arg,
    ]
    params = add_launch_args(ld, p)

    # Launch gazebo with the world
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    FindPackageShare(
                        params['package_gazebo']
                    ),
                    '/launch/gazebo.launch.py'
                ]
            ),
            launch_arguments={
                'gui': params['gui'],
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
                        'robot_id': 'robot_a',
                        'robot_description_file': 'dual_laser.urdf.xacro',
                        'pos_x': '-1.0',
                        'pos_y': '0.0',
                        'pos_z': '0.1',
                    }.items(),
                )
            ]
        )
    )
    # Spawn the robot b
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
                        'robot_id': 'robot_b',
                        'robot_description_file': 'dual_laser.urdf.xacro',
                        'pos_x': '0.5',
                        'pos_y': '0.86',
                        'pos_z': '0.1',
                    }.items(),
                )
            ]
        )
    )

    return ld

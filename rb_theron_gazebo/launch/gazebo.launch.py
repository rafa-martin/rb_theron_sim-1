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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition


def generate_launch_description():

    ld = LaunchDescription()
    gui_arg = DeclareLaunchArgument(
        name='gui',
        description='Set to "false" to run headless.',
        default_value='true',
    )
    verbose_arg = DeclareLaunchArgument(
        name='verbose',
        description='Enable verbose output',
        default_value='false',
    )
    world_arg = DeclareLaunchArgument(
        name='world_name',
        description='Name of the world to load',
        default_value='rb_theron_office',
    )

    param_list = [
        gui_arg,
        verbose_arg,
        world_arg,
    ]
    params = {}
    for arg in param_list:
        name = arg.name
        ld.add_action(arg)
        params[name] = LaunchConfiguration(arg.name)

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch',
                    'gzserver.launch.py'
                )
            ),
            launch_arguments={
                'verbose': LaunchConfiguration('verbose'),
                'world': [
                    get_package_share_directory('rb_theron_gazebo'),
                    '/worlds/',
                    LaunchConfiguration('world_name'),
                    '.world'
                ],
                'paused': 'false',
                'init': 'true',
                'factory': 'true',
                'force_system': 'true',
                'params_file': [
                    get_package_share_directory('rb_theron_gazebo'),
                    '/config/gazebo.yaml'
                ],
            }.items(),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch',
                    'gzclient.launch.py'
                )
            ),
            launch_arguments={
                'verbose': LaunchConfiguration('verbose'),
            }.items(),
            condition=IfCondition(
                LaunchConfiguration('gui')
            )
        )
    )

    return ld

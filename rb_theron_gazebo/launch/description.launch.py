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
from launch_ros.substitutions import FindPackageShare

from robotnik_common.launch import RewrittenYaml


def read_params(
    ld: LaunchDescription,
    params: list[
        tuple[
            str,
            str,
            str,
        ]
    ]
):
    # name, description, default_value

    # Declare the launch options
    for param in params:
        arg = DeclareLaunchArgument(
            name=param[0],
            description=param[1],
            default_value=param[2],
        )
        ld.add_action(arg)

    # Get the launch configuration variables
    ret = {}
    for param in params:
        ret[param[0]] = LaunchConfiguration(param[0])

    return ret


def generate_launch_description():

    ld = LaunchDescription()
    use_sime_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        description='Use simulation (Gazebo) clock if true',
        default_value='true',
    )
    robot_id_arg = DeclareLaunchArgument(
        name='robot_id',
        description='Robot ID',
        default_value='robot',
    )
    controller_path_arg = DeclareLaunchArgument(
        name='controller_path',
        description='Path of controllers',
        default_value=[
            FindPackageShare('rb_theron_gazebo'),
            '/config/controller.yaml',
        ],
    )
    p = [
        use_sime_time_arg,
        robot_id_arg,
        controller_path_arg,
    ]
    params = read_params(ld, p)

    config_file_rewritten = RewrittenYaml(
        source_file=params['controller_path'],
        param_rewrites={
            'left_wheel_names': [
                '[\'left_wheel_joint',
                '\']'
            ],
            'right_wheel_names': [
                '[\'right_wheel_joint',
                '\']'
            ],
            'odom_frame_id': ['odom'],
            'base_frame_id': ['base_footprint'],
        },
        root_key=[
            params['robot_id'],
        ],
        convert_types=True,
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('rb_theron_description'),
                    'launch',
                    'default.launch.py'
                )
            ),
            launch_arguments={
                'environment': 'false',
                'use_sim_time': params['use_sim_time'],
                'robot_id': params['robot_id'],
                'controller_path': config_file_rewritten,
            }.items()
        )
    )

    return ld

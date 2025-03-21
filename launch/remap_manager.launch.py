# Copyright (c) 2025 PAL Robotics S.L. All rights reserved.
#
# Unauthorized copying of this file, via any medium is strictly prohibited,
# unless it was supplied under the terms of a license agreement or
# nondisclosure agreement with PAL Robotics SL. In this case it may not be
# copied or disclosed except in accordance with the terms of that agreement.

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_pal import get_pal_configuration
from launch_ros.actions import Node

def generate_launch_description():
    pkg = 'remap_manager'
    name = 'remap_manager'
    ld = LaunchDescription()
    config = get_pal_configuration(pkg=pkg, node=name, ld=ld)

    node = Node(
        package=pkg,
        executable='remap_manager_node',
        namespace='',
        name=name,
        parameters=config["parameters"],
        remappings=config["remappings"],
        arguments=config["arguments"],
        output='both', emulate_tty=True)

    ld.add_action(node)
    return ld

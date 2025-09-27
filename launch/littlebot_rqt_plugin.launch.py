import os
import argparse
import sys
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gui_node = Node(
        package="littlebot_rqt_plugin",
        executable="littlebot_rqt_plugin",
        name="littlebot_gui",
        output="screen",
        arguments=["--force-discover"],
    )
    return LaunchDescription([gui_node])

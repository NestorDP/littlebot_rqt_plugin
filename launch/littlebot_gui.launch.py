import os
import argparse
import sys
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    gui_node = Node(
        package="littlebot_gui",
        executable="littlebot_gui",
        name="littlebot_gui",
        output="screen",
    )
    return LaunchDescription([gui_node])

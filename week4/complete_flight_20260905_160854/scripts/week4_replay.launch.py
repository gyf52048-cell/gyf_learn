"""Replay and validate the completed flight in isolated ROS domain 74."""
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([ExecuteProcess(cmd=['python3','/home/drone/week4_debug/scripts/replay_verify.py'],output='screen',additional_env={'ROS_DOMAIN_ID':'74'})])

from launch import LaunchDescription
from launch_ros.actions import Node

import os

def generate_launch_description():
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_microxrcedds'

    return LaunchDescription([
        Node( package='caddybot_mcu',
            executable='mcu',
            name='mcu',
            namespace='caddybot_mcu',
            output='screen',
            emulate_tty=True,
        )
    ])


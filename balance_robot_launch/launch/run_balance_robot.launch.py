from launch import LaunchDescription
from launch_ros.actions import Node

_NAMESPACE = '/balance_robot/'
_DEBUG = True

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package = 'arduino_interface',
        #     namespace = _NAMESPACE,
        #     executable = 'arduino_interface_node',
        #     name = 'arduino_interface',
        #     # remappings = [
        #     # ],
        #     # parameters = [
        #     # ]
        # ),
        Node(
            package = 'joy',
            namespace = _NAMESPACE,
            executable = 'joy_node',
            name = 'joy'
        ),
        Node(
            package = 'led_driver',
            namespace = _NAMESPACE,
            executable = 'neopixel_driver',
            name = 'neopixel_driver',
            output = 'screen',
            remappings = [
                ('joy/in','joy')
            ],
            parameters = [
                {'debug': _DEBUG},
                {'num_neopixels': 16},
                {'neopixel_brightness': 1.0}
            ]
        )
    ])
# General Imports
import board
import neopixel

# ROS Imports
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

class NeopixelDriver(Node):

    def __init__(self):
        super().__init__('neopixel_driver')
        self.declare_parameter('debug',False)
        self.debug = self.get_parameter('debug').value
        self.declare_parameter('timer_hz',5)
        self.timer_hz = self.get_parameter('timer_hz').value
        
        # Subscribers
        self.create_subscription(Joy,'joy/in',self.joy_callback,1)

        # Timer
        self.timer = self.create_timer(self.timer_hz**-1,self.timer_callback)

        # Setup Neopixels
        self.declare_parameter('num_neopixels',16)
        self.num_neopixels = self.get_parameter('num_neopixels').value
        self.declare_parameter('neopixel_brightness',1.0)
        self.neopixel_brightness = self.get_parameter('neopixel_brightness').value
        self.pixels = neopixel.NeoPixel(board.D18,self.num_neopixels,brightness=self.neopixel_brightness,pixel_order=neopixel.RGBW)
        self.gamepad = {}
        self.gamepad_previous = {}
        self.neopixel_enable = False
        self.neopixel_color = (0,0,0,255)

    def timer_callback(self):
        if self.neopixel_enable:
            if self.debug:
                print('Showing neopixels')
                print('  color:',self.neopixel_color,flush=True)
            self.pixels.fill(self.neopixel_color)
            self.pixels.show()

    def joy_callback(self,msg):
        temp_stamp_sec = msg.header.stamp.sec
        temp_stamp_nanosec = msg.header.stamp.nanosec
        temp_frame_id = msg.header.frame_id
        temp_axes = msg.axes
        temp_buttons = msg.buttons

        # Assign Axes
        self.gamepad['left_stick_H_axis'] = temp_axes[0] # Positive Left
        self.gamepad['left_stick_V_axis'] = temp_axes[1] # Positive Up
        self.gamepad['right_stick_H_axis'] = temp_axes[3] # Positive Left
        self.gamepad['right_stick_V_axis'] = temp_axes[4] # Positive Up
        self.gamepad['dpad_H_axis'] = temp_axes[6] # Positive Left
        self.gamepad['dpad_V_axis'] = temp_axes[7] # Positive Up
        self.gamepad['left_trigger'] = temp_axes[2] # Positive 1.0 at rest, -1.0 depressed
        self.gamepad['right_trigger'] = temp_axes[5] # Positive 1.0 at rest, -1.0 depressed

        # Assign Buttons
        self.gamepad['A_button'] = temp_buttons[0] # 0 off, 1 on
        if 'A_button' in self.gamepad_previous:
            if self.gamepad_previous['A_button'] != self.gamepad['A_button']:
                if self.gamepad['A_button']:
                    self.A_button_callback()
                self.gamepad_previous['A_button'] = self.gamepad['A_button']
        else:
            self.gamepad_previous['A_button'] = self.gamepad['A_button']

        self.gamepad['B_button'] = temp_buttons[1] # 0 off, 1 on
        if 'B_button' in self.gamepad_previous:
            if self.gamepad_previous['B_button'] != self.gamepad['B_button']:
                if self.gamepad['B_button']:
                    self.B_button_callback()
                self.gamepad_previous['B_button'] = self.gamepad['B_button']
        else:
            self.gamepad_previous['B_button'] = self.gamepad['B_button']

        self.gamepad['X_button'] = temp_buttons[2] # 0 off, 1 on
        if 'X_button' in self.gamepad_previous:
            if self.gamepad_previous['X_button'] != self.gamepad['X_button']:
                if self.gamepad['X_button']:
                    self.X_button_callback()
                self.gamepad_previous['X_button'] = self.gamepad['X_button']
        else:
            self.gamepad_previous['X_button'] = self.gamepad['X_button']

        self.gamepad['Y_button'] = temp_buttons[3] # 0 off, 1 on
        if 'Y_button' in self.gamepad_previous:
            if self.gamepad_previous['Y_button'] != self.gamepad['Y_button']:
                if self.gamepad['Y_button']:
                    self.Y_button_callback()
                self.gamepad_previous['Y_button'] = self.gamepad['Y_button']
        else:
            self.gamepad_previous['Y_button'] = self.gamepad['Y_button']

        self.gamepad['start_button'] = temp_buttons[7] # 0 off, 1 on
        if 'start_button' in self.gamepad_previous:
            if self.gamepad_previous['start_button'] != self.gamepad['start_button']:
                if self.gamepad['start_button']:
                    self.start_button_callback()
                self.gamepad_previous['start_button'] = self.gamepad['start_button']
        else:
            self.gamepad_previous['start_button'] = self.gamepad['start_button']

        self.gamepad['back_button'] = temp_buttons[6] # 0 off, 1 on
        if 'back_button' in self.gamepad_previous:
            if self.gamepad_previous['back_button'] != self.gamepad['back_button']:
                if self.gamepad['back_button']:
                    self.back_button_callback()
                self.gamepad_previous['back_button'] = self.gamepad['back_button']
        else:
            self.gamepad_previous['back_button'] = self.gamepad['back_button']

        self.gamepad['left_bumper'] = temp_buttons[4] # 0 off, 1 on
        if 'left_bumper' in self.gamepad_previous:
            if self.gamepad_previous['left_bumper'] != self.gamepad['left_bumper']:
                if self.gamepad['left_bumper']:
                    self.left_bumper_callback()
                self.gamepad_previous['left_bumper'] = self.gamepad['left_bumper']
        else:
            self.gamepad_previous['left_bumper'] = self.gamepad['left_bumper']

        self.gamepad['right_bumper'] = temp_buttons[5] # 0 off, 1 on
        if 'right_bumper' in self.gamepad_previous:
            if self.gamepad_previous['right_bumper'] != self.gamepad['right_bumper']:
                if self.gamepad['right_bumper']:
                    self.right_bumper_callback()
                self.gamepad_previous['right_bumper'] = self.gamepad['right_bumper']
        else:
            self.gamepad_previous['right_bumper'] = self.gamepad['right_bumper']

        self.gamepad['left_stick_button'] = temp_buttons[9] # 0 off, 1 on
        if 'left_stick_button' in self.gamepad_previous:
            if self.gamepad_previous['left_stick_button'] != self.gamepad['left_stick_button']:
                if self.gamepad['left_stick_button']:
                    self.left_stick_button_callback()
                self.gamepad_previous['left_stick_button'] = self.gamepad['left_stick_button']
        else:
            self.gamepad_previous['left_stick_button'] = self.gamepad['left_stick_button']

        self.gamepad['right_stick_button'] = temp_buttons[10] # 0 off, 1 on
        if 'right_stick_button' in self.gamepad_previous:
            if self.gamepad_previous['right_stick_button'] != self.gamepad['right_stick_button']:
                if self.gamepad['right_stick_button']:
                    self.right_stick_button_callback()
                self.gamepad_previous['right_stick_button'] = self.gamepad['right_stick_button']
        else:
            self.gamepad_previous['right_stick_button'] = self.gamepad['right_stick_button']

    def A_button_callback(self):
        if self.debug:
            print('A button callback',flush=True)
        self.neopixel_color = (0,255,0,0)

    def B_button_callback(self):
        if self.debug:
            print('B button callback',flush=True)
        self.neopixel_color = (255,0,0,0)

    def X_button_callback(self):
        if self.debug:
            print('X button callback',flush=True)
        self.neopixel_color = (0,0,255,0)

    def Y_button_callback(self):
        if self.debug:
            print('Y button callback',flush=True)
        self.neopixel_color = (0,0,0,255)

    def start_button_callback(self):
        if self.debug:
            print('Start button callback',flush=True)
        if self.neopixel_enable:
            self.neopixel_enable = False
        else:
            self.neopixel_enable = True

    def back_button_callback(self):
        if self.debug:
            print('Back button callback',flush=True)

    def left_bumper_callback(self):
        if self.debug:
            print('LB button callback',flush=True)

    def right_bumper_callback(self):
        if self.debug:
            print('RB button callback',flush=True)

    def left_stick_button_callback(self):
        if self.debug:
            print('LS button callback',flush=True)

    def right_stick_button_callback(self):
        if self.debug:
            print('RS button callback',flush=True)


def main():
    rclpy.init()
    node = NeopixelDriver()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
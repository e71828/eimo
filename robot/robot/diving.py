#!/bin/python3

""""depth_control.py"""

import rclpy
from rclpy.node import Node
from eimo_msgs.srv import Scl
from time import sleep
from eimo_msgs.msg import Control, Depth
from simple_pid import PID

class DepthControl(Node):
    def __init__(self):
        super().__init__('depth_control')
        self.current_depth = None
        self.controlling_flag = False
        self.controlling_flag_old = False
        self.declare_parameter('~weight_compensate', 10000)
        self.declare_parameter('init_depth', 10000)

        self.weight_compensate = self.get_parameter('~weight_compensate').get_parameter_value().string_value
        self.base_output = 0

        sleep(3)  # wait here for 3 seconds

        # my_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.cli = self.create_client(Scl, 'scl_passthrough')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.config_QA = Scl.Request()

        config_echo = self.scl_request('DL1')
        if config_echo == '%\r':
            self.get_logger().info("Motor Define CW-limit and CCW-limit OK with DL1.")
        config_echo = self.scl_request('ME')
        if config_echo == '%\r':
            self.get_logger().info("Motor enable OK with ME.")

        self.scl_config('DC2000')
        self.scl_config('DI400000')
        self.scl_config('FS1H')
        sleep(3)
        config_echo = self.scl_request('IP')
        if len(config_echo) > 2 and 'IP=' in config_echo:
            index = config_echo.index('IP=')
            self.cw_limit = int(config_echo[index + 3:index + 11], 16)
            if self.cw_limit & (1 << (32 - 1)):
                self.cw_limit -= 1 << 32
            self.get_logger().info(f"cw_limit position: {self.cw_limit}")

        self.scl_config('DI-400000')
        self.scl_config('FS2H')
        sleep(3)
        config_echo = self.scl_request('IP')
        if len(config_echo) > 2 and 'IP=' in config_echo:
            index = config_echo.index('IP=')
            self.ccw_limit = int(config_echo[index + 3:index + 11], 16)
            if self.ccw_limit & (1 << (32 - 1)):
                self.ccw_limit -= 1 << 32
            self.get_logger().info(f"ccw_limit position: {self.ccw_limit}")

        self.scl_config('FP' + str(int(self.ccw_limit / 2 + self.cw_limit / 2)))
        sleep(2)
        self.scl_config('SP0')
        self.get_logger().info(f"reset middle position")

        self.setpoint_depth = self.get_parameter('init_depth').get_parameter_value().integer_value + 400

        self.pid = PID(300, 1, 200, setpoint=self.setpoint_depth)
        self.pid.sample_time = 1 / self.get_parameter('depth_frequency').get_parameter_value().integer_value
        self.pid.output_limits = (-180000, 180000)

        self.sub_control = self.create_subscription(Control, 'control', self.deal_control_cmd)
        self.sub_depth = self.create_subscription(Depth, 'depth', self.keep_depth)

    def __del__(self):
        self.scl_config('FP0')
        self.sub_control.destroy()
        self.sub_depth.destroy()
        self.get_logger().info('Welcome Back to Middle Position')

    def scl_config(self, config_str):
        self.config_QA.request = config_str
        self.cli.call_async(self.req)
        rclpy.spin_once(self)

    def scl_request(self, config_str):
        self.config_QA.request = config_str
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        config_echo = response.answer
        if config_echo == '%\r':
            self.gget_logger().debug(f'Config with string: {config_str}, result: OK')
        return config_echo

    def deal_control_cmd(self, cmd):
            self.weight_compensate = self.get_parameter('~weight_compensate').get_parameter_value().string_value
            if cmd.light1 and cmd.light2 and cmd.up:
                self.scl_config('SKD') if self.controlling_flag_old != self.controlling_flag else None
                self.controlling_flag_old = self.controlling_flag
                self.controlling_flag = True
                self.scl_config('DI-50000')
                self.scl_config('FS2H')
                self.setpoint_depth -= 100
            elif cmd.light1 and cmd.light2 and cmd.down:
                self.scl_config('SKD') if self.controlling_flag_old != self.controlling_flag else None
                self.controlling_flag_old = self.controlling_flag
                self.controlling_flag = True
                self.scl_config('DI50000')
                self.scl_config('FS1H')
                self.setpoint_depth += 100
            elif cmd.up and not cmd.down:
                self.controlling_flag_old = self.controlling_flag
                self.controlling_flag = False
                self.setpoint_depth -= 10
            elif cmd.down and not cmd.up:
                self.controlling_flag_old = self.controlling_flag
                self.controlling_flag = False
                self.setpoint_depth += 10
            else:
                self.controlling_flag_old = self.controlling_flag
                self.controlling_flag = False
            if self.setpoint_depth < 0:
                self.setpoint_depth = 0

    def keep_depth(self, msg):
        self.current_depth = msg.depth_mm
        if not self.controlling_flag and not self.controlling_flag_old:
            self.pid.setpoint = self.setpoint_depth
            output = int(self.pid(self.current_depth)) + self.weight_compensate
            self.get_logger().info(f'setpoint_depth: {self.setpoint_depth}')
            self.get_logger().info(f'current  depth: {msg.depth_mm}')
            # self.get_logger().info(f'current output: {output}')
            self.scl_config('FP' + str(output))


def main(arg=None):
    rclpy.init(args=arg)
    depth_control_node = DepthControl()
    rclpy.spin(depth_control_node)
    depth_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
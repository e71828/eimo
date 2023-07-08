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

        self.cli = self.create_client(Scl, 'scl_passthrough')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            
        self.config_QA = Scl.Request()
        self.config_QA.request = 'DL1'
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        config_echo = response.answer
        if config_echo == '%\r':
            self.get_logger().info("Motor Define CW-limit and CCW-limit OK with DL1.")
        self.config_QA.request = 'ME'
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        config_echo = response.answer
        if config_echo == '%\r':
            self.get_logger().info("Motor enable OK with ME.")

        self.config('DC2000')
        self.config('DI400000')
        self.config('FS1H')
        sleep(3)
        self.config_QA.request = 'IP'
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        config_echo = response.answer
        if len(config_echo) > 2 and 'IP=' in config_echo:
            index = config_echo.index('IP=')
            self.cw_limit = int(config_echo[index + 3:index + 11], 16)
            if self.cw_limit & (1 << (32 - 1)):
                self.cw_limit -= 1 << 32
            self.get_logger().info(f"cw_limit position: {self.cw_limit}")

        self.config('DI-400000')
        self.config('FS2H')
        sleep(3)
        self.config_QA.request = 'IP'
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        config_echo = response.answer
        if len(config_echo) > 2 and 'IP=' in config_echo:
            index = config_echo.index('IP=')
            self.ccw_limit = int(config_echo[index + 3:index + 11], 16)
            if self.ccw_limit & (1 << (32 - 1)):
                self.ccw_limit -= 1 << 32
            self.get_logger().info(f"ccw_limit position: {self.ccw_limit}")

        self.config('FP' + str(int(self.ccw_limit/2+self.cw_limit/2)))
        sleep(2)
        self.config('SP0')
        self.get_logger().info(f"reset middle position")

        self.setpoint_depth = self.get_parameter('init_depth').get_parameter_value().integer_value + 400

        self.pid = PID(300, 1, 200, setpoint=self.setpoint_depth)
        self.pid.sample_time = 1 / self.get_parameter('depth_frequency').get_parameter_value().integer_value
        self.pid.output_limits = (-180000, 180000)

        self.sub_control = self.create_subscription(Control, 'control', self.diving, 1, queue_size=3)
        self.sub_depth = self.create_subscription(Depth, 'depth', self.diving, 2, queue_size=3)

    def __del__(self):
        self.config('FP0')
        self.sub_control.destroy()
        self.sub_depth.destroy()
        self.get_logger().info('Welcome Back to Middle Position')

    def config(self, config_str):
        self.config_QA.request = config_str
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        config_echo = response.answer
        if config_echo == '%\r':
            self.gget_logger().debug(f'Config with string: {config_str}, result: OK')
        pass

    def diving(self, data, args):
        if args == 1:
            self.weight_compensate = self.get_parameter('~weight_compensate').get_parameter_value().string_value
            if data.light1 and data.light2 and data.up:
                self.config('SKD') if self.controlling_flag_old != self.controlling_flag else None
                self.controlling_flag_old = self.controlling_flag
                self.controlling_flag = True
                self.config('DI-50000')
                self.config('FS2H')
                self.setpoint_depth -= 100
            elif data.light1 and data.light2 and data.down:
                self.config('SKD') if self.controlling_flag_old != self.controlling_flag else None
                self.controlling_flag_old = self.controlling_flag
                self.controlling_flag = True
                self.config('DI50000')
                self.config('FS1H')
                self.setpoint_depth += 100
            elif data.up and not data.down:
                self.controlling_flag_old = self.controlling_flag
                self.controlling_flag = False
                self.setpoint_depth -= 10
            elif data.down and not data.up:
                self.controlling_flag_old = self.controlling_flag
                self.controlling_flag = False
                self.setpoint_depth += 10
            else:
                self.controlling_flag_old = self.controlling_flag
                self.controlling_flag = False
            if self.setpoint_depth < 0:
                self.setpoint_depth = 0
        elif args == 2:
            self.current_depth = data.depth_mm
            if not self.controlling_flag and not self.controlling_flag_old:
                self.pid.setpoint = self.setpoint_depth
                output = int(self.pid(self.current_depth)) + self.weight_compensate
                self.get_logger().info(f'setpoint_depth: {self.setpoint_depth}')
                self.get_logger().info(f'current  depth: {data.depth_mm}')
                # self.get_logger().info(f'current output: {output}')
                self.config('FP' + str(output))
        else:
            pass


def main(arg=None):
    rclpy.init(args=arg)
    depth_control_node = DepthControl()
    rclpy.spin(depth_control_node)
    depth_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
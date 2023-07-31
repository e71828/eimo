#!/bin/python3

""""depth_control.py"""

import rclpy
from rclpy.node import Node
from eimo_msgs.srv import Scl
from time import sleep
from eimo_msgs.msg import Control, Depth
from simple_pid import PID
from rcl_interfaces.srv import GetParameters


class DepthControl(Node):
    def __init__(self):
        super().__init__('depth_control')
        self.current_depth = None
        self.controlling_flag = False
        self.controlling_flag_old = False
        self.declare_parameter('~weight_compensate', 10000)

        self.weight_compensate = self.get_parameter('~weight_compensate').get_parameter_value().string_value
        self.base_output = 0

        sleep(3)  # wait here for 3 seconds

        class SCL(Node):
            def __init__(self):
                super().__init__('scl_control')
                self.response = None
                self.future = None
                my_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
                self.client_scl = self.create_client(Scl, 'scl_passthrough', callback_group=my_callback_group)
                while not self.client_scl.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('service not available, waiting again...')
                self.config_QA = Scl.Request()

            def config(self, config_str):
                self.config_QA.request = config_str
                self.client_scl.call_async(self.config_QA)
                rclpy.spin_once(self)

            def request(self, config_str):
                self.config_QA.request = config_str
                self.future = self.client_scl.call_async(self.config_QA)
                rclpy.spin_until_future_complete(self, self.future)
                self.response = self.future.result()
                config_echo = self.response.answer
                if config_echo == '%\r':
                    self.get_logger().debug(f'Config with string: {config_str}, result: OK')
                return config_echo
        self.scl = SCL()        
        echo = self.scl.request('DL1')
        if echo == '%\r':
            self.get_logger().info("Motor Define CW-limit and CCW-limit OK with DL1.")
        echo = self.scl.request('ME')
        if echo == '%\r':
            self.get_logger().info("Motor enable OK with ME.")

        self.scl.config('DC2000')
        self.scl.config('DI400000')
        self.scl.config('FS1H')
        sleep(3)
        echo = self.scl.request('IP')
        if len(echo) > 2 and 'IP=' in echo:
            index = echo.index('IP=')
            self.cw_limit = int(echo[index + 3:index + 11], 16)
            if self.cw_limit & (1 << (32 - 1)):
                self.cw_limit -= 1 << 32
            self.get_logger().info(f"cw_limit position: {self.cw_limit}")

        self.scl.config('DI-400000')
        self.scl.config('FS2H')
        sleep(3)
        echo = self.scl.request('IP')
        if len(echo) > 2 and 'IP=' in echo:
            index = echo.index('IP=')
            self.ccw_limit = int(echo[index + 3:index + 11], 16)
            if self.ccw_limit & (1 << (32 - 1)):
                self.ccw_limit -= 1 << 32
            self.get_logger().info(f"ccw_limit position: {self.ccw_limit}")

        self.scl.config('FP' + str(int(self.ccw_limit / 2 + self.cw_limit / 2)))
        sleep(2)
        self.scl.config('SP0')
        self.get_logger().info(f"reset middle position")

        class GetExternalParam(Node):
            def __init__(self):
                super().__init__('fetch_depth_parameter_node')
                self.future = None
                self.cli = self.create_client(GetParameters, '/' + 'publish_depth' + '/get_parameters')
                while not self.cli.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('service not available, waiting again...')
                self.req = GetParameters.Request()

            def get_param(self):
                self.req.names = ['init_depth', 'depth_frequency']
                self.future = get_param.cli.call_async(self.req)
        get_param = GetExternalParam()
        get_param.get_param()

        rclpy.spin_until_future_complete(get_param, get_param.future)
        response = get_param.future.result()
        self.init_depth = response.values[0].integer_value
        self.depth_frequency = response.values[1].integer_value
        get_param.destroy_node()

        self.setpoint_depth = self.init_depth + 400

        self.pid = PID(300, 1, 200, setpoint=self.setpoint_depth)
        self.pid.sample_time = 1 / self.depth_frequency
        self.pid.output_limits = (-180000, 180000)

        self.sub_control = self.create_subscription(Control, 'control', self.deal_control_cmd, 1)
        self.sub_depth = self.create_subscription(Depth, 'depth', self.keep_depth, 1)

    def __del__(self):
        self.scl.config('FP0')
        self.sub_control.destroy()
        self.sub_depth.destroy()
        self.scl.destroy_node()
        self.get_logger().info('Welcome Back to Middle Position')



    def deal_control_cmd(self, cmd):
            self.weight_compensate = self.get_parameter('~weight_compensate').get_parameter_value().string_value
            if cmd.light1 and cmd.light2 and cmd.up:
                self.scl.config('SKD') if self.controlling_flag_old != self.controlling_flag else None
                self.controlling_flag_old = self.controlling_flag
                self.controlling_flag = True
                self.scl.config('DI-50000')
                self.scl.config('FS2H')
                self.setpoint_depth -= 100
            elif cmd.light1 and cmd.light2 and cmd.down:
                self.scl.config('SKD') if self.controlling_flag_old != self.controlling_flag else None
                self.controlling_flag_old = self.controlling_flag
                self.controlling_flag = True
                self.scl.config('DI50000')
                self.scl.config('FS1H')
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
            self.scl.config('FP' + str(output))


def main(arg=None):
    rclpy.init(args=arg)
    depth_control_node = DepthControl()
    rclpy.spin(depth_control_node)
    depth_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
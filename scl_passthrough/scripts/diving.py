#!/usr/bin/env python3

""""depth_control.py"""

import rospy
from eimo_msgs.srv import scl
from eimo_msgs.msg import control, depth
from time import sleep
from simple_pid import PID


class DepthControl:
    def __init__(self):
        self.current_depth = None
        self.setpoint_depth = rospy.get_param('~init_setpoint_depth', default=400)
        self.controlling_flag = False
        self.controlling_flag_old = False
        self.base_output = 0
        self.pid = PID(300, 30, 500, setpoint=self.setpoint_depth)
        self.pid.sample_time = 1  # Update every 1 seconds
        self.pid.output_limits = (-180000, 180000)

        rospy.init_node('depth_control', anonymous=True)
        rospy.wait_for_service('scl_passthrough')
        self.control = rospy.ServiceProxy('scl_passthrough', scl)
        config_QA = self.control('DL1')
        config_echo = config_QA.Answer
        if config_echo == '%\r':
            rospy.loginfo("Motor Define CW-limit and CCW-limit OK with DL1.")
        config_QA = self.control('ME')
        config_echo = config_QA.Answer
        if config_echo == '%\r':
            rospy.loginfo("Motor enable OK with ME.")

        self.config('DC2000')
        self.config('DI400000')
        self.config('FS1H')
        sleep(3)
        config_QA = self.control('IP')
        config_echo = config_QA.Answer
        if len(config_echo) > 2 and 'IP=' in config_echo:
            index = config_echo.index('IP=')
            self.cw_limit = int(config_echo[index + 3:index + 11], 16)
            if self.cw_limit & (1 << (32 - 1)):
                self.cw_limit -= 1 << 32
            rospy.loginfo(f"cw_limit position: {self.cw_limit}")

        self.config('DI-400000')
        self.config('FS2H')
        sleep(3)
        config_QA = self.control('IP')
        config_echo = config_QA.Answer
        if len(config_echo) > 2 and 'IP=' in config_echo:
            index = config_echo.index('IP=')
            self.ccw_limit = int(config_echo[index + 3:index + 11], 16)
            if self.ccw_limit & (1 << (32 - 1)):
                self.ccw_limit -= 1 << 32
            rospy.loginfo(f"ccw_limit position: {self.ccw_limit}")

        self.config('FP' + str(int(self.ccw_limit/2+self.cw_limit/2)))
        sleep(2)
        self.config('SP0')
        rospy.loginfo(f"reset middle position")
        rospy.on_shutdown(self.go_back_to_mid_position)

        self.sub_control = rospy.Subscriber('control', control, self.diving, 1, queue_size=3)
        self.sub_depth = rospy.Subscriber('depth', depth, self.diving, 2, queue_size=3)

        rospy.spin()

    def go_back_to_mid_position(self):
        self.config('FP0')
        self.sub_control.unregister()
        self.sub_depth.unregister()
        rospy.loginfo('Welcome Back to Home')

    def config(self, config_str):
        config_QA = self.control(config_str)
        config_echo = config_QA.Answer
        if config_echo == '%\r':
            rospy.logdebug(f'Config with string: {config_str}, result: OK')
        pass

    def diving(self, data, args):
        if args == 1:
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
                self.setpoint_depth -= 20
            elif data.down and not data.up:
                self.controlling_flag_old = self.controlling_flag
                self.controlling_flag = False
                self.setpoint_depth += 20
            else:
                self.controlling_flag_old = self.controlling_flag
                self.controlling_flag = False
        elif args == 2:
            self.current_depth = data.depth_mm
            if not self.controlling_flag and not self.controlling_flag_old:
                self.pid.setpoint = self.setpoint_depth
                output = int(self.pid(self.current_depth))
                rospy.loginfo(f'setpoint_depth: {self.setpoint_depth}')
                rospy.loginfo(f'current  depth: {data.depth_mm}')
                rospy.loginfo(f'current output: {output}')
                self.config('FP' + str(output))
        else:
            pass


if __name__ == '__main__':

    try:
        DepthControl()
    except rospy.ROSInterruptException:
        pass

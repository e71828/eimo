#!/usr/bin/env python3

""""depth_control.py"""

import rospy
from eimo_msgs.srv import scl
from eimo_msgs.msg import control, depth
from time import  sleep


class DepthControl:
    def __init__(self):
        self.depth = []
        self.current_depth = None
        rospy.wait_for_service('scl_passthrough')
        rospy.init_node('depth_control', anonymous=True)
        rospy.Subscriber('control', control, self.diving, 1)
        rospy.Subscriber('depth', depth, self.diving, 2)

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
        sleep(2)
        config_QA = self.control('IP')
        config_echo = config_QA.Answer
        if config_echo == '':
            self.cw_limit = 40000  # TODO
            rospy.loginfo(f"Current position: {self.cw_limit}")

        self.config('DI-400000')
        self.config('FS2H')
        sleep(2)
        config_QA = self.control('IP')
        config_echo = config_QA.Answer
        if config_echo == '':
            self.ccw_limit = -360000  # TODO
            rospy.loginfo(f"Current position: {self.ccw_limit}")


        rospy.spin()

    def config(self, config_str):
        config_QA = self.control(config_str)
        config_echo = config_QA.Answer
        if config_echo == '%\r':
            rospy.loginfo(f'Config with string: {config_str}, result: OK')
        pass

    def diving(self, data, args):
        if args == 1:
            if data.up and not data.down:
                self.config('DI400000')
                self.config('FS1H')
            elif data.down and not data.up:
                self.config('DI-400000')
                self.config('FS2H')
            elif not data.down and not data.up:
                self.keep_depth()
            else:
                pass
        elif args == 2:
            self.depth.insert(0, self.current_depth)
            self.current_depth = data.depth_mm
            if len(self.depth) > 10:
                self.depth.pop()
        else:
            pass

    def keep_depth(self):
        pass  # TODO


if __name__ == '__main__':

    try:
        DepthControl()
    except rospy.ROSInterruptException:
        pass

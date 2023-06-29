#!/usr/bin/env python3

""""depth_control.py"""

import rospy
from eimo_msgs.srv import scl
from eimo_msgs.msg import control, depth
from time import sleep
from simple_pid import PID

"""
                                      1                                
                           <--------------------->                     
                         /-                       -\                   
               3      /--                           --\   3.1          
                   /--                                 --\             
                 /-                                       -\           
              /--                                           --\        
            --                                                 --      
            ^                                                   ^          
            |                                                   |      
            |                                                   |      
         1  |                                                   |  1   
            |                                                   |        
            |                                                   |      
            v                                                   v      
            --                                                 --      
              \-                                            --/        
                \--                                       -/           
                   \--                                 --/             
              5       \-                            --/    5           
                        \-                        -/                   
                           <--------------------->                     
                                     3.5                                

                              # robot: 5.47kg
                              # total: 6.45kg
                           # counterweight: 0.978kg
                     # details: 1, 3.1, 1, 5, 3.5, 5, 1, 3
                     # 22*40+6*5+17*0.7+2.8*6+4.0*10=978.7g
                     # 40g: 1+3+1+5+3+5+1+3=22; 22*40=880
                     # 5g: (0.1+0.5)*10=6; 6*5=30
                     # M4: 1.7g; 17*0.7=11.9
                     # M4x22 2.8; 2.8*6=16.8
                     # M4x40 4.0; 4.0*10=40
"""

class DepthControl:
    def __init__(self):
        rospy.init_node('depth_control', anonymous=True)
        self.current_depth = None
        self.controlling_flag = False
        self.controlling_flag_old = False
        self.weight_compensate = rospy.get_param('~weight_compensate', default=10000)  # total: 6.43kg
        self.base_output = 0

        sleep(3)  # wait here for 3 seconds
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

        self.setpoint_depth = rospy.get_param('init_depth') + 400
        # fetch a group (dictionary) of parameters
        self.gains = rospy.get_param('depth_gains', default={"p": 200, "i": 0, "d": 50})
        p, i, d = self.gains['p'], self.gains['i'], self.gains['d']
        self.p = p
        self.i = i
        self.d = d
        self.pid = PID(self.p, self.i, self.d, setpoint=self.setpoint_depth)
        self.pid.sample_time = 1 / rospy.get_param('depth_frequency', default=1)
        self.pid.output_limits = (-180000, 180000)

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
            self.gains = rospy.get_param('depth_gains')
            self.pid.tunings = (self.gains['p'], self.gains['i'], self.gains['d'])
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
            if self.setpoint_depth < 400:
                self.setpoint_depth = 400
        elif args == 2:
            self.current_depth = data.depth_mm
            if not self.controlling_flag and not self.controlling_flag_old:
                self.pid.setpoint = self.setpoint_depth
                output = int(self.pid(self.current_depth)) + self.weight_compensate
                rospy.loginfo(f'setpoint_depth: {self.setpoint_depth}')
                rospy.loginfo(f'current  depth: {data.depth_mm}')
                # rospy.loginfo(f'current output: {output}')
                self.config('FP' + str(output))
        else:
            pass


if __name__ == '__main__':

    try:
        DepthControl()
    except rospy.ROSInterruptException:
        pass

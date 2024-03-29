#!/usr/bin/env python3
from time import sleep

import rospy
from eimo_msgs.msg import control, depth , angle
from eimo_msgs.srv import scl
from rosgraph import is_master_online

from PySide6.QtCore import QObject, Signal, QThread


def check_ROS_master():
    if not is_master_online():
        return False
    return True

def pi_clip(angle):
    if angle > 180:
        return angle - 360
    elif angle < -180:
        return angle + 360
    else:
        return angle

def battery_level_map(voltage):
    if 25.2 >= voltage >= 21.2:
        return int((voltage - 21.2) / 4.0 * 100)
    else:
        return -1

# Signals must inherit QObject
class Communicate(QObject):
    signal_init_depth = Signal(int)
    signal_init_yaw = Signal(int)
    signal_yaw = Signal(int)
    signal_depth = Signal(int)
    signal_setpoint_depth = Signal(int)
    signal_setpoint_yaw = Signal(int)
    signal_battery_level = Signal(int)


class UpdateDataWorker(QThread):
    def __init__(self):
        super().__init__()
        self.signals = Communicate()
        self.setpoint_depth = None
        self.setpoint_yaw = None
        self.sub_angle = None
        self.sub_depth = None
        self.sub_control = None
        self.voltage_timer = None
        rospy.on_shutdown(self.release)

    def run(self):
        self.setpoint_depth = rospy.get_param('init_depth', default=0)
        self.setpoint_yaw = rospy.get_param('init_yaw', default=0)
        self.signals.signal_init_depth.emit(self.setpoint_depth)
        self.signals.signal_init_yaw.emit(self.setpoint_yaw)
        self.setpoint_depth += 400
        self.sub_control = rospy.Subscriber('control', control, self.gui_emit, 1)
        self.sub_depth = rospy.Subscriber('depth', depth, self.gui_emit, 2)
        self.sub_angle = rospy.Subscriber('angle', angle, self.gui_emit, 3)
        self.read_battery_level()
        self.voltage_timer = rospy.Timer(rospy.Duration(120), self.read_battery_level)

        rospy.spin()


    def gui_emit(self, msg, arg):
        if arg == 1:
            if msg.turn_left:
                self.setpoint_yaw -= 5
                self.setpoint_yaw = pi_clip(self.setpoint_yaw)
            elif msg.turn_right:
                self.setpoint_yaw += 5
                self.setpoint_yaw = pi_clip(self.setpoint_yaw)
            elif msg.up and not msg.down:
                self.setpoint_depth -= 10
            elif msg.down and not msg.up:
                self.setpoint_depth += 10
            elif msg.light1 and msg.light2 and msg.up:
                self.setpoint_depth -= 100
            elif msg.light1 and msg.light2 and msg.down:
                self.setpoint_depth += 100
            self.signals.signal_setpoint_yaw.emit(self.setpoint_yaw)
            self.signals.signal_setpoint_depth.emit(self.setpoint_depth)
        elif arg == 2:
            self.signals.signal_depth.emit(msg.depth_mm)
        elif arg == 3:
            self.signals.signal_yaw.emit(msg.yaw)

    def read_battery_level(self, timer=None):
        rospy.wait_for_service('scl_passthrough')
        try:
            request_voltage = rospy.ServiceProxy('scl_passthrough', scl)
            voltage_QA = request_voltage('IU')
            vol_str = voltage_QA.Answer
            if len(vol_str) > 2 and 'IU=' in vol_str:
                index = vol_str.index('IU=')
                voltage = int(vol_str[index + 3:index + 7], 16) / 10
                if voltage <= 22.2:
                    rospy.logwarn('voltage is low: %.1f V' % voltage)
                self.signals.signal_battery_level.emit(battery_level_map(voltage))
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def release(self):
        rospy.wait_for_service('scl_passthrough')
        if self.sub_control is not None:
            self.sub_control.unregister()
        if self.sub_depth is not None:
            self.sub_depth.unregister()
        if self.sub_angle is not None:
            self.sub_angle.unregister()

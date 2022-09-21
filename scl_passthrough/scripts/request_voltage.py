#!/usr/bin/env python3

""""request_voltage.py"""

import rospy
from eimo_msgs.srv import scl

if __name__ == '__main__':
    rospy.wait_for_service('serial_passthrough')
    try:
        request_voltage = rospy.ServiceProxy('serial_passthrough', scl)
        voltage_str = request_voltage('IT\n')
        rospy.loginfo(voltage_str.Answer)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

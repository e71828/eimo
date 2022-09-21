#!/usr/bin/env python3

""""request_voltage.py"""

import rospy
from eimo_msgs.srv import scl

if __name__ == '__main__':
    rospy.wait_for_service('scl_passthrough')
    try:
        request_voltage = rospy.ServiceProxy('scl_passthrough', scl)
        voltage_str = request_voltage('IU')
        rospy.loginfo(voltage_str.Answer)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

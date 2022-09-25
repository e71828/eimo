#!/usr/bin/env python3

""""request_voltage.py"""

import rospy
from eimo_msgs.srv import scl

if __name__ == '__main__':
    rospy.wait_for_service('scl_passthrough')
    rospy.init_node('request_voltage', anonymous=True)
    rospy.logdebug('Hello to debug')
    try:
        request_voltage = rospy.ServiceProxy('scl_passthrough', scl)
        voltage_QA = request_voltage('IU')
        vol_str = voltage_QA.Answer
        if len(vol_str) > 2 and 'IU=' in vol_str:
            index = vol_str.index('IU=')
            rospy.loginfo('voltage is %.1f V' % (int(vol_str[index+3:index+7], 16) / 10))

        # test MD
        voltage_QA = request_voltage('MD')
        vol_str = voltage_QA.Answer
        if vol_str == '%\r':
            rospy.loginfo("Motor disable OK with MD.")

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

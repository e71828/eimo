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
        rospy.loginfo('voltage is %.1f V' % (int(vol_str[3::], 16) / 10))
        voltage_QA = request_voltage('MD')
        vol_str = voltage_QA.Answer
        rospy.loginfo("Receiving {} for MD.".format(vol_str))

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

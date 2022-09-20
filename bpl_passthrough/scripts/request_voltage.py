#!/usr/bin/env python3

""""request_voltage.pn"""

import rospy


def receive_packet(packet):
    pass

if __name__ == '__main__':
    tx_publisher = rospy.Publisher("tx", String_request_voltage, queue_size=10)
    rospy.init_node("request_voltage_script")
    frequency = rospy.get_param('~frequency', default=1)
    rx_subscriber = rospy.Subscriber("rx", String, receive_packet)

    rate = rospy.Rate(frequency)
    while not rospy.is_shutdown():
    tx_publisher.publish(String_request_voltage)
    rate.sleep()
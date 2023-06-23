# !/bin/python3
from witprotocol import PacketReader, WITProtocol, PacketID
import serial
import rospy
from eimo_msgs.msg import angle

if __name__ == '__main__':
    with serial.Serial('/dev/ttyAMA1', 115200, timeout=1) as ser:

        pub = rospy.Publisher('angle', angle, queue_size=1)
        rospy.init_node('publish_angle', anonymous=True)
        frequency = rospy.get_param('~frequency', default=1)
        rate = rospy.Rate(frequency)  # 10hz
        while not rospy.is_shutdown():
            ser.reset_input_buffer()
            read_data = ser.read(44)
            packet_reader = PacketReader()
            if read_data != b'':
                packets = packet_reader.receive_bytes(read_data)
                if packets:
                    data_value = []
                    for packet in packets:
                        due_result = WITProtocol.get_gyro(*packet)
                        data_value.append(due_result)
                        if packet[0] == PacketID.POSITION:
                            roll, pitch, yaw = due_result
                            rospy.loginfo('Roll  : {:5d}'.format(int(roll)))
                            rospy.loginfo('Pitch : {:5d}'.format(int(pitch)))
                            rospy.loginfo('Yaw   : {:5d}'.format(int(yaw)))
                        elif packet[0] == PacketID.VELOCITY:
                            roll_v, pitch_v, yaw_v = due_result
                            # rospy.loginfo('angle velocity: {}'.format(due_result))
                        elif packet[0] == PacketID.ACCELERATION:
                            roll_a, pitch_a, yaw_a = due_result
                            # rospy.loginfo('angle acceleration: {}'.format(due_result))
                        else:
                            pass
                    pub.publish(yaw, roll, pitch, yaw_v, roll_v, pitch_v, yaw_a, roll_a, pitch_a)
            rate.sleep()

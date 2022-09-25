# !/bin/python3
from witprotocol import PacketReader, WITProtocol, PacketID
import serial
import rospy
from eimo_msgs.msg import angle


class WitSensor:
    def __init__(self):
        pass

    def pub_angle(self):
        pass


if __name__ == '__main__':
    with serial.Serial('/dev/ttyAMA1', 115200, timeout=1) as ser:

        pub = rospy.Publisher('angle', angle, queue_size=10)
        rospy.init_node('publish_angle', anonymous=True)
        frequency = rospy.get_param('~frequency', default=10)
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
                            roll, pitch, yaw = tuple(map(int, due_result))
                            rospy.loginfo('angle position: {}'.format(due_result))
                        elif packet[0] == PacketID.VELOCITY:
                            roll_v, pitch_v, yaw_v = due_result
                            rospy.loginfo('angle velocity: {}'.format(due_result))
                        elif packet[0] == PacketID.ACCELERATION:
                            roll_a, pitch_a, yaw_a = due_result
                            rospy.loginfo('angle acceleration: {}'.format(due_result))
                        else:
                            pass
                    pub.publish(yaw, roll, pitch, yaw_v, roll_v, pitch_v, yaw_a, roll_a, pitch_a)
            rate.sleep()

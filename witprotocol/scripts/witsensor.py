# !/bin/python3
from witprotocol import PacketReader, WITProtocol, PacketID
import serial
import rospy
from eimo_msgs.msg import angle


def read_angle(data_include):
    yaw_, roll_, pitch_, yaw_v, roll_v, pitch_v, yaw_a, roll_a, pitch_a = 0, 0, 0, 0, 0, 0, 0, 0, 0
    if data_include != b'':
        packet_reader = PacketReader()
        packets = packet_reader.receive_bytes(data_include)
        if packets:
            data_value = []
            angle_count, roll_sum, yaw_sum, pitch_sum = 0,0,0,0
            for packet in packets:
                due_result = WITProtocol.get_gyro(*packet)
                data_value.append(due_result)
                if packet[0] == PacketID.POSITION:
                    roll_, pitch_, yaw_ = due_result
                    roll_sum += roll_
                    pitch_sum += pitch_
                    yaw_sum += yaw_
                    angle_count += 1
                elif packet[0] == PacketID.VELOCITY:
                    roll_v, pitch_v, yaw_v = due_result
                elif packet[0] == PacketID.ACCELERATION:
                    roll_a, pitch_a, yaw_a = due_result
                else:
                    pass
            roll_, pitch_, yaw_ = roll_sum/angle_count, pitch_sum/angle_count, yaw_sum/angle_count
    return int(yaw_), int(roll_), int(pitch_)


if __name__ == '__main__':
    pub = rospy.Publisher('angle', angle, queue_size=1)
    rospy.init_node('publish_angle', anonymous=True)
    frequency = rospy.get_param('angle_frequency', default=10)
    rate = rospy.Rate(frequency)  # Hz

    serial_port = rospy.get_param('~serial_port', default='/dev/ttyAMA1')
    baudrate = rospy.get_param('~baudrate', default=115200)
    with serial.Serial(serial_port, baudrate, timeout=1) as ser:
        ser.reset_input_buffer()
        read_data = ser.read(233)
        yaw, roll, pitch = read_angle(read_data)
        rospy.set_param('init_yaw', yaw)
        while not rospy.is_shutdown():
            ser.reset_input_buffer()
            read_data = ser.read(233)
            yaw, roll, pitch = read_angle(read_data)
            rospy.loginfo('Roll  : {:5d}'.format(roll))
            rospy.loginfo('Pitch : {:5d}'.format(pitch))
            rospy.loginfo('Yaw   : {:5d}'.format(yaw))
            # rospy.loginfo('angle acceleration: {}'.format(due_result))
            # rospy.loginfo('angle velocity: {}'.format(due_result))

            pub.publish(yaw=yaw, roll=roll, pitch=pitch)
            rate.sleep()

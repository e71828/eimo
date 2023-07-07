# !/bin/python3
from .witprotocol import PacketReader, WITProtocol, PacketID
import serial
import rclpy
from rclpy.node import Node
from eimo_msgs.msg import Angle

class AngleWitSensor(Node):
    def __init__(self):
        super().__init__('publish_angle')
        self.angle_publisher = self.create_publisher(Angle, 'Angle', 10)
        self.declare_parameter('angle_frequency', 10)
        self.declare_parameter('~serial_port', '/dev/ttyAMA0')
        self.declare_parameter('~baudrate', 115200)
        self.declare_parameter('init_yaw', 0)

        self.frequency = self.get_parameter('angle_frequency').get_parameter_value().integer_value
        self.serial_port = self.get_parameter('~serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('~baudrate').get_parameter_value().integer_value

        self.ser = None
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error("Unable to open serial port {}".format(e))
            return

        yaw, _, _ = self.read_angle()
        self.init_yaw = rclpy.parameter.Parameter(
            'init_yaw',
            rclpy.Parameter.Type.INTEGER,
            yaw
        )
        self.set_parameters([self.init_yaw])

        self.timer = self.create_timer(1 / self.frequency, self.pub_angle)

    def read_angle(self):
        yaw_, roll_, pitch_, yaw_v, roll_v, pitch_v, yaw_a, roll_a, pitch_a = 0, 0, 0, 0, 0, 0, 0, 0, 0
        self.ser.reset_input_buffer()
        read_data = self.ser.read(44)
        packet_reader = PacketReader()
        if read_data != b'':
            packets = packet_reader.receive_bytes(read_data)
            if packets:
                data_value = []
                for packet in packets:
                    due_result = WITProtocol.get_gyro(*packet)
                    data_value.append(due_result)
                    if packet[0] == PacketID.POSITION:
                        roll_, pitch_, yaw_ = due_result
                    elif packet[0] == PacketID.VELOCITY:
                        roll_v, pitch_v, yaw_v = due_result
                    elif packet[0] == PacketID.ACCELERATION:
                        roll_a, pitch_a, yaw_a = due_result
                    else:
                        pass
        return int(yaw_), int(roll_), int(pitch_)

    def pub_angle(self):
        yaw, roll, pitch = self.read_angle()
        self.get_logger().info('Roll  : {:5d}'.format(roll))
        self.get_logger().info('Pitch : {:5d}'.format(pitch))
        self.get_logger().info('Yaw   : {:5d}'.format(yaw))

        msg = Angle()
        msg.yaw, msg.roll, msg.pitch = yaw, roll, pitch
        self.angle_publisher.publish(msg)
    def __del__(self):
        if self.ser is not None:
            self.ser.close()

def main(args=None):
    rclpy.init(args=args)
    angle_node = AngleWitSensor()
    rclpy.spin(angle_node)
    angle_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
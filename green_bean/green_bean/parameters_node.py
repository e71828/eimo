import rclpy
import rclpy.node

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')

        self.declare_parameter('my_string', 'world')
        self.declare_parameter('my_num', 2)
        self.declare_parameter('my_pid', [1.0, 2.0, 3.0])
        my_new_string = rclpy.parameter.Parameter(
            'my_string',
            rclpy.Parameter.Type.STRING,
            'world'
        )

        all_new_parameters = [my_new_string]
        self.set_parameters(all_new_parameters)

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        my_param = self.get_parameter('my_string').get_parameter_value().string_value

        self.get_logger().info('Hello %s!' % my_param)

        my_param = self.get_parameter('my_num').value
        self.get_logger().info('GOOD %d!' % my_param)

def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
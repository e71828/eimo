import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue


class SetExternalParam(Node):
    def __init__(self):
        super().__init__('change_parameter_node')

        self.cli = self.create_client(SetParameters, '/' + 'minimal_param_node' + '/set_parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetParameters.Request()

    def send_request(self):
        new_param_value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value='July!')
        self.req.parameters = [Parameter(name='my_string', value=new_param_value)]
        self.future = self.cli.call_async(self.req)


class GetExternalParam(Node):
    def __init__(self):
        super().__init__('fetch_parameter_node')

        self.cli = self.create_client(GetParameters, '/' + 'minimal_param_node' + '/get_parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetParameters.Request()

    def get_param(self):
        self.req.names = ['my_num', 'my_string', 'my_pid']
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    set_param = SetExternalParam()
    set_param.send_request()
    while rclpy.ok():
        rclpy.spin_once(set_param)
        if set_param.future.done():
            try:
                response = set_param.future.result()
                if response is not None:
                    assert len(response.results) == 1
                    res = [i.successful for i in response.results]
                    assert all(res)
                    set_param.get_logger().info(
                        'Parameter changed successfully')
            except Exception as e:
                set_param.get_logger().info(
                    'Service call failed %r' % (e,))
            break



    get_param = GetExternalParam()
    get_param.get_param()
    # use a blocking call to get the latest state of the parameters
    rclpy.spin_until_future_complete(get_param, get_param.future)
    try:
        response = get_param.future.result()
        if response is not None:
            assert response.values[0].integer_value == 2
            assert response.values[1].string_value == 'July!'
            assert list(response.values[2].double_array_value) == [1.0, 2.0, 3.0]
            get_param.get_logger().info(
                'Parameter read successfully')
    except Exception as e:
        get_param.get_logger().info(
            'Service call failed %r' % (e,))



    set_param.destroy_node()
    get_param.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
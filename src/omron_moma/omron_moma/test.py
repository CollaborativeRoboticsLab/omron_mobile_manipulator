import sys

from pickplace_msgs.srv import AskModbus
import rclpy
from rclpy.node import Node


class ModbusClientAsync(Node):

    def __init__(self):
        super().__init__('modbus_client_async')
        self.cli = self.create_client(AskModbus, 'ask_modbus')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AskModbus.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    modbus_client = ModbusClientAsync()
    modbus_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(modbus_client)
        if modbus_client.future.done():
            try:
                response = modbus_client.future.result()
            except Exception as e:
                modbus_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                modbus_client.get_logger().info(str(response))
            break

    modbus_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Suscriptor(Node):

    def __init__(self):
        super().__init__('suscriptor')

        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.callback,
            10
        )

    def callback(self, msg):
        self.get_logger().info(
            f'Recibido: {msg.data}'
        )


def main(args=None):

    rclpy.init(args=args)

    nodo = Suscriptor()

    rclpy.spin(nodo)

    nodo.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

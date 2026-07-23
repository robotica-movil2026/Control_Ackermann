import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Publicador(Node):

    def __init__(self):
        super().__init__('publicador')

        self.publisher_ = self.create_publisher(
            String,
            'chatter',
            10
        )

        self.timer = self.create_timer(
            1.0,
            self.publicar
        )

    def publicar(self):
        msg = String()
        msg.data = "Hola desde ROS2 Jazzy"

        self.publisher_.publish(msg)

        self.get_logger().info(f'Publicado: {msg.data}')


def main(args=None):

    rclpy.init(args=args)

    nodo = Publicador()

    rclpy.spin(nodo)

    nodo.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

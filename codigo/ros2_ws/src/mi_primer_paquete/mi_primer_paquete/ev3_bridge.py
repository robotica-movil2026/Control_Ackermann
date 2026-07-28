import socket
import math
import time
import traceback

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class EV3Bridge(Node):
    def __init__(self):
        super().__init__('ev3_bridge')

        self.declare_parameter('ev3_host', '10.51.221.1')
        self.declare_parameter('ev3_port', 12348)
        self.declare_parameter('wheel_base', 0.25)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ticks_per_meter', 100.0)

        ev3_host = self.get_parameter('ev3_host').value
        ev3_port = self.get_parameter('ev3_port').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.ticks_per_meter = self.get_parameter('ticks_per_meter').value

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_encoder = 0
        self.last_time = self.get_clock().now()
        self.connected = False

        self.sub = self.create_subscription(
            Twist, self.get_parameter('cmd_vel_topic').value,
            self.cmd_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.connect_to_ev3(ev3_host, ev3_port)

        self.timer = self.create_timer(0.05, self.timer_callback)

    def connect_to_ev3(self, host, port):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((host, port))
            self.sock.setblocking(False)
            self.buffer = ''
            self.connected = True
            self.get_logger().info(f'Conectado a EV3 en {host}:{port}')
        except Exception as e:
            self.get_logger().error(f'Error conectando a EV3: {e}')
            self.connected = False

    def cmd_callback(self, msg):
        if not self.connected:
            return

        linear = msg.linear.x
        angular = msg.angular.z

        vel_pct = max(-100.0, min(100.0, (linear / self.max_speed) * 100.0))
        dir_deg = math.degrees(angular)
        dir_deg = max(-90.0, min(90.0, dir_deg))

        try:
            cmd = f'{vel_pct:.1f},{dir_deg:.1f}\n'
            self.sock.sendall(cmd.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Error enviando a EV3: {e}')
            self.connected = False

    def timer_callback(self):
        if not self.connected:
            return

        try:
            data = self.sock.recv(1024).decode('utf-8')
            if data:
                self.buffer += data
                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        self.process_feedback(line)
        except socket.errors.WouldBlock:
            pass
        except (ConnectionResetError, BrokenPipeError, OSError):
            self.get_logger().error('Conexion con EV3 perdida')
            self.connected = False
        except Exception:
            pass

    def process_feedback(self, line):
        try:
            partes = line.split(',')
            if len(partes) != 3:
                return

            enc = float(partes[0])
            dir_actual = float(partes[2])

            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds / 1e9
            if dt <= 0 or dt > 0.5:
                self.last_time = now
                self.last_encoder = enc
                return

            delta_enc = enc - self.last_encoder
            dist = delta_enc / self.ticks_per_meter

            if dist != 0:
                steer_rad = math.radians(dir_actual)
                radius = self.wheel_base / math.tan(steer_rad) if abs(steer_rad) > 0.01 else float('inf')

                if abs(steer_rad) > 0.01:
                    delta_theta = dist / radius
                else:
                    delta_theta = 0.0

                dx = dist * math.cos(self.theta + delta_theta / 2.0)
                dy = dist * math.sin(self.theta + delta_theta / 2.0)

                self.x += dx
                self.y += dy
                self.theta += delta_theta

            v = dist / dt if dt > 0 else 0.0
            w = delta_theta / dt if dt > 0 else 0.0

            self.publish_odometry(now, v, w)

            self.last_time = now
            self.last_encoder = enc

        except Exception as e:
            self.get_logger().error(f'Error procesando feedback: {e}')

    def publish_odometry(self, stamp, v, w):
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = EV3Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.connected:
            try:
                node.sock.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
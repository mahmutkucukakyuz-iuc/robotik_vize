#!/usr/bin/env python3
"""ee_pose_publisher: TF base_link -> tool0 donusumunu /ee_pose olarak yayinlar.

Parametreler:
  base_frame (str, 'base_link'), ee_frame (str, 'tool0'),
  rate_hz (float, 20.0), topic (str, '/ee_pose')
Yayin: /ee_pose  geometry_msgs/PoseStamped  (frame_id = base_frame)
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException


class EePosePublisher(Node):
    def __init__(self):
        super().__init__('ee_pose_publisher')
        self.base = self.declare_parameter('base_frame', 'base_link').value
        self.ee = self.declare_parameter('ee_frame', 'tool0').value
        rate = float(self.declare_parameter('rate_hz', 20.0).value)
        topic = self.declare_parameter('topic', '/ee_pose').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self._warned = False
        self.get_logger().info(
            f'{self.base} -> {self.ee} donusumu {topic} uzerinden {rate:.0f} Hz yayinlanacak')

    def on_timer(self):
        try:
            # Time() = en son mevcut donusum
            tf = self.tf_buffer.lookup_transform(self.base, self.ee, Time())
        except TransformException as ex:
            if not self._warned:
                self.get_logger().warning(f'TF henuz yok: {ex}')
                self._warned = True
            return
        msg = PoseStamped()
        msg.header.stamp = tf.header.stamp
        msg.header.frame_id = self.base
        t = tf.transform.translation
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = t.x, t.y, t.z
        msg.pose.orientation = tf.transform.rotation
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EePosePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: BLE001
        # Ctrl+C sonrasi context kapaninca rclpy RCLError atar; bu normal kapanistir
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

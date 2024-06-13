import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class StaticFramePublisher(Node):
    def __init__(self):
        super().__init__('static_tf2_broadcaster')
        self._broadcaster = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(2, self.broadcast_static_transform)

    def broadcast_static_transform(self):
        static_transformStamped = TransformStamped()

        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = 'world'
        static_transformStamped.child_frame_id = 'robot_frame'

        static_transformStamped.transform.translation.x = 1.0
        static_transformStamped.transform.translation.y = 2.0
        static_transformStamped.transform.translation.z = 0.0

        quat = tf_transformations.quaternion_from_euler(0, 0, 45)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self._broadcaster.sendTransform(static_transformStamped)

def main():
    rclpy.init()
    node = StaticFramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


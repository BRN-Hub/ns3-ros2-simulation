import rclpy
from geometry_msgs.msg import TransformStamped, Pose
import numpy as np

def publish_to_ns3(node_handle, publisher, parent_node_id, child_node_id):
    msg = TransformStamped()
    msg.header.stamp = node_handle.get_clock().now().to_msg()
    msg.header.frame_id = parent_node_id
    msg.child_frame_id = child_node_id
    msg.transform.translation.x += np.random.uniform(-1, 1)
    msg.transform.translation.y += np.random.uniform(-1, 1)
    msg.transform.translation.z = np.random.uniform(0, 1)
    msg.transform.rotation.x = 0.0
    msg.transform.rotation.y = 0.0
    msg.transform.rotation.z = 0.0
    msg.transform.rotation.w = 1.0
    publisher.publish(msg)

def main():
    rclpy.init()
    node_handle = rclpy.create_node('publisher_ros_ns3')
    publisher = node_handle.create_publisher(TransformStamped, 'position_ros_ns3', 10)
    node_handle.create_timer(1.0, lambda: publish_to_ns3(node_handle, publisher, "map", "node-mobile-0"))
    rclpy.spin(node_handle)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
import rclpy
from geometry_msgs.msg import Vector3
from  geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

def publish_tf(node_handle, transform_broadcaster, position, parent_frame_id, child_frame_id):
    transform = TransformStamped()
    transform.header.stamp = node_handle.get_clock().now().to_msg()
    transform.header.frame_id = parent_frame_id
    transform.child_frame_id = child_frame_id
    transform.transform.translation.x = position.transform.translation.x
    transform.transform.translation.y = position.transform.translation.y
    transform.transform.translation.z = position.transform.translation.z
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    transform.transform.rotation.w = 1.0
    transform_broadcaster.sendTransform(transform)

def main():
    rclpy.init()
    node = rclpy.create_node("position_subscriber")

    def position_callback(msg):
        node_handle = rclpy.create_node('ns3_tf_publisher')
        transform_broadcaster = TransformBroadcaster(node_handle)

        publish_tf(node_handle, transform_broadcaster, msg, "map", msg.child_frame_id)

    subscription = node.create_subscription(TransformStamped, "position", position_callback, 10)

    rclpy.spin(node)

    # Encerre o nó ROS 2
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
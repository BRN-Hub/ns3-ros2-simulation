# Import the necessary libraries
import rclpy
from geometry_msgs.msg import TransformStamped, Pose

def publish_to_ns3(node_handle, publisher, parent_node_id, child_node_id, position):
    if position.position.x < 1.0 and position.position.y == 1:
            position.position.x += 0.2
            position.position.x = round(position.position.x, 2)
    elif position.position.x == 1.0 and position.position.y > -1.0:
        position.position.y -= 0.2
        position.position.y = round(position.position.y, 2)
    elif position.position.x > -1.0 and position.position.y == -1.0:
        position.position.x -= 0.2
        position.position.x = round(position.position.x, 2)
    elif position.position.x == -1.0 and position.position.y < 1.0:
        position.position.y += 0.2
        position.position.y = round(position.position.y, 2)
    msg = TransformStamped()
    msg.header.stamp = node_handle.get_clock().now().to_msg()
    msg.header.frame_id = parent_node_id
    msg.child_frame_id = child_node_id
    msg.transform.translation.x = position.position.x
    msg.transform.translation.y = position.position.y
    msg.transform.translation.z = position.position.z
    msg.transform.rotation.x = 0.0
    msg.transform.rotation.y = 0.0
    msg.transform.rotation.z = 0.0
    msg.transform.rotation.w = 1.0
    publisher.publish(msg)

def main():
    # Inicialização do ROS 2
    rclpy.init()
    
    # Inicialização do nó controlador
    node_handle = rclpy.create_node('publisher_ros_ns3')
    publisher = node_handle.create_publisher(TransformStamped, 'position_ros_ns3', 10)
    position = Pose()
    position.position.x = -1.0
    position.position.y = 1.0
    position.position.z = 2.0
    timer = node_handle.create_timer(1.0, lambda: publish_to_ns3(node_handle, publisher, "map", "node-mobile-0", position))
    rclpy.spin(node_handle)

if __name__ == "__main__":
    main()
# Importação das bibliotecas do ROS 2
import rclpy
from  geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# Função para publicar a transformação para o Rviz 2
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
    # Inicialização do ROS 2
    rclpy.init()
    node = rclpy.create_node("position_subscriber")

    # Função callback para publicação da transformação para o Rviz 2
    def position_callback(msg):
        node_handle = rclpy.create_node('ns3_tf_publisher')
        transform_broadcaster = TransformBroadcaster(node_handle)

        publish_tf(node_handle, transform_broadcaster, msg, "map", msg.child_frame_id)

    # Criação da subscrição do posicionamento dos nós móveis
    subscription = node.create_subscription(TransformStamped, "position_ns3_ros", position_callback, 10)

    # Mantém o nó ROS 2 ativo
    rclpy.spin(node)

    # Encerre o nó ROS 2
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
import rclpy
from geometry_msgs.msg import TransformStamped
import ns.core
import ns.network
import ns.mobility

def publish_to_ros2(node_handle, position, publisher, parent_node_id, child_node_id):
    msg = TransformStamped()
    msg.header.stamp = node_handle.get_clock().now().to_msg()
    msg.header.frame_id = parent_node_id
    msg.child_frame_id = child_node_id
    msg.transform.translation.x = position.x
    msg.transform.translation.y = position.y
    msg.transform.translation.z = position.z
    msg.transform.rotation.x = 0.0
    msg.transform.rotation.y = 0.0
    msg.transform.rotation.z = 0.0
    msg.transform.rotation.w = 1.0
    publisher.publish(msg)


def main(args=None):
    qtd_nodes_mobiles = 3
    qtd_nodes_static = 4
    ns.core.GlobalValue.Bind("SimulatorImplementationType", ns.core.StringValue("ns3::RealtimeSimulatorImpl"))
    rclpy.init()
    node_handle = rclpy.create_node('position_publisher')

    publisher = node_handle.create_publisher(TransformStamped, 'position', 10)
    nodesStatic = ns.network.NodeContainer()
    nodesStatic.Create(qtd_nodes_static)
    mobility = ns.mobility.MobilityHelper()
    position_allocator = ns.mobility.ListPositionAllocator()
    position_allocator.Add(ns.mobility.Vector(-1.0, -1.0, 1.0))
    position_allocator.Add(ns.mobility.Vector(1.0, -1.0, 1.0))
    position_allocator.Add(ns.mobility.Vector(-1.0, 1.0, 1.0))
    position_allocator.Add(ns.mobility.Vector(1.0, 1.0, 1.0))
    mobility.SetPositionAllocator(position_allocator)
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel")
    mobility.Install(nodesStatic)

    nodeMobile = ns.network.NodeContainer()
    nodeMobile.Create(qtd_nodes_mobiles)
    mobility_mobile = ns.mobility.MobilityHelper()
    mobility_mobile.SetPositionAllocator("ns3::RandomWalk2dPositionAllocator",
                                        "Bounds", ns.core.RectangleValue(ns.core.Rectangle(-10, 10, -10, 10)))
    nodeMobile.install(mobility_mobile)

    def publish_position_to_ros2(publisher):
        for i in range(qtd_nodes_static):
            node = nodesStatic.Get(i)
            position = node.GetObject(ns.mobility.MobilityModel.GetTypeId()).GetPosition()

            publish_to_ros2(node_handle, position, publisher, "map", f"node-static-{i}")
        for i in range(qtd_nodes_mobiles):
            node = nodeMobile.Get(i)
            position = node.GetObject(ns.mobility.MobilityModel.GetTypeId()).GetPosition()

            publish_to_ros2(node_handle, position, publisher, "map", f"node-mobile-{i}")

    for i in range(1000):
        ns.core.Simulator.Schedule(ns.core.Seconds(i), publish_position_to_ros2, publisher)
    
    ns.core.Simulator.Run()
    node_handle.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
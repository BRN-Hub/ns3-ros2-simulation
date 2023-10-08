import rclpy
from geometry_msgs.msg import TransformStamped
import ns.core
import ns.network
import ns.mobility

# Função para publicação do posicionamento dos nós para o ROS 2
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
    # Configuração do simulador
    ns.core.GlobalValue.Bind("SimulatorImplementationType", ns.core.StringValue("ns3::RealtimeSimulatorImpl"))

    # Constantes
    qtd_nodes_mobiles = 1
    qtd_nodes_static = 4
    
    # Inicialização do ROS 2
    rclpy.init()
    node_handle = rclpy.create_node('position_publisher')
    publisher = node_handle.create_publisher(TransformStamped, 'position_ns3_ros', 10)

    # Configuração dos nós estáticos
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

    # Configuração dos nós móveis
    nodeMobile = ns.network.NodeContainer()
    nodeMobile.Create(qtd_nodes_mobiles)
    mobility_mobile = ns.mobility.MobilityHelper()
    mobility_mobile.SetMobilityModel("ns3::ConstantVelocityMobilityModel")

    mobility_mobile.Install(nodeMobile)

    def update_position(msg):
        nodeMobile.Get(0).GetObject(ns.mobility.MobilityModel.GetTypeId()).SetPosition(ns.mobility.Vector(msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z))

    subscribe = node_handle.create_subscription(TransformStamped, "position_ros_ns3", update_position, 10)    

    '''
    ============= Moviment using WaypointMobilityModel =============

    mobility1 = ns.mobility.WaypointMobilityModel()
    nodeMobile.Get(0).AggregateObject(mobility1)

    # Adicione waypoints para o nó 1
    waypoint1 = ns.mobility.Waypoint(ns.core.Seconds(0), ns.mobility.Vector(-1, 1, 2))
    waypoint2 = ns.mobility.Waypoint(ns.core.Seconds(5), ns.mobility.Vector(1, 1, 2))
    waypoint3 = ns.mobility.Waypoint(ns.core.Seconds(10), ns.mobility.Vector(1, -1, 2))
    waypoint4 = ns.mobility.Waypoint(ns.core.Seconds(15), ns.mobility.Vector(-1, -1, 2))
    waypoint5 = ns.mobility.Waypoint(ns.core.Seconds(20), ns.mobility.Vector(-1, 1, 2))

    # Adicione os waypoints ao modelo de mobilidade
    mobility1.AddWaypoint(waypoint1)
    mobility1.AddWaypoint(waypoint2)
    mobility1.AddWaypoint(waypoint3)
    mobility1.AddWaypoint(waypoint4)
    mobility1.AddWaypoint(waypoint5)
    '''

    # Função callback para atualização do posicionamento dos nós
    def publish_position_to_ros2(publisher):

        # Atualização do posicionamento dos nós estáticos
        for i in range(qtd_nodes_static):
            node = nodesStatic.Get(i)
            position = node.GetObject(ns.mobility.MobilityModel.GetTypeId()).GetPosition()

            publish_to_ros2(node_handle, position, publisher, "map", f"node-static-{i}")

        # Atualização do posicionamento dos nós móveis
        for i in range(qtd_nodes_mobiles):
            node = nodeMobile.Get(i)
            position = node.GetObject(ns.mobility.MobilityModel.GetTypeId()).GetPosition()

            publish_to_ros2(node_handle, position, publisher, "map", f"node-mobile-{i}")

    # Configuração do timer para atualização do posicionamento dos nós
    for i in range(1000):
        ns.core.Simulator.Schedule(ns.core.Seconds(i), publish_position_to_ros2, publisher)
        ns.core.Simulator.Schedule(ns.core.Seconds(i), rclpy.spin_once, node_handle)
    
    # Início da simulação
    ns.core.Simulator.Run()
    node_handle.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
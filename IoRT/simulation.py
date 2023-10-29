# Importação das bibliotecas necessárias
import rclpy
from geometry_msgs.msg import TransformStamped
import ns.core
import ns.network
import ns.mobility
import ns.point_to_point
import ns.wifi

# // Default Network Topology
# //
# //                  10.1.1.0
# //            n0 -------------- n1
# //      n2  n3  n4  n5
# //
# //

temp = 1

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

    # Configuração dos nós point to point
    nodesP2P = ns.network.NodeContainer()
    nodesP2P.Create(2)

    # Configuração dos nós estáticos
    nodesStatic = ns.network.NodeContainer()
    nodesStatic.Add(nodesP2P.Get(0))
    nodesStatic.Create(qtd_nodes_static)

    # Configuração do canal de comunicação
    pointToPoint = ns.point_to_point.PointToPointHelper()
    pointToPoint.SetDeviceAttribute("DataRate", ns.core.StringValue("5Mbps"))
    pointToPoint.SetChannelAttribute("Delay", ns.core.StringValue("2ms"))

    # Instalação dos dispositivos de comunicação
    devices = pointToPoint.Install(nodesP2P)

    # Configuração da pilha de protocolos
    wifiApNode = nodesP2P.Get(0)

    channel = ns.wifi.YansWifiChannelHelper.Default()
    phy = ns.wifi.YansWifiPhyHelper.Default()
    phy.SetChannel(channel.Create())

    wifi = ns.wifi.WifiHelper()
    wifi.SetRemoteStationManager("ns3::AarfWifiManager")

    mac = ns.wifi.WifiMacHelper()
    ssid = ns.wifi.SsidValue("ns-3-ssid")

    mac.SetType("ns3::StaWifiMac", "Ssid", ssid, "ActiveProbing", ns.core.BooleanValue(False))
    staDevices = wifi.Install(phy, mac, nodesStatic)

    mac.SetType("ns3::ApWifiMac", "Ssid", ssid)
    apDevices = wifi.Install(phy, mac, wifiApNode)


    '''================= Configuração dos nós estáticos =================='''
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
    mobility.Install(nodesStatic),

    # Configuração dos nós móveis
    nodeMobile = ns.network.NodeContainer()
    nodeMobile.Create(qtd_nodes_mobiles)

    mobility1 = ns.mobility.WaypointMobilityModel()
    nodeMobile.Get(0).AggregateObject(mobility1)

    # Configuração do modelo de mobilidade usando contante position dos nós móveis
    #mobility_mobile = ns.mobility.MobilityHelper()
    #mobility_mobile.SetMobilityModel("ns3::ConstantVelocityMobilityModel")
    #mobility_mobile.Install(nodeMobile)

    # Função callback para atualização do posicionamento dos nós móveis na simulação ns-3
    def update_position(msg):
        global temp

        '''==================== Moviment using SetPosition ====================
        nodeMobile.Get(0).GetObject(ns.mobility.MobilityModel.GetTypeId()).SetPosition(ns.mobility.Vector(msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z))
        '''

        # Movimentação usando WaypointMobilityModel
        waypoint = ns.mobility.Waypoint(ns.core.Seconds(temp * 2), ns.mobility.Vector(msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z))
        mobility1.AddWaypoint(waypoint)
        temp += 1

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
        ns.core.Simulator.Schedule(ns.core.Seconds(i * 0.2), publish_position_to_ros2, publisher)
        ns.core.Simulator.Schedule(ns.core.Seconds(i), rclpy.spin_once, node_handle)
    
    # Início da simulação
    ns.core.Simulator.Run()
    node_handle.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
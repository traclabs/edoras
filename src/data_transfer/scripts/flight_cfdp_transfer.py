#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import cfdp
from cfdp.transport.udp import UdpTransport
from cfdp.filestore import NativeFileStore

#import logging
#logging.basicConfig(level=logging.DEBUG)


class FlightEdorasCfdpTransfer(Node):
    """
    This class receives the Odometry information from topic /groundsystem/rover_app_hk_tlm
    (cfe_msgs) and publishes them to /odom
    This class also subscribes to "/cmd_vel", and publishes the said command
    to /groundsystem/rover_app_cmd (cfe_msgs)
    """
    def __init__(self):
        super().__init__('flight_edoras_cfdp_transfer')

        self.declare_parameter('cfdp.ip',  "127.0.0.1")
        self.declare_parameter('cfdp.port',  5222)
        self.declare_parameter('cfdp.entity_id',  2)

        self.declare_parameter('cfdp.ip_peer',  "127.0.0.1")
        self.declare_parameter('cfdp.port_peer',  5111)
        self.declare_parameter('cfdp.entity_id_peer',  1)
        
        self.declare_parameter('cfdp.store_package',  "data_transfer")
        self.declare_parameter("cfdp.store_folder", "data/flight")        

        self.ip = self.get_parameter('cfdp.ip').value
        self.port = self.get_parameter('cfdp.port').value
        self.entity_id = self.get_parameter('cfdp.entity_id').value
        
        self.ip_peer = self.get_parameter('cfdp.ip_peer').value
        self.port_peer = self.get_parameter('cfdp.port_peer').value
        self.entity_id_peer = self.get_parameter('cfdp.entity_id_peer').value
        
        self.store_pkg = self.get_parameter('cfdp.store_package').value
        self.store_folder = self.get_parameter('cfdp.store_folder').value         

        # may raise PackageNotFoundError
        pkg_share_dir = get_package_share_directory(self.store_pkg)        
        self.filestore = pkg_share_dir + "/" + self.store_folder
        self.get_logger().info(" Filestore: %s" % self.filestore)
        
        # Udp transport       
        self.udp_transport = UdpTransport(routing={"*": [(self.ip_peer, self.port_peer)]})
        self.get_logger().info("Binding to: %s : %d " % (self.ip, self.port))
        self.udp_transport.bind(self.ip, self.port)
         
        self.cfdp_entity = cfdp.CfdpEntity(entity_id=self.entity_id, 
                      filestore = NativeFileStore(self.filestore), 
                      transport = self.udp_transport)


#################################
def main(args=None):

    rclpy.init(args=args)

    fect = FlightEdorasCfdpTransfer()
    rclpy.spin(fect)

    fect.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from gmserver.srv import LoadMap
from rcl_interfaces.srv import GetParameters

class GPSFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('gps_tf_broadcaster')

        self.map_utm_easting = None
        self.map_utm_northing = None
        
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.map_origin_set = False
        
        self.declare_parameter('map_file_path')
        self.map_file_path = self.get_parameter('map_file_path').get_parameter_value().string_value
        
        # Localization node name
        self.localization_node_name = 'tiny_localization_node'
        
        self.get_logger().info('GPS frame broadcaster waiting for map origin parameters...')

        self.gmserver_client = self.create_client(LoadMap, '/load_map')
        
        # Timer to periodically check for parameters
        self.param_check_timer = self.create_timer(1.0, self.check_map_origin_params)
        
        # Try to get parameters immediately on startup
        self.check_map_origin_params()

        # Timer to periodically check for gmserver availability
        self.service_check_timer = self.create_timer(1.0, self.check_gmserver_and_load_map)
        
        # Try to call gmserver immediately
        self.check_gmserver_and_load_map()
    
    def check_gmserver_and_load_map(self):
        """Check if gmserver is available and load map to get first node coordinates"""
        if not self.map_origin_set:
            if self.gmserver_client.service_is_ready():
                self.get_logger().info('gmserver is ready, loading map...')
                self.call_gmserver_load_map()
            else:
                self.get_logger().debug('Waiting for gmserver to be available...')
    
    def call_gmserver_load_map(self):
        """Call gmserver LoadMap service to get map data"""
        request = LoadMap.Request()
        request.map_file_path = self.map_file_path
        
        future = self.gmserver_client.call_async(request)
        future.add_done_callback(self.gmserver_response_callback)
    
    def gmserver_response_callback(self, future):
        """Handle gmserver response and extract first node UTM coordinates"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Map loaded successfully: {response.message}')
                
                # Get first node from the map
                if len(response.graph_map.map_data.nodes) > 0:
                    first_node = response.graph_map.map_data.nodes[0]
                    
                    # Extract UTM coordinates from first node
                    self.map_utm_easting = first_node.utm_info.easting
                    self.map_utm_northing = first_node.utm_info.northing
                    utm_zone = first_node.utm_info.zone
                    
                    self.get_logger().info(f'Using first node {first_node.id} as map origin: '
                                         f'UTM({self.map_utm_easting:.2f}, {self.map_utm_northing:.2f}) Zone {utm_zone}')
                else:
                    self.get_logger().error('Map contains no nodes!')
            else:
                self.get_logger().error(f'Failed to load map: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Error processing gmserver response: {str(e)}')
    
    def check_map_origin_params(self):
        """Check if map origin parameters are available from localization node"""
        if not self.map_origin_set:
            try:
                import subprocess
                
                # Use ros2 param get command directly - more reliable
                cmd = ['ros2', 'param', 'get', '/localization/tiny_localization_node', 'map_origin.utm_easting']
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=2.0)
                
                if result.returncode == 0:
                    # Extract the value from output like "Double value is: 482253.6011915017"
                    easting_line = result.stdout.strip()
                    if "Double value is:" in easting_line:
                        easting = float(easting_line.split(":")[-1].strip())
                        
                        # Get northing
                        cmd = ['ros2', 'param', 'get', '/localization/tiny_localization_node', 'map_origin.utm_northing']
                        result = subprocess.run(cmd, capture_output=True, text=True, timeout=2.0)
                        if result.returncode == 0 and "Double value is:" in result.stdout:
                            northing = float(result.stdout.split(":")[-1].strip())
                            
                            # Get zone
                            cmd = ['ros2', 'param', 'get', '/localization/tiny_localization_node', 'map_origin.utm_zone']
                            result = subprocess.run(cmd, capture_output=True, text=True, timeout=2.0)
                            if result.returncode == 0 and "Integer value is:" in result.stdout:
                                zone = int(result.stdout.split(":")[-1].strip())
                                
                                # Successfully got all parameters
                                self.set_map_origin(easting, northing, zone)
                                return
                
                self.get_logger().debug('Map origin parameters not ready yet')
                        
            except Exception as e:
                self.get_logger().debug(f'Map origin parameters not ready: {e}')
    
    def set_map_origin(self, utm_easting, utm_northing, utm_zone):
        """Set map->odom transform when map origin parameters are received"""
        # Create static transform: map frame is at the GPS origin
        # odom frame starts at (0,0) relative to map
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = "map"
        static_transform.child_frame_id = "odom"
        
        # Map origin is at the first GPS position
        # Odom frame starts at the same location, so no translation needed
        # 아래 UTM 좌표 연산에서 (Map의 원점 좌표 - GPS 초기 좌표) 연산을 수행해야 함
        # utm_easting은 정확히는 GPS의 초기 좌표에 해당
        static_transform.transform.translation.x = self.map_utm_easting - utm_easting
        static_transform.transform.translation.y = self.map_utm_northing - utm_northing
        static_transform.transform.translation.z = 0.0
        
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0
        
        self.broadcaster.sendTransform(static_transform)
        self.map_origin_set = True
        
        # Cancel the timer
        self.param_check_timer.cancel()
        
        self.get_logger().info(f'Map->odom transform set with map origin at UTM ({utm_easting - self.map_utm_easting:.2f}, {utm_northing - self.map_utm_northing:.2f}) Zone {utm_zone}')
        self.get_logger().info('GPS frame broadcaster initialized')

def main(args=None):
    rclpy.init(args=args)
    
    gps_broadcaster = GPSFrameBroadcaster()
    
    try:
        rclpy.spin(gps_broadcaster)
    except KeyboardInterrupt:
        pass
    finally:
        gps_broadcaster.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.srv import GetParameters

class GPSFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('gps_tf_broadcaster')
        
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.map_origin_set = False
        
        # Localization node name
        self.localization_node_name = 'tiny_localization_node'
        
        self.get_logger().info('GPS frame broadcaster waiting for map origin parameters...')
        
        # Timer to periodically check for parameters
        self.param_check_timer = self.create_timer(1.0, self.check_map_origin_params)
        
        # Try to get parameters immediately on startup
        self.check_map_origin_params()
    
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
        static_transform.transform.translation.x = utm_easting
        static_transform.transform.translation.y = utm_northing
        static_transform.transform.translation.z = 0.0
        
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0
        
        self.broadcaster.sendTransform(static_transform)
        self.map_origin_set = True
        
        # Cancel the timer
        self.param_check_timer.cancel()
        
        self.get_logger().info(f'Map->odom transform set with map origin at UTM ({utm_easting:.2f}, {utm_northing:.2f}) Zone {utm_zone}')
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
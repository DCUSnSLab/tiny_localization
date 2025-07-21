#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

class GPSFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('gps_tf_broadcaster')
        
        self.broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Create static transform
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = "gps_utm"
        static_transform.child_frame_id = "odom_utm"
        
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0
        
        self.broadcaster.sendTransform(static_transform)
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
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class OdomFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_to_base_link_broadcaster')
        
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Declare parameter for output odom topic
        self.declare_parameter('output_odom_topic', '/odom/ekf')
        output_odom_topic = self.get_parameter('output_odom_topic').get_parameter_value().string_value
        
        # Create subscription to odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            output_odom_topic,
            self.handle_odom_pose,
            10
        )
        
        self.get_logger().info(f'Odom frame broadcaster initialized, listening to: {output_odom_topic}')
    
    def handle_odom_pose(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom_utm"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        
        self.broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    
    odom_broadcaster = OdomFrameBroadcaster()
    
    try:
        rclpy.spin(odom_broadcaster)
    except KeyboardInterrupt:
        pass
    finally:
        odom_broadcaster.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
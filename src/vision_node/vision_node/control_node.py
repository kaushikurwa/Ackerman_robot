#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import math


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self.subscription = self.create_subscription(
            Point,
            '/box_centroid',
            self.centroid_callback,
            10
        )
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_width   = 640        
        self.wheel_base    = 0.295      
        self.max_steer     = 0.524      
        self.Kp_steer      = 0.008     
        self.drive_speed   = 0.4        
        self.stop_area     = 18000.0    
        self.box_visible   = False
        self.lost_count    = 0          
        self.timer = self.create_timer(0.1, self.safety_check)
        self.last_msg_time = self.get_clock().now()
        self.get_logger().info('Control Node started — Ackermann steering active')

    def centroid_callback(self, msg: Point):
        self.last_msg_time = self.get_clock().now()
        twist = Twist()

        if msg.x < 0:
            self.lost_count += 1
            self.box_visible = False

            if self.lost_count <= 20:
                twist.linear.x  = self.drive_speed * 0.5
                twist.angular.z = 0.0
            else:
                twist.linear.x  = self.drive_speed * 0.4
                twist.angular.z = -0.3  

            self.cmd_pub.publish(twist)
            return

        self.box_visible = True
        self.lost_count  = 0
        cx          = msg.x
        image_cx    = self.image_width / 2.0
        pixel_error = cx - image_cx          
        box_area = msg.z
        if box_area >= self.stop_area:
            self.get_logger().info('TARGET REACHED — stopping')
            self.cmd_pub.publish(Twist())    # all-zero = stop
            return

        steer_angle = self.Kp_steer * pixel_error
        steer_angle = max(-self.max_steer, min(self.max_steer, steer_angle))
        steer_ratio   = abs(steer_angle) / self.max_steer
        forward_speed = self.drive_speed * (1.0 - 0.5 * steer_ratio)
        twist.linear.x  = forward_speed
        twist.angular.z = -steer_angle  
        self.cmd_pub.publish(twist)

        self.get_logger().info(
            f'pixel_err={pixel_error:+.0f}px  '
            f'steer={math.degrees(steer_angle):+.1f}°  '
            f'speed={forward_speed:.2f}m/s  '
            f'area={box_area:.0f}'
        )

    def safety_check(self):
        """Stop robot if no centroid message received for 0.5s"""
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if elapsed > 0.5:
            self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.subscription = self.create_subscription(
            Image,
            '/ackerman/camera/image_raw',
            self.image_callback,
            10
        )

        self.centroid_pub = self.create_publisher(Point, '/box_centroid', 10)

        self.bridge = CvBridge()
        self.get_logger().info('Vision Node started — looking for RED BOX')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0,   120,  70])
        upper_red1 = np.array([10,  255, 255])
        lower_red2 = np.array([170, 120,  70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN,  kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        best_box = None
        best_area = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 300:          
                continue

            x, y, cw, ch = cv2.boundingRect(cnt)
            bounding_area = cw * ch
            extent = area / bounding_area  

            epsilon = 0.04 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            corners = len(approx)

            if extent > 0.75 and 4 <= corners <= 6:
                if area > best_area:
                    best_area = area
                    best_box = (cnt, x, y, cw, ch)

        pt = Point()

        if best_box is not None:
            cnt, x, y, cw, ch = best_box
            cx = x + cw // 2
            cy = y + ch // 2

            pt.x = float(cx)
            pt.y = float(cy)
            pt.z = float(best_area)  

            self.get_logger().info(
                f'BOX found → centroid=({cx},{cy})  '
                f'error_from_center={cx - w//2}px  area={best_area:.0f}'
            )

            cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 2)
            cv2.rectangle(frame, (x, y), (x+cw, y+ch), (0, 255, 255), 2)
            cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
            cv2.putText(frame, f'BOX cx={cx}', (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            pt.x = -1.0
            pt.y = -1.0
            pt.z =  0.0
            self.get_logger().warn('BOX not found in frame')

        self.centroid_pub.publish(pt)
        cv2.line(frame, (w//2, 0), (w//2, h), (255, 255, 0), 1)
        cv2.imshow('Camera Feed', frame)
        cv2.imshow('Red Mask',    red_mask)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
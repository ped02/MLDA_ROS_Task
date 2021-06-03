import rclpy

from rclpy.node import Node
from std_msgs.msg import Header, String
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from cv_bridge import CvBridge

import cv2 as cv
import numpy as np

import math
import random
import time

class ImageReaderNode(Node):

    def __init__(self):
        super().__init__("image_reader")
        self.cv_bridge = CvBridge()
        self.image = None

        self.imageSubscription = self.create_subscription(Image, '/map_image', self.getMap, 5)
        self.get_logger().info("Starting image reader node")
    
    def getMap(self, imgMsg):
        self.image = self.cv_bridge.imgmsg_to_cv2(imgMsg)

def main(args=None):
    rclpy.init(args=args)

    imgReadNode = ImageReaderNode()

    while(rclpy.ok()):

        rclpy.spin_once(imgReadNode)

        if(imgReadNode.image is not None):
            cv.imshow("Map", imgReadNode.image)

        if(cv.waitKey(1) and 0xFF == ord('q')):
            break

    imgReadNode.destroy_now()
    rclpy.shutdown()
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
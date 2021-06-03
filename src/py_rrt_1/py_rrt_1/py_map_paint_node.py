import rclpy

from rclpy.node import Node
from std_msgs.msg import Header, String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2 as cv
import numpy as np

import math
import random
import time

import tkinter as tk
from tkinter import filedialog

class MapWindow(object):

    def __init__(self, windowName='Map Paint Tool'):
        self.windowName = windowName

        self.canvasSize = (500, 500, 3)
        self.minBrushSize = 1
        self.maxBrushSize = 30
        self.markerRadius = 3

        self.reset()

        self.start = None
        self.end = None

        self.x = 0
        self.y = 0
        self.mDown = False #mouse down

        self.brushRad = 3

        self.mode = 0
        #mode
        #0 - Draw
        #1 - Set Start
        #2 - Set Goal
        #3 - Erase

        #Create Window
        cv.namedWindow(self.windowName, cv.WINDOW_GUI_NORMAL)
        cv.setMouseCallback(self.windowName, self.mouseCallback)

    def reset(self):
        self.canvas = np.zeros(self.canvasSize, dtype=np.uint8)
        self.start = None
        self.end = None

    def mouseCallback(self, event, x, y, flags, params):

        if(event == cv.EVENT_MOUSEMOVE):
            #set x y
            self.x = x
            self.y = y

            if(self.mDown and (self.mode == 0 or self.mode == 3)):
                c = (0,0,0)
                if(self.mode == 0):
                    c = (255,255,255)
                cv.circle(self.canvas, (self.x, self.y), self.brushRad, c, thickness=-1)

        elif(event == cv.EVENT_LBUTTONDOWN):
            self.mDown = True

            if(self.mode == 0 or self.mode == 3):
                c = (0,0,0)
                if(self.mode == 0):
                    c = (255,255,255)
                cv.circle(self.canvas, (self.x, self.y), self.brushRad, c, thickness=-1)
            elif(self.mode == 1):
                self.start = (self.x, self.y)
            elif(self.mode == 2):
                self.end = (self.x, self.y)

        elif(event == cv.EVENT_LBUTTONUP):
            self.mDown = False

        pass

    def keyboardPoll(self):
        #Return should quit?

        events = dict()

        k = cv.waitKey(1) & 0xFF
        if(k == ord('q')):
            #Quit
            events['quit'] = True

        if(k == ord('y')):
            #Increase Brush Size
            if(self.brushRad < self.maxBrushSize):
                self.brushRad += 1

        if(k == ord('h')):
            #Decrease Brush Size
            if(self.brushRad > self.minBrushSize):
                self.brushRad -= 1

        if(k == ord('r')):
            #Reset
            self.reset()

        if(k == ord('d')):
            #Draw
            self.mode = 0

        if(k == ord('f')):
            #Start
            self.mode = 1

        if(k == ord('g')):
            #Goal/End
            self.mode = 2

        if(k == ord('e')):
            #Eraser
            self.mode = 3

        if(k == ord('s')):
            #Send
            events['send'] = True

        if(k == ord('c')):
            #Save
            events['save'] = True

        if(k == ord('o')):
            #Open
            events['open'] = True

        return events


    def display(self):

        dispImg = self.canvas.copy()

        #Draw start end points
        if(self.start is not None):
            dispImg = cv.circle(dispImg, self.start, self.markerRadius, (0, 255, 0), thickness=-1)
        
        if(self.end is not None):
            dispImg = cv.circle(dispImg, self.end, self.markerRadius, (0, 0, 255), thickness=-1)

        #Preview Mouse
        if(self.mode == 0 or self.mode == 3):
            c = (125, 125, 125)
            if(self.mode == 0):
                c = (255, 255, 255)
            dispImg = cv.circle(dispImg, (self.x, self.y), self.brushRad, c, thickness=1)
        elif(self.mode == 1 or self.mode == 2):
            c = (0, 255, 0)
            if(self.mode == 2):
                c = (0, 0, 255)
            rHalf = self.markerRadius//2
            dispImg = cv.rectangle(dispImg, (self.x-rHalf, self.y-rHalf), (self.x+rHalf, self.y+rHalf), c, thickness=1)

        cv.imshow(self.windowName, dispImg)

class ImagePaintPublisherNode(Node):
    def __init__(self):
        super().__init__("map_paint_publisher_node")
        self.cv_bridge = CvBridge()

        self.mapPublisher = self.create_publisher(Image, "/map_image", 5)
        self.startPublisher = self.create_publisher(Point, "/start_point", 5)
        self.goalPublisher = self.create_publisher(Point, "/goal_point", 5)
        self.get_logger().info("Starting Map Paint Publisher Node")
        
    def publishImage(self, img):
        imgB = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        imgMsg = self.cv_bridge.cv2_to_imgmsg(imgB, encoding='8UC1')

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map" #For rviz
        imgMsg.header = header

        self.get_logger().info("Publishing Image")
        self.mapPublisher.publish(imgMsg)
    
    def publishStart(self, start):
        pointMsg = Point(x=float(start[0]), y=float(start[1]), z=0.0)
        self.get_logger().info("Publishing Start Point")
        self.startPublisher.publish(pointMsg)

    def publishGoal(self, goal):
        pointMsg = Point(x=float(goal[0]), y=float(goal[1]), z=0.0)
        self.get_logger().info("Publishing End Point")
        self.goalPublisher.publish(pointMsg)

def askSaveFile():
    root = tk.Tk()
    root.withdraw()

    fDir = filedialog.asksaveasfilename(filetypes=[("Image", ".png .pgm")])

    result = fDir

    if(len(fDir) == 0):
        result = None
    root.destroy()

    return result

def askOpenFile():
    root = tk.Tk()
    root.withdraw()

    fDir = filedialog.askopenfilename(filetypes=[("Image", ".png .pgm")])

    result = fDir

    if(len(fDir) == 0):
        result = None
    root.destroy()

    return result


def main(args=None):
    rclpy.init(args=args)

    imagePub = ImagePaintPublisherNode()
    window = MapWindow()

    controlString = ("Controls - D: Draw   E: Erase   Y: + Brush Size   H: - Brush Size   "
                    "F:Place Start   G: Place Goal   O: Open File   S: Send To Map Server   C: Save File on Disk")

    imagePub.get_logger().info(controlString)

    while(rclpy.ok()):
        events = window.keyboardPoll()

        if('quit' in events):
            break

        if('send' in events):
            #Send
            imagePub.publishImage(window.canvas)

            if(window.start is not None):
                imagePub.publishStart(window.start)
            
            if(window.end is not None):
                imagePub.publishGoal(window.end)
        
        if('save' in events):
            fDir = askSaveFile()
            if(fDir is not None):
                imagePub.get_logger().info(f"Writing map to {fDir}")
                cv.imwrite(fDir, window.canvas)

        if('open' in events):
            fDir = askOpenFile()
            if(fDir is not None):
                imagePub.get_logger().info(f"Reading map at {fDir}")
                img = cv.imread(fDir)
                window.canvas = img

        window.display()

    cv.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
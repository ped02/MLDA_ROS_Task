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

#Utilities Function
def rotation2D(angle):
    #Angle in radians CCW
    cT = math.cos(angle)
    sT = math.sin(angle)
    return np.array([[cT, -sT], [sT, cT]])

#RRT
class RRT(object):
    def __init__(self, mapImgG: np.ndarray):
        #Map in binary 'color space'
        self.reset()
        self.setMap(mapImgG)
    
    def reset(self):
        self.nodes = []
        self.nodeChild = dict()
        self.nodeParent = dict()
        self.path = []
        self.reach = []
        self.samples = []
        
        self.map = None
        self.mShape = None
        
        self.start = None
        self.end = None
        
        self.found = False
    
    def setMap(self, mapImgG: np.ndarray):
        self.map = mapImgG
        self.mShape = np.array(self.map.shape)
        
    def setGoals(self, startPoint: tuple, endPoint: tuple):
        self.start = startPoint
        self.end = endPoint
        
        self.nodes.append(self.start)
        self.nodeChild[self.start] = []
        self.nodeParent[self.start] = None
        
    def roundClipToEdge(self, vertices):
        #vertices is (Nx2) array
        return np.clip(np.round(vertices).astype(np.int32), [0,0], self.mShape[::-1]-1)
    
    def addNode(self, dQ: int, greedyBias: float = 0.05):
        '''
            dQ: positive integer of max step size in pixel
            greedyBias: Percentage weight on sampling the goal state
        '''
        #Random Config
        while True:
            
            if(random.random() < (1-greedyBias)):
                rConf = self.roundClipToEdge(np.multiply(self.mShape[::-1], np.random.uniform(size=(2))))
            else:
                rConf = np.array(self.end)
            
            if(self.map[rConf[1], rConf[0]] == 0):
                break
        
        self.samples.append(tuple(rConf))
        
        #Nearest Node
        nodeVec = np.array(self.nodes, dtype=np.float32)
        nodeDiff = nodeVec-rConf
        nodeDists = np.sum(np.multiply(nodeDiff, nodeDiff), axis=1)
        nearestNodeInd = np.argmin(nodeDists)
        
        #Add new node by dQ
        #Get Target Location
        nearestNode = self.nodes[nearestNodeInd]
        nearestNodeNp = np.array(nearestNode)
        moveDir = rConf - nearestNodeNp
        angle = np.arctan2(moveDir[1], moveDir[0])
        
        #Construct Path for collision detection
        pathV = np.vstack([np.arange(0, math.ceil(dQ), 1, dtype=np.float32), np.zeros(dQ)])
        pathT = np.matmul(rotation2D(angle), pathV)
        
        pixelsToCheckD = self.roundClipToEdge(pathT.T + nearestNodeNp)
        pixelsCheckIndexUnique = pixelsToCheckD[:,0]*self.mShape[1]  + pixelsToCheckD[:,1]
        uniquePixels, uniquePixelsIndex = np.unique(pixelsCheckIndexUnique, return_index = True)
        uniquePixelsIndex.sort()
        slicePixel = pixelsToCheckD[uniquePixelsIndex]
        slicePixelDiff = np.concatenate([np.diff(slicePixel, axis=0), np.zeros((1,2), dtype=np.int32)])
        
        checkFreeCardinal = np.logical_or(self.map[slicePixel[:,1]+slicePixelDiff[:,1],slicePixel[:,0]] == 0, self.map[slicePixel[:,1],slicePixel[:,0]+slicePixelDiff[:,0]] == 0)
        checkFreeBlocks = np.logical_and(self.map[slicePixel[:,1], slicePixel[:,0]] == 0, checkFreeCardinal)
        
        hitInd = -1 if not checkFreeBlocks[0] else len(checkFreeBlocks)-1 - np.argmin(np.cumsum(np.logical_not(checkFreeBlocks).astype(np.int32))[::-1])

        if(hitInd == -1):
            return self.found
        
        newConf = rConf if np.any(np.all(slicePixel[:hitInd] == np.array(rConf), axis=1)) else slicePixel[hitInd]
        newConf = np.array(self.end) if np.any(np.all(slicePixel[:hitInd] == np.array(self.end), axis=1)) else newConf
        newConfT = tuple(newConf)
        
        if(newConfT not in self.nodes):
            
            if(np.all(newConf - rConf < 0.5)):
                self.reach.append(newConfT)
        
            self.nodes.append(newConfT)
            self.nodeChild[newConfT] = []
            self.nodeParent[newConfT] = nearestNode

            self.nodeChild[nearestNode].append(newConfT)
            
            if(newConfT == self.end):
                self.found = True
                #Construct Path
                self.constructPath()
        
        return self.found
    
    def constructPath(self):
        #Construct Path and assign to self.path
        if(not self.found):
            self.path = []
        else:
            tempN = self.end
            while(tempN is not None):
                self.path.append(tempN)
                tempN = self.nodeParent[tempN]
    
    def showState(self, pointerRad : int = 5, lineWidth = 2, path=True, sample=False, reach=False):
        '''
            Input
            pointerRad: Radius of vertices
            lineWidth: Width of edge in graph
            path: Highlight path in different color
            sample: Show latest point sampled
            reach: Show if the point reached the random config

            Output
            img: RGB Image with tree drawn
        '''

        #Return map marked
        img = 255*np.stack([self.map, self.map, self.map], axis=-1)
        
        #return img
        
        #Edges
        for k,v in self.nodeParent.items():
            if(v is None):
                continue
            img = cv.line(img, k, v, (0,255,255), lineWidth)
            
        #Nodes
        nodes = self.nodes[:]
        if(self.end not in self.nodes):
            nodes.append(self.end)

        for n in nodes:
            c = (0, 255, 255)
            if(n == self.start):
                c = (0, 255, 0)
            elif(n == self.end):
                c = (255, 0, 0)
            img = cv.circle(img, n, pointerRad, c, thickness=-1)
            
        #Path
        if(len(self.path) > 0 and path):
            for i in range(len(self.path)-1):
                img = cv.line(img, self.path[i], self.path[i+1], (0,0,255), lineWidth)
            for n in self.path:
                img = cv.circle(img, n, pointerRad, (0,0,255), thickness=-1)
        
        #Reach
        if(reach):
            for n in self.reach:
                img = cv.circle(img, n, pointerRad, (255, 0 ,255), thickness=-1)
        
        #Sample
        if(sample):
            if(len(self.samples) > 0):
                img = cv.circle(img, self.samples[-1], pointerRad, (255, 255 ,0), thickness=-1)
            
        return img

class PlannerNode(Node):
    def __init__(self):
        super().__init__("planner_node")
        self.cv_bridge = CvBridge()
        self.map = None
        self.mapW = None
        self.mapH = None
        self.start = None
        self.goal = None
        self.rrt = None
        self.stepSize = None
        self.nodeCount = None
        self.goalBias = None

        self.visualizationPublisher = self.create_publisher(Image, "/rrt_WIP", 5)
        self.pathPublisher = self.create_publisher(Path, "/path", 5)

        self.mapSubscription = self.create_subscription(OccupancyGrid, "/map", self.get_map, 5)
        self.startSubscription = self.create_subscription(Point, "/start_point", self.get_start, 5)
        self.goalSubscription = self.create_subscription(Point, "/goal_point", self.get_goal, 5)

        self.startSignalSubscription = self.create_subscription(String, "/start_plan", self.start_search, 5)

        self.get_logger().info("Starting Planner Node")
    
    def get_map(self, data):
        #nav_msg.msg.OccupancyGrid
    
        self.mapW = data.info.width
        self.mapH = data.info.height

        self.map = (np.array(data.data, dtype=np.uint8).reshape(self.mapH, self.mapW) / 100).astype(np.uint8)
        
        self.get_logger().info(f"Received Map - {self.mapW} x {self.mapH}")
        #cv.imshow("Image", self.map)

    def get_start(self, data):
        #geometry_msg.msg.Point
        self.start = (int(data.x), int(data.y))
        self.get_logger().info(f"Received Starting Point - {self.start} - {data.x} , {data.y}")
    
    def get_goal(self, data):
        #geometry_msg.msg.Point
        self.goal = (int(data.x), int(data.y))
        self.get_logger().info(f"Received Goal Point - {self.goal}")

    def publishImage(self, img):
        self.get_logger().info("Publishing WIP")
        imgMsg = self.cv_bridge.cv2_to_imgmsg(img, encoding="rgb8")

        #Set Headers
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map" #For rviz
        imgMsg.header = header

        self.visualizationPublisher.publish(imgMsg)
    
    def publishPath(self, path):
        pathMsg = Path()

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map" #For rviz

        poses = []

        for po in path[::-1]:
            p = PoseStamped()

            p_point = Point(x = float(po[0])*0.01, y = -float(po[1])*0.01, z = 0.0)
            p_quat = Quaternion(x = 1.0, y = 0.0, z = 0.0, w = 0.0)

            p_pose = Pose(position=p_point, orientation=p_quat)
            p = PoseStamped(header=header,pose=p_pose)
 
            poses.append(p)
        
        pathMsg.header = header
        pathMsg.poses = poses

        self.get_logger().info("Publishing Path")
        self.pathPublisher.publish(pathMsg)

    def start_search(self, data):
        self.get_logger().info("Calling Search")
        self.searchRRT(visualize = True)
    
    def searchRRT(self, stepSize=20, maxCount=5000, goalBias=0.05, iterationLimit=100000, visualize=False, visualizeFrequency=50):
        #Errors
        if(self.map is None):
            self.get_logger().info("Map not found")
            return None

        if(self.start is None):
            self.get_logger().info("Start point not specified")
            return None

        if(self.goal is None):
            self.get_logger().info("Goal point not specified")
            return None
            
        if(self.start[0] < 0 or self.start[0] > self.mapW or self.start[1] < 0 or self.start[1] > self.mapH or self.map[self.start[1], self.start[0]] != 0):
            self.get_logger().info("Invalid starting point")
            return None
            
        if(self.goal[0] < 0 or self.goal[0] > self.mapW or self.goal[1] < 0 or self.goal[1] > self.mapH or self.map[self.goal[1], self.goal[0]] != 0):
                self.get_logger().info("Invalid goal point")
                return None

        #Info
        if(self.stepSize is None):
            self.get_logger().info("No step size set. Default step size: {stepSize}}")

        if(self.goalBias is None):
            self.get_logger().info("No goal bias set. Default goal bias: {goalBias}}")

        if(self.nodeCount is None):
            self.get_logger().info("No node count limit set. Default limit: {maxCount}}")

        self.get_logger().info(f"Starting search: Start - {self.start} Goal - {self.goal}")

        self.rrt = RRT(self.map)
        self.rrt.setGoals(self.start, self.goal)
        
        step = stepSize
        if(self.stepSize is not None):
            step = self.stepSize

        nodeLimit = maxCount
        if(self.nodeCount is not None):
            nodeLimit = self.nodeCount

        iterationCount = 0
        found = False
        while(len(self.rrt.nodes) < nodeLimit):
            found = self.rrt.addNode(step, greedyBias=goalBias)

            if(visualize and (((iterationCount+1) % visualizeFrequency) == 0)):
                self.get_logger().info(f"Publishing Progress Iteration: {iterationCount+1} - Node Count: {len(self.rrt.nodes)}")
                state = self.rrt.showState()
                self.publishImage(state)
                #self.get_logger().info(f"State: {state}")

            if(found):
                self.get_logger().info("Goal Found")
                
                break

            if(iterationCount > iterationLimit):
                self.get_logger().info("Iteration Limit Hits. Probably some error because this shoudln't happen.")
                break

            iterationCount += 1

        if(found):
            #Publish Path
            self.get_logger().info(f"Publishing Found Image - Node Count: {len(self.rrt.nodes)}")
            state = self.rrt.showState()
            self.publishImage(state)

            self.get_logger().info("Publishing Path")
            self.publishPath(self.rrt.path)
        else:
            self.get_logger().info("Path not found!")


def main(args=None):
    rclpy.init(args=args)

    planner_node = PlannerNode()

    rclpy.spin(planner_node)
    planner_node.destroy_now()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
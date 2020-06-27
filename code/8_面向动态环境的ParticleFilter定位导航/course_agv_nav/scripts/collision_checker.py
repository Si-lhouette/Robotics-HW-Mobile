#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose,Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from course_agv_nav.srv import Plan,PlanResponse
import math
from threading import Lock,Thread
import numpy as np
import matplotlib.pyplot as plt

MAX_LASER_RANGE = 30

class Checker():
    def __init__(self):
        self.threshold = 0.1 # use to check collision
        self.first_map = True

        self.path = np.zeros([3,0])
        # x y yaw
        self.robot_x = np.zeros([3,1])
        # ros topic
        self.path_sub = rospy.Subscriber('/course_agv/global_path',Path,self.pathCallback)
        self.map_sub = rospy.Subscriber('/grid_map_mine', OccupancyGrid, self.mapCallback)
        # self.laser_sub = rospy.Subscriber('/course_agv/laser/scan',LaserScan,self.laserCallback)
        self.collision_pub = rospy.Publisher('/collision_checker_result',Bool,queue_size=5)
        self.replan_client = rospy.ServiceProxy('/course_agv/global_plan',Plan)
        self.marker_pub = rospy.Publisher('/collision_marker',Marker,queue_size=1)
        ## global real pose TODO replace with slam pose
        self.pose_sub = rospy.Subscriber('/gazebo/course_agv__robot_base',Pose,self.poseCallback)

        self.lock = Lock()
        # plt.ion()
        # fig = plt.figure()
        # self.ax = fig.add_subplot(111)

    def poseCallback(self,msg):
        p = msg.position
        o = msg.orientation
        e = tf.transformations.euler_from_quaternion([o.x,o.y,o.z,o.w])
        self.robot_x = np.array([[p.x,p.y,e[2]]]).T

    def pathCallback(self,msg):
        self.lock.acquire()
        self.path = self.pathToNumpy(msg) # shape (3,pose_num)
        print("Check node get path! shape: ",self.path.shape)
        self.lock.release()

    def mapCallback(self,msg):
        if(self.first_map):
            self.map_height = msg.info.height
            self.map_width = msg.info.width
            self.map_reso = msg.info.resolution
            self.first_map = False
        self.map = np.array(msg.data).reshape((-1,msg.info.height)).transpose()
        if self.collision_check():
            self.publish_collision()
            try:
                resp = self.replan_client.call()
                rospy.logwarn("Service call res: %s"%resp)
            except rospy.ServiceException, e:
                rospy.logwarn("Service call failed: %s"%e)
        pass

    # def laserCallback(self,msg):
    #     np_msg = self.laserToNumpy(msg)
    #     obs = self.u2T(self.robot_x).dot(np_msg)
	#     ## TODO update robot global pose
    #     if self.collision_check(obs):
    #         self.publish_collision()

    def collision_check(self):
        self.lock.acquire()
        res = False
        if self.path.shape[1] == 0:
            self.lock.release()
            return res

        
        blockx = []
        blocky = []
        px = []
        py = []
        for i in range(self.path.shape[1]):
            xinx = self.global2inx(self.path[0,i])
            yinx = self.global2inx(self.path[1,i])
            px.append(xinx)
            py.append(yinx)
            for m in range(xinx-2,xinx+3):
                for n in range(yinx-2,yinx+3):
                    if m < 0 or n < 0 or m >= self.map_height or n >= self.map_width:
                        continue
                    blockx.append(m)
                    blocky.append(n)
                    if self.map[m,n] > 50:
                        res = True
                        print(m,n)
                        print("Collsion!!!!!!!!!!!!!!!!")
                        break 
        # if(res):
        #     plt.figure()
        #     plt.plot(blockx,blocky,'r.')
        #     plt.plot(px,py,'g.')
        #     plt.show()

        self.lock.release()
        return res

    def global2inx(self, gx):
        xinx = int((gx+10.0)/self.map_reso)
        return xinx

    def publish_collision(self):
        res = Bool()
        res.data = True
        self.collision_pub.publish(res)

    def pathToNumpy(self,msg):
        pos = np.ones([0,3])
        for i in range(len(msg.poses)):
            p = msg.poses[i].pose.position
            pos = np.append(pos,[[p.x,p.y,1]],axis=0)
        return pos.T

    # def laserToNumpy(self,msg):
    #     total_num = len(msg.ranges)
    #     pc = np.ones([3,total_num])
    #     range_l = np.array(msg.ranges)
    #     range_l[range_l == np.inf] = MAX_LASER_RANGE
    #     angle_l = np.linspace(msg.angle_min,msg.angle_max,total_num)
    #     pc[0:2,:] = np.vstack((np.multiply(np.cos(angle_l),range_l),np.multiply(np.sin(angle_l),range_l)))
    #     return pc

    def T2u(self,t):
        dw = math.atan2(t[1,0],t[0,0])
        u = np.array([[t[0,2],t[1,2],dw]])
        return u.T
    
    def u2T(self,u):
        w = u[2]
        dx = u[0]
        dy = u[1]

        return np.array([
            [ math.cos(w),-math.sin(w), dx],
            [ math.sin(w), math.cos(w), dy],
            [0,0,1]
        ])

def main():
    rospy.init_node('collision_checker_node')
    n = Checker()
    rospy.spin()
    pass

def test():
    pass

if __name__ == '__main__':
    main()
    # test()

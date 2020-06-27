#!/usr/bin/env python
#coding=utf-8
import rospy
import tf
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from course_agv_nav.srv import Plan,PlanResponse

import time
import math

import numpy as np
import matplotlib.pyplot as plt


# from a_star import AStarPlanner

#Sub最新mapping节点MAP & check res
#若check返回Plan.srv = True，Astar replan，Pub global_path

class GlobalPlanner:
    def __init__(self):
        # TODO  : 
        #   tf listener : 
        #   publisher   : /course_agv/global_path
        #   subscriver  : /course_agv/goal
        #   service     : /course_agv/global_plan
        #   initialize for other variable
        self.plan_sx = 0.0
        self.plan_sy = 0.0
        self.plan_gx = 8.0
        self.plan_gy = -8.0
        self.plan_grid_size = 0.3
        self.plan_robot_radius = 0.6
        self.plan_ox = []
        self.plan_oy = []
        self.plan_rx = []
        self.plan_ry = []

        self.first_map = True

        self.motion = self.get_motion_model()
        self.g_w = 0.5
        self.h_w = 1

        
        #example
        self.tf = tf.TransformListener()
        self.map_sub = rospy.Subscriber('/grid_map_mine', OccupancyGrid, self.mapCallback)
        self.goal_sub = rospy.Subscriber('/course_agv/goal',PoseStamped,self.goalCallback)
        self.plan_srv = rospy.Service('/course_agv/global_plan',Plan,self.replan)
        self.path_pub = rospy.Publisher('/course_agv/global_path',Path,queue_size = 1)

    class Node:
        def __init__(self, x, y, g, f, pind):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.g = g
            self.f = f
            self.pind = pind

        def __str__(self):
            return str(self.x) + "," + str(self.y)


    def goalCallback(self,msg):
        self.plan_goal = msg
        self.plan_gx = msg.pose.position.x
        self.plan_gy = msg.pose.position.y
        print("get new goal!!! ")
        self.replan(0)

    def updateGlobalPose(self):
        try:
            self.tf.waitForTransform("/map", "/robot_base", rospy.Time(), rospy.Duration(4.0))
            (self.trans,self.rot) = self.tf.lookupTransform('/map','/robot_base',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        self.plan_sx = self.trans[0]
        self.plan_sy = self.trans[1]

    def mapCallback(self,msg):
        if(self.first_map):
            self.map_height = msg.info.height
            self.map_width = msg.info.width
            self.map_reso = msg.info.resolution
            self.first_map = False
        self.map = np.array(msg.data).reshape((-1,msg.info.height)).transpose()
        pass



    def replan(self,req):
        print('get request for replan!!!!!!!!')
        # 更新当前位姿
        self.updateGlobalPose()
        # A*
        self.plan_rx,self.plan_ry = self.Astar()
        self.publishPath()
        res = True
        return PlanResponse(res)


    def publishPath(self):
        path = Path()
        path.header.seq = 0
        path.header.stamp = rospy.Time(0)
        path.header.frame_id = 'map'
        for i in range(len(self.plan_rx)):
            pose = PoseStamped()
            pose.header.seq = i
            pose.header.stamp = rospy.Time(0)
            pose.header.frame_id = 'map'
            pose.pose.position.x = self.plan_rx[len(self.plan_rx)-1-i]
            pose.pose.position.y = self.plan_ry[len(self.plan_rx)-1-i]
            pose.pose.position.z = 0.01
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            path.poses.append(pose)
        self.path_pub.publish(path)


    def Astar(self):
        """
        A star path search

        input:
            sx: start x position [m]
            sy: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        nstart = self.Node(self.global2inx(self.plan_sx),
                           self.global2inx(self.plan_sy), 0.0, 0.0, -1)
        ngoal = self.Node(self.global2inx(self.plan_gx),
                          self.global2inx(self.plan_gy), 0.0, 0.0, -1)
        nstart.f = self.calc_dis(nstart, ngoal)
        open_set, closed_set = dict(), dict()           #
        open_set[self.calc_grid_index(nstart)] = nstart

        while 1:
            # Find the smallest in open set & Remove it
            min_f = 1e8
            

            for tinx, tnode in open_set.items():
                if tnode.f < min_f:
                    min_f = tnode.f
                    min_inx = tinx

            min_node = open_set.pop(min_inx)

            # Add it to closed set
            closed_set[self.calc_grid_index(min_node)] = min_node

            # Is Goal?
            if min_node.x == ngoal.x and min_node.y == ngoal.y:
                print("Search arrive goal!")
                ngoal = min_node
                break

            # Handle all neighbor - Relaxtion 
            for motion_i in self.motion:
                tnode = self.Node(min_node.x + motion_i[0], min_node.y + motion_i[1], 0.0, 0.0, self.calc_grid_index(min_node))
                tnode.g = min_node.g + motion_i[2]
                tnode.f = self.g_w * tnode.g + self.h_w *self.calc_dis(tnode, ngoal)
                tnode_inx = self.calc_grid_index(tnode)

                # check obstacle & if in closed set
                if self.verify_node(tnode) and not(closed_set.has_key(tnode_inx)):
                    # if not in open set -> it's a new point
                    if not open_set.has_key(tnode_inx):
                        # add it to openset
                        open_set[tnode_inx] = tnode

                    # if in open set
                    else:
                        if open_set[tnode_inx].g > tnode.g:
                            open_set.pop(tnode_inx)
                            open_set[tnode_inx] = tnode


        rx, ry = self.calc_final_path(ngoal, closed_set)
        # rx, ry = self.smooth(rx,ry)

        # plt.figure()
        # plt.grid(True)
        # plt.plot(rx,ry,'b')
        # plt.plot(rx,ry,'g.')


        # rxn, ryn = self.filter_path(rx,ry)
        # plt.plot(rxn,ryn,'r.')
        # plt.show()

        return rx, ry

    def calc_final_path(self, ngoal, closedset):
        # generate final course
        rx, ry = [self.calc_grid_position(ngoal.x)], [
            self.calc_grid_position(ngoal.y)]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            rx.append(self.calc_grid_position(n.x))
            ry.append(self.calc_grid_position(n.y))
            pind = n.pind

        return rx, ry

    def smooth(self, x, y):
        for i in range(len(x)-1,1,-2):
            midx=(x[i]+x[i-2])/2
            midy=(y[i]+y[i-2])/2

        if(self.map[midx,midy]<50):
            x[i-1]=midx
            y[i-1]=midy
        return  x,y


    def global2inx(self, gx):
        xinx = int((gx+10.0)/self.map_reso)
        return xinx
    
    def calc_dis(self,n1, n2):
        d = math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_index(self, node):
        return int(node.y * self.map_height + node.x)

    def calc_grid_position(self, index):
        pos = index * self.map_reso - 10.0
        return pos

    def verify_node(self, node):
        # collision check
        for m in range(node.x-2,node.x+3):
            for n in range(node.y-2,node.y+3):
                if m < 0 or n < 0 or m >= self.map_height or n >= self.map_width:
                    continue
                if self.map[m,n] > 50:
                    return False

        return True
    
    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion

def main():
    rospy.init_node('global_planner')
    gp = GlobalPlanner()
    rospy.spin()
    pass

if __name__ == '__main__':
    main()

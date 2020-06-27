#!/usr/bin/env python

import math

import matplotlib.pyplot as plt

show_animation = False


class AStarPlanner:

    def __init__(self, ox, oy, reso, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        reso: grid resolution [m]
        rr: robot radius[m]
        """

        self.reso = reso
        self.rr = rr
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()
        self.g_w = 1
        self.h_w = 0.5
 
    class Node:
        def __init__(self, x, y, g, f, pind):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.g = g
            self.f = f
            self.pind = pind

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.pind)

    def planning(self, sx, sy, gx, gy):
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

        nstart = self.Node(self.calc_xyindex(sx, self.minx),
                           self.calc_xyindex(sy, self.miny), 0.0, 0.0, -1)
        ngoal = self.Node(self.calc_xyindex(gx, self.minx),
                          self.calc_xyindex(gy, self.miny), 0.0, 0.0, -1)
        nstart.f = self.calc_dis(nstart, ngoal)
        open_set, closed_set = dict(), dict()           #
        open_set[self.calc_grid_index(nstart)] = nstart

        while 1:
            # TODO here
            # Remove the item from the open set
            # Add it to the closed set
            # expand_grid search grid based on motion model



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

        plt.figure()
        plt.grid(True)
        plt.plot(rx,ry,'b')
        plt.plot(rx,ry,'g.')


        # rxn, ryn = self.filter_path(rx,ry)
        # plt.plot(rxn,ryn,'r.')
        # plt.show()

        return rx, ry

    def calc_final_path(self, ngoal, closedset):
        # generate final course
        rx, ry = [self.calc_grid_position(ngoal.x, self.minx)], [
            self.calc_grid_position(ngoal.y, self.miny)]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            rx.append(self.calc_grid_position(n.x, self.minx))
            ry.append(self.calc_grid_position(n.y, self.miny))
            pind = n.pind

        return rx, ry


    def filter_path(self, rx, ry):
        rx.reverse()
        ry.reverse()

        rxn = [rx[0]]
        ryn = [ry[0]]
        cnt = 0
        for i in range(len(rx)-2):
            if math.fabs(math.atan2(ry[i+1]-ry[i],rx[i+1]-rx[i]) - math.atan2(ry[i+2]-ry[i+1],rx[i+2]-rx[i+1])) < 1e-3:
                cnt += 1
                if cnt == 1:
                    rxn.append(rx[i+1])
                    ryn.append(ry[i+1])                   
            else:
                cnt = 0
                rxn.append(rx[i+1])
                ryn.append(ry[i+1])

        rxn.append(rx[-1])
        ryn.append(ry[-1])

        rxn.reverse()
        ryn.reverse()

        return rxn, ryn

    @staticmethod
    def calc_dis(n1, n2):
        d = math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, minp):
        """
        calc grid position

        :param index:
        :param minp:
        :return:
        """
        pos = index * self.reso + minp
        return pos

    def calc_xyindex(self, position, min_pos):
        return int(round((position - min_pos) / self.reso))

    def calc_grid_index(self, node):
        return int((node.y - self.miny) * self.xwidth + (node.x - self.minx))

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.minx)
        py = self.calc_grid_position(node.y, self.miny)

        if px < self.minx:
            return False
        elif py < self.miny:
            return False
        elif px >= self.maxx:
            return False
        elif py >= self.maxy:
            return False

        # collision check
        if self.obmap[int(node.x)][int(node.y)]:
            return False

        return True

    # Generate obstacle_map[x_width][y_width] index of resolution
    def calc_obstacle_map(self, ox, oy):

        self.minx = int(round(min(ox)))
        self.miny = int(round(min(oy)))
        self.maxx = int(round(max(ox)))
        self.maxy = int(round(max(oy)))
        print("minx:", self.minx)
        print("miny:", self.miny)
        print("maxx:", self.maxx)
        print("maxy:", self.maxy)

        self.xwidth = int(round((self.maxx - self.minx) / self.reso))
        self.ywidth = int(round((self.maxy - self.miny) / self.reso))
        print("xwidth:", self.xwidth)
        print("ywidth:", self.ywidth)

        # obstacle map generation
        self.obmap = [[False for i in range(self.ywidth)]
                      for i in range(self.xwidth)]
        for ix in range(self.xwidth):
            x = self.calc_grid_position(ix, self.minx)
            for iy in range(self.ywidth):
                y = self.calc_grid_position(iy, self.miny)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y) #
                    if d <= self.rr:
                        self.obmap[ix][iy] = True
                        break

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

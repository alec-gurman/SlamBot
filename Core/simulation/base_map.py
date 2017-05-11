#!/usr/bin/env python

import matplotlib.pyplot as plt

class base_map(object):

    def __init__(self):
        self.robot_pose = ('robot',0.2,0.2)
        self.goal_1 = ('goal 1',0.635,1.350)
        self.goal_2 = ('goal 2',1.44,1.290)
        self.landmark_1 = ('27', 15.1956, -9.5014)
        self.landmark_2 = ('29', -8.3379, 5.5814)
        self.landmark_3 = ('38', 9.1698, 16.3630)
        self.landmark_4 = ('45', 0.4456, -5.6378)
        self.landmark_5 = ('57', 15.5354, 12.9258)
        # self.landmark_1 = ('27', 1.060, 0.125)
        # self.landmark_2 = ('29', 1.775, 0.695)
        # self.landmark_3 = ('38', 1.790, 1.745)
        # self.landmark_4 = ('45', 0.975, 1.790)
        # self.landmark_5 = ('57', 0.180, 1.380)

    def draw_base_map(self):

        labels = [self.landmark_1[0],self.landmark_2[0],self.landmark_3[0],self.landmark_4[0],self.landmark_5[0]]
        landmark_x = [self.landmark_1[1],self.landmark_2[1],self.landmark_3[1],self.landmark_4[1],self.landmark_5[1]]
        landmark_y = [self.landmark_1[2],self.landmark_2[2],self.landmark_3[2],self.landmark_4[2],self.landmark_5[2]]
        goal_x = [self.goal_1[1],self.goal_2[1]]
        goal_y = [self.goal_1[2],self.goal_2[2]]
        robot_x = [self.robot_pose[1]]
        robot_y = [self.robot_pose[2]]

        plt.scatter(landmark_x,landmark_y,marker=(8,2,0),color='k',s=250)
        #plt.scatter(goal_x,goal_y,marker="D",facecolors='none',edgecolors='k',s=100)
        #plt.scatter(robot_x,robot_y,marker=">",facecolors='none',edgecolors='b',s=100)
        for index,label in enumerate(labels):
            y_list = landmark_y
            x_list = landmark_x
            plt.annotate(
                label,
                xy=(x_list[index], y_list[index]), xytext=(20, 10),
                textcoords='offset points', ha='right', va='bottom')

    def get_landmark_pos(self, landmark):
        if landmark == 0:
            return self.landmark_1
        if landmark == 1:
            return self.landmark_2
        if landmark == 2:
            return self.landmark_3
        if landmark == 3:
            return self.landmark_4
        if landmark == 4:
            return self.landmark_5

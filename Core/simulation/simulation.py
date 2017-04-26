#!/usr/bin/env python

'''

SIMULATION SCRIPT WEEK 8

'''

import time
import math
import matplotlib.pyplot as plt

ROBOT_X = 0.0
ROBOT_Y = 0.0
ROBOT_HEADING = 0.0

ROBOT_V = 1
ROBOT_W = 50

GOAL_X = 2.0
GOAL_Y = 3.0

FREQUENCY = 0.1

FOUND_GOAL = False

distance_travelled = ROBOT_V * FREQUENCY
delta_theta = ROBOT_W * FREQUENCY

heading_radians = math.atan2(GOAL_Y - ROBOT_Y, GOAL_X - ROBOT_X) #get heading angle
heading_angle = math.degrees(heading_radians) #56 degrees

stopped_turning = False
stopped_driving = False

x_axis = []
y_axis = []
heading_axis = []

while not FOUND_GOAL:

    heading_radians = math.atan2(GOAL_Y - ROBOT_Y, GOAL_X - ROBOT_X) #get heading angle
    heading_angle = math.degrees(heading_radians) #56 degrees

    robot_heading = heading_angle - ROBOT_HEADING

    ROBOT_X = ROBOT_X + distance_travelled * math.cos(math.radians(ROBOT_HEADING))
    ROBOT_Y = ROBOT_Y + distance_travelled * math.sin(math.radians(ROBOT_HEADING))

    if robot_heading > 0.0:
        ROBOT_HEADING = ROBOT_HEADING + delta_theta
    if robot_heading <= 0.0:
        ROBOT_HEADING = ROBOT_HEADING

    if ROBOT_Y > GOAL_Y:
        FOUND_GOAL = True
        print('FOUND GOAL')

    print('Heading Angle: %s' % heading_angle)
    print('Distance Travelled: %s' % distance_travelled)
    print('robot_heading: %s' % robot_heading)

    print('ROBOT X: %s' % ROBOT_X)
    print('ROBOT Y: %s' % ROBOT_Y)
    print('ROBOT HEADING: %s' % ROBOT_HEADING)

    x_axis.append(ROBOT_X)
    y_axis.append(ROBOT_Y)
    heading_axis.append(ROBOT_HEADING)

    time.sleep(FREQUENCY) #Loop at 5Hz

print('X AXIS: %s' % x_axis)
print('Y AXIS: %s' % y_axis)

for s,i in enumerate(x_axis):
    plt.plot(i, y_axis[s], marker=(3, 0, heading_axis[s]), markersize=10, linestyle='None')


#plt.plot([x_axis],[y_axis],'r>')
plt.axis([0, 5, 0, 5])
plt.show()

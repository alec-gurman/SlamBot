#!/usr/bin/env python

'''

SIMULATION SCRIPT WEEK 8

'''

import time
import math
import matplotlib.pyplot as plt
import random

ROBOT_X = 0.0
ROBOT_Y = 0.0
ROBOT_HEADING = 0.0

ROBOT_V = 1
ROBOT_W = 40

GOAL_X = 2.0
GOAL_Y = 3.0

FREQUENCY = 0.1

FOUND_GOAL = False

distance_travelled = ROBOT_V * FREQUENCY + 0.05*random.random()
delta_theta = ROBOT_W * FREQUENCY + 0.01*(math.pi/180)*random.random()

heading_radians = math.atan2(GOAL_Y - ROBOT_Y, GOAL_X - ROBOT_X) #get heading angle
heading_angle = math.degrees(heading_radians) #56 degrees

x_axis = []
y_axis = []
heading_axis = []

while not FOUND_GOAL:

    heading_radians = math.atan2(GOAL_Y - ROBOT_Y, GOAL_X - ROBOT_X) #get heading angle
    heading_angle = math.degrees(heading_radians) #56 degrees

    robot_headingg = heading_angle - ROBOT_HEADING

    ROBOT_X = ROBOT_X + distance_travelled * math.cos(math.radians(ROBOT_HEADING))
    ROBOT_Y = ROBOT_Y + distance_travelled * math.sin(math.radians(ROBOT_HEADING))

    if robot_headingg > 0.0:
        ROBOT_HEADING = ROBOT_HEADING + delta_theta
    if robot_headingg <= 0.0:
        ROBOT_HEADING = ROBOT_HEADING

    if ROBOT_Y > GOAL_Y:
        FOUND_GOAL = True
        print('\n\nFOUND GOAL!!!\n\n')

    print('Heading Angle: %s' % heading_angle)
    print('Distance Travelled: %s' % distance_travelled)
    print('robot_headingg: %s' % robot_headingg)

    print('ROBOT X: %s' % ROBOT_X)
    print('ROBOT Y: %s' % ROBOT_Y)
    print('ROBOT HEADING: %s' % ROBOT_HEADING)

    x_axis.append(ROBOT_X)
    y_axis.append(ROBOT_Y)
    heading_axis.append(ROBOT_HEADING)

    time.sleep(FREQUENCY) #Loop at 5Hz

    xa = [
    0.09999926282674983, 0.1999961785912288, 0.2999882830116104, 0.3999714914462935, 0.4999399103746235, 0.5998900166676975, 0.6998196202275314, 0.7997259589029909, 0.8996065726371343, 0.9994592942955598, 1.099281200823381, 1.1990693700677832, 1.2988196497094102, 1.3985289282634545, 1.498192666945258, 1.597807559635001, 1.6973686699478987, 1.7968742269440214, 1.8963190947821627, 1.995701709575242, 2.095020809304898, 2.1942754244690303, 2.29346264952152, 2.392577265031222, 2.491616179341437, 2.590573791201702, 2.6894468179163624, 2.788231979595685, 2.886925999264661, 2.9855285121336586, 3.084033437289869, 3.182437507054498, 3.280737457094402
    ]

    ya = [
    9.162977290761458e-05, 0.0002661626094974455, 0.0005105084616319391, 0.0007941240848559257, 0.0010995562290629964, 0.0014093516757554135, 0.0017453269246832158, 0.0021205718332270285, 0.002548176196862748, 0.0030325031707737525, 0.003573552599126801, 0.004166961062098675, 0.004808365164239358, 0.005493401537695158, 0.006222070052044817, 0.006990007374823087, 0.007797213387604382, 0.008648051131907164, 0.009542520445743939, 0.010484984289058339, 0.011479805552895775, 0.012531347047408341, 0.013635245378046069, 0.014791500242168971, 0.01599574831719994, 0.017243626343520814, 0.018535134083553235, 0.019874634224351564, 0.02126212645733928, 0.022706336204508112, 0.024215988658697653, 0.02579544595356302, 0.0274403445880634
    ]

for s,i in enumerate(xa):

    plt.plot(i, ya[s], marker=(3, 0, 0), markersize=10, linestyle='None')


#plt.plot([x_axis],[y_axis],'r>')
plt.axis([0, 3, -4, 4])
plt.show()

#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
from turtlebot3_gazebo.srv import *


def move_client(x, y, bug_one):
    rospy.wait_for_service('move')
    try:
        move = rospy.ServiceProxy('move', Move)
        move(x, y,bug_one)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def usage():
    return "%s [x y bug_one]" % sys.argv[0]


if __name__ == "__main__":
    if len(sys.argv) == 4:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        if sys.argv[3].lower() in ['true','false']:
            bug_one = sys.argv[3].lower() =='true'
        else:
            print(usage())
            sys.exit(1)
    else:
        print(usage())
        sys.exit(1)
    method = 'bug2'
    if bug_one:
        method = 'bug1'
    print("Requesting to movement to (%s,%s) using %s" % (x, y, method))
    resp = move_client(x, y, bug_one)

#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from controller.srv import *
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point
import math
import sys

class Dist:
    def __init__(self):
        self.left = 0
        self.front = 0
        self.raw = None

    def update(self, data):
        """
        left chosen to look slightly back to get in front of wall before turning
        :param data: laser raw data.
        :return:
        """
        def getmin(a, b):
            in_rng = lambda x: data.range_min <= x <= data.range_max
            vsp = list(filter(in_rng, data.ranges[a:b]))
            if len(vsp) > 0:
                return min(vsp)
            else:
                return sys.maxint

        '''in gazebo, laser index is 0:360'''
        newfront = getmin(160, 200)  # 80 to 100 degree
        newleft = getmin(280, 335)  # 140 to 167.5 degree!
        # newleft = getmin(300, 350)  # 150 to 175 degree!

        self.left = newleft
        self.front = newfront
        self.raw = data

    def get(self):
        l = self.left
        f = self.front
        return f, l

    def angle_to_index(self, angle):
        return int((angle - self.raw.angle_min) / self.raw.angle_increment)

    # angle in radians
    def at(self, angle):
        """for wall detection"""

        def getmin(a, b):
            in_rng = lambda x: self.raw.range_min <= x <= self.raw.range_max
            vsp = list(filter(in_rng, self.raw.ranges[a:b]))
            if len(vsp) > 0:
                return min(vsp)
            else:
                return sys.maxint
        i = self.angle_to_index(angle)
        # TODO : maybe angle needed global_to_local() in location.py
        # TODO : no this made in should_leave_wall() function
        print("angle=====>", angle, "and i=====>", i)
        start = i - 40  # For searching using for urg hokuyo
        if start < 0:
            start = 0
        if start > len(self.raw.ranges):
            start = len(self.raw.ranges) - 1
        end = i + 40   # for searching
        if end >= len(self.raw.ranges):
            end = len(self.raw.ranges) - 1
        if end < 0:
            end = 0
        print("start=====>", start, "end=====>", end)
        ans = getmin(start, end)
        return ans


class Location:
    def __init__(self):
        self.x = None
        self.y = None
        self.t = None
        self.deltaT = 0.05  # how close to angle to be to go

    def update_location(self, x, y, t):
        self.x = x
        self.y = y
        self.t = t

    def current_location(self):
        x = self.x
        y = self.y
        t = self.t
        return x, y, t

    def distance(self, x, y):
        (x0, y0, _) = self.current_location()
        if x0 is None or y0 is None:
            # will be none on the first iteration
            return sys.maxint
        return math.sqrt((x-x0)**2 + (y-y0)**2)

    def facing_point(self, x, y):
        (cx, cy, current_heading) = self.current_location()
        if None in (cx, cy, current_heading):
            print("hi agn2 false")
            return False
        n = necessary_heading(cx, cy, x, y)
        # TODO(exm) possible bug with boundary conditions?
        return n - self.deltaT <= current_heading <= n + self.deltaT  # if true--> going straight

    def faster_left(self, x, y):
        (cx, cy, current_heading) = self.current_location()
        if None in (cx, cy, current_heading):
            print("agn12 hi im here false")
            return False

        '''if true-->go to left if false-->go to right'''
        return current_heading - necessary_heading(cx, cy, x, y) < 0

    def global_to_local(self, desired_angle):
        (_, _, current_heading) = self.current_location()
        ans = desired_angle - current_heading
        if ans < -math.pi:
            ans += 2 * math.pi  # Theta must be bounded to [-pi, pi]
        elif ans > math.pi:
            ans -= 2 * math.pi
        return ans


# current x, y; target x,y
def necessary_heading(cx, cy, tx, ty):
    return math.atan2(ty-cy, tx-cx)


delta = .1
WALL_PADDING = .5

STRAIGHT = 0
LEFT = 1
RIGHT = 2
MSG_STOP = 3

RIGHTandSTRAIGHT = 4
LEFTandSTRAIGHT = 5

rate = 0.1
counter_rate = 200

current_location = Location()
current_dists = Dist()

class Controller:
    def __init__(self):
        self.sensor_data = LaserScan()
        self.goal = Point()
        self.goal.x = 2
        self.goal.y = 0

        self.flag = True
        self.thr1 = 0.2
        self.speed = Twist()


        self.x = 0
        self.y = 0
        self.theta = 0
        self.closest_point = Point()
        self.origin = Point()
        self.following_wall = False
        self.state = 'free'
        self.deltaT = 0.05
        self.closest_point = Point()
        self.closest_point.x = None
        self.closest_point.y = None

        self.origin = Point()
        self.origin.x = None
        self.origin.y = None
        self.circumnavigated = False
        self.counter = 0
        current_location.update_location(self.x, self.y, self.theta)
        self.closest_distance = current_location.distance(self.goal.x, self.goal.y)

    def get_position(self):
        return self.x,self.y,self.theta


    def update_target(self, req):
        self.goal.x = req.x
        self.goal.y = req.y
        self.bug_one = req.is_bug_1
        print(self.goal)
    def senesor_callback(self, msg):
        self.sensor_data = msg
        current_dists.update(msg)

        #print(self.print_sensor_data())
    def distance(self, x, y):
        x0, y0, _ = self.get_position()
        if x0 is None or y0 is None:
            return None
        return math.sqrt((x - x0) ** 2 + (y - y0) ** 2)

    def is_facing_target(self, x, y):
        x0, y0, current_heading = self.get_position()
        if None in (x0, y0, current_heading):
            return False
        #print(f'{x0},{y0},{x},{y}')
        goal_theta = math.atan2(y - y0, x - x0)
        #(f'goal_theta:{goal_theta}')
        return goal_theta - self.deltaT <= current_heading <= goal_theta + self.deltaT

    def is_left(self, x, y):
        x0, y0, current_heading = self.get_position()
        if None in (x0, y0, current_heading):
            return False
        return current_heading - math.atan2(y - y0, x - x0) < 0

    def print_sensor_data(self):
        valid_list = [0,15,45,90,135,180,225,270,315,345,359]
        for i in valid_list:
            print(f'{i}:{self.sensor_data.ranges[i]}')
        '''
        print(f'x:{self.x}')
        print(f'y:{self.y}')
        print(f'x_goal:{self.goal.x}')
        print(f'y_goal:{self.goal.y}')
        print(f'theta:{self.theta}')
        print(f'theta:{math.atan2( self.goal.y - self.y, self.goal.x- self.x)}')
        '''
    def update_position(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rotation_q = msg.pose.pose.orientation
        roll, pitch, theta = euler_from_quaternion([rotation_q.x, rotation_q.y, rotation_q.z, rotation_q.w])
        self.theta = theta

        current_location.update_location(self.x, self.y, self.theta)

    def face_goal(self):
        """such as bug2"""
        while not current_location.facing_point(self.goal.x, self.goal.y):
            print('in face goal2 and turn right')
            self.go(RIGHT)
            rospy.sleep(.01)

    def go(self, direction):
        cmd = Twist()
        if direction == STRAIGHT:
            cmd.linear.x = 0.3
            cmd.angular.z = 0
        elif direction == LEFT:
            cmd.angular.z = 0.1
        elif direction == RIGHT:
            cmd.angular.z = -0.1
            cmd.linear.x = 0
        elif direction == MSG_STOP:
            cmd.angular.z = 0
            cmd.linear.x = 0
        elif direction == RIGHTandSTRAIGHT:
            print('RIGHTandSTRAIGHT')
            cmd.angular.z = -0.25
            cmd.linear.x = 0.05
        elif direction == LEFTandSTRAIGHT:
            print('RIGHTandSTRAIGHT')
            cmd.angular.z = +0.25
            cmd.linear.x = 0.05
        pub.publish(cmd)

    def go_until_obstacle(self):
        print("Going until destination or obstacle")
        while current_location.distance(self.goal.x, self.goal.y) > delta:
            (frontdist, _) = current_dists.get()
            if frontdist <= WALL_PADDING:
                print("agn8 stop")
                self.go(MSG_STOP)
                return True

            if current_location.facing_point(self.goal.x, self.goal.y):
                print("agn9 straight")
                self.go(STRAIGHT)
            elif current_location.faster_left(self.goal.x, self.goal.y):
                print("agn10 left")
                self.go(LEFT)
            else:
                print("agn11 right")
                self.go(RIGHT)
            rospy.sleep(.01)
        return False

    def follow_wall(self):
        print("Wall Following")
        while current_dists.get()[0] <= WALL_PADDING:
            '''zero index is front value that returned in get() function'''
            print('right in zero index')
            self.go(RIGHT)
            rospy.sleep(.01)
        while not self.should_leave_wall():
            '''Wall following started.'''
            (front, left) = current_dists.get()
            if front <= WALL_PADDING:
                print('right')
                self.go(RIGHT)
            elif WALL_PADDING - .1 <= left <= WALL_PADDING + .1:
                print('straight')
                self.go(STRAIGHT)
            elif WALL_PADDING + .1 < left < WALL_PADDING + 1:
                print('lef and straight')
                self.go(LEFTandSTRAIGHT)
            elif left > WALL_PADDING + .1:
                print('left')
                self.go(LEFT)
            else:
                '''This means left < wall_padding'''
                print('right with else')
                # self.go(RIGHT)
                self.go(RIGHTandSTRAIGHT)
            rospy.sleep(rate)

        self.face_goal()

    def should_leave_wall(self):
        (x, y, t) = current_location.current_location()

        if self.closest_point.x is None or self.closest_point.y is None:
            self.origin.x = x
            self.origin.y = y
            self.closest_point.x = x
            self.closest_point.y = y
            self.closest_distance = current_location.distance(self.goal.x, self.goal.y)
            '''get once distance for each obstacle'''

            self.left_origin_point = False
            return False
        d = current_location.distance(self.goal.x, self.goal.y)
        if d < self.closest_distance:
            print ("New closest point at", (x, y))
            self.closest_distance = d
            '''closest distance updated'''

            self.closest_point.x  = x
            self.closest_point.y =  y

        if not self.left_origin_point and not near(x, y, self.origin.x, self.origin.y):
            print("Left original touch point")
            self.left_origin_point = True
        elif near(x, y, self.origin.x, self.origin.y) and self.left_origin_point:
            '''circumnavigation achieved!'''
            print("Circumnavigated obstacle")
            self.circumnavigated = True

        if self.circumnavigated and near(x, y, self.closest_point.x, self.closest_point.y):
            '''achieve nearest point after a circumnavigated obstacle'''

            self.closest_point.x = None
            self.closest_point.y = None
            '''Init for next obstacle.'''

            self.origin.x = None
            self.origin.y = None
            self.circumnavigated = False
            self.left_origin_point = False
            print("Leaving wall")
            return True

        else:
            return False

def near(cx, cy, x, y):
    nearx = x - .3 <= cx <= x + .3
    neary = y - .3 <= cy <= y + .3
    return nearx and neary


if __name__ == '__main__':

    controller = Controller()
    rospy.init_node('move_server')
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    sub = rospy.Subscriber("/scan", LaserScan, controller.senesor_callback)
    odom_sub = rospy.Subscriber('/odom', Odometry, controller.update_position)
    server = rospy.Service('move', Control, controller.update_target)
    r = rospy.Rate(10)
    print('control server is ready')

    print("Calibrating sensors...")
    '''This actually just lets the sensor readings propagate into the system'''
    rospy.sleep(1)
    print("Calibrated")

    while current_location.distance(controller.goal.x, controller.goal.y) > delta:
        hit_wall = controller.go_until_obstacle()
        if hit_wall and controller.flag:
            '''arrive to the wall or obstacle'''
            print('near wall following', controller.flag)
            controller.follow_wall()
    print("Arrived at", (controller.goal.x, controller.goal.y))

    cmd = Twist()
    cmd.linear.x = 0
    cmd.angular.z = 0
    pub.publish(cmd)


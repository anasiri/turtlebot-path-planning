#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from controller.srv import *
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point
import math
import logging
delta = .1


STRAIGHT = 0
LEFT = 1
RIGHT = 2
MSG_STOP = 3

rate = 0.1



class Controller:
    def __init__(self):
        self.sensor_data = LaserScan()
        self.goal = Point()
        self.goal.x = None
        self.goal.y = None

        self.thr1 = 0.2
        self.speed = Twist()

        self.x = 0
        self.y = 0
        self.theta = 0
        self.closest_point = Point()
        self.origin = Point()
        self.second_round = False
        self.left_origin = False
        self.state = 'free'
        self.deltaT = 0.2
        self.algorithm = self.leave_check_bug1
        self.encountered_wall_at = (None,None)

        self.lh = None

    def get_position(self):
        return self.x, self.y, self.theta

    def update_position(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rotation_q = msg.pose.pose.orientation
        roll, pitch, theta = euler_from_quaternion([rotation_q.x, rotation_q.y, rotation_q.z, rotation_q.w])
        self.theta = theta

    def update_target(self, req):
        self.goal.x = req.x
        self.goal.y = req.y
        if req.is_bug_1:
            self.algorithm = self.leave_check_bug1
        else:
            self.algorithm = self.leave_check_bug2
        logging.info(self.goal)
        resp = ControlResponse()
        return resp

    def senesor_callback(self, msg):
        self.sensor_data = msg

    def distance(self, x, y):
        x0, y0, _ = self.get_position()
        if x0 is None or y0 is None:
            return None
        return math.sqrt((x - x0) ** 2 + (y - y0) ** 2)

    def is_facing_target(self, x, y):
        x0, y0, current_heading = self.get_position()
        if None in (x0, y0, current_heading):
            return False
        goal_theta = math.atan2(y - y0, x - x0)
        sign = 1
        if current_heading>0:
            sign = -1
        if sign*math.pi - 0.2 < goal_theta < sign*math.pi + 0.2:
            goal_theta = -goal_theta

        return goal_theta - self.deltaT <= current_heading <= goal_theta + self.deltaT

    def is_left(self, x, y):
        x0, y0, current_heading = self.get_position()
        if None in (x0, y0, current_heading):
            return False
        goal_theta = math.atan2(y - y0, x - x0)
        if math.pi - 0.2 < goal_theta < math.pi + 0.2:
            goal_theta = -goal_theta
        return current_heading - goal_theta < 0

    def print_sensor_data(self):
        message = f'x:{self.x} \n' \
                  f'y:{self.y} \n' \
                  f'x_goal:{self.goal.x} \n' \
                  f'y_goal:{self.goal.y} \n' \
                  f'theta:{self.theta} \n' \
                  f'theta_goal:{math.atan2(self.goal.y - self.y, self.goal.x - self.x)} \n' \
                  f"front:{self.get_data(sensors['front'])} \n" \
                  f"left:{self.get_data(sensors['left'])} \n" \
                  f'state:{self.state}'
        logging.info(message)


    def go(self, direction):
        cmd = Twist()
        if direction == STRAIGHT:
            cmd.linear.x = 0.15
            cmd.angular.z = 0.0
            logging.info('straight')

        elif direction == LEFT:
            cmd.linear.x = 0
            cmd.angular.z = 0.2
            logging.info('left')
        elif direction == RIGHT:
            cmd.linear.x = 0
            cmd.angular.z = -0.2
            logging.info('right')
        elif direction == MSG_STOP:
            cmd.linear.x = 0
            cmd.angular.z = 0
            logging.info('stop')
        pub.publish(cmd)

    def get_data(self, data):
        a, b = data[0],data[1]
        if a > b:
            curr_min = min(self.sensor_data.ranges[a:])
            return min(curr_min, min(self.sensor_data.ranges[:b]))
        else:
            return min(self.sensor_data.ranges[a:b])

    def go_until_obstacle(self):
        # print("Going until destination or obstacle")

        self.print_sensor_data()
        if self.is_facing_target(self.goal.x, self.goal.y):
            # self.print_sensor_data()
            if self.get_data(sensors['front']) < self.thr1:
                self.state = 'follow'
                self.origin.x = self.x
                self.origin.y = self.y
                self.closest_point.x = self.x
                self.closest_point.y = self.y
                self.closest_point_dst = self.distance(self.goal.x, self.goal.y)
                self.go(MSG_STOP)
            else:
                self.go(STRAIGHT)
        elif self.is_left(self.goal.x, self.goal.y):
            self.go(LEFT)
        else:
            self.go(RIGHT)
        r.sleep()



    def follow_wall(self):
        logging.info("Following wall")
        self.print_sensor_data()
        if self.algorithm():
            self.state = 'free'
            logging.info(f'orig:{self.origin}')
            logging.info(f'close:{self.closest_point}')
            return

        if self.get_data(sensors['front']) < self.thr1+0.1:
            logging.info('Detected wall ahead.')
            self.go(RIGHT)
        elif self.get_data(sensors['fleft']) <= self.thr1 :
            logging.info('Wall is on the left side')
            self.go(STRAIGHT)
        #elif self.get_data(60, 120) > self.thr1 + 0.1:
        #    logging.info(3)
        #    self.go(MSG_STOP)
        else:
            logging.info('Turning left')
            self.go(LEFT)
        r.sleep()

    def leave_check_bug1(self):

        # if this point is closer
        if self.distance(self.goal.x, self.goal.y) < self.closest_point_dst:
            self.closest_point.x = self.x
            self.closest_point.y = self.y
            self.closest_point_dst = self.distance(self.goal.x,self.goal.y)
        # if returned to original
        if self.distance(self.origin.x, self.origin.y) < delta:
            if self.left_origin:
                logging.info('returned to origin point')
                self.second_round = True
        else:
            self.left_origin=True

        # if returned to closest point
        if self.second_round and self.distance(self.closest_point.x, self.closest_point.y) < delta:
            self.second_round = False
            self.left_origin = False
            return True
        else:
            return False

    def leave_check_bug2(self):

        (x, y, _) = self.get_position()
        if None in self.encountered_wall_at:
            self.encountered_wall_at = (x, y)
            self.lh = math.atan2(y - self.goal.y, x - self.goal.x)
            return False
        t_angle = math.atan2(y - self.goal.y, x - self.goal.x)
        (ox, oy) = self.encountered_wall_at
        od = math.sqrt((ox - self.goal.x) ** 2 + (oy - self.goal.y) ** 2)
        cd = math.sqrt((x - self.goal.x) ** 2 + (y - self.goal.y) ** 2)
        dt = 0.01

        if self.lh - dt <= t_angle <= self.lh + dt and not near(x, y, ox, oy):
            if cd < od:
                logging.info("Leaving wall")
                self.encountered_wall_at = (None,None )
                return True
        return False


def near(cx, cy, x, y):
    nearx = x - .3 <= cx <= x + .3
    neary = y - .3 <= cy <= y + .3
    return nearx and neary
sensors = {'front':[337,22],'fleft':[22,67],'left':[67,112],
               'bleft':[112,157],'back':[157,202],'bright':[202,247],
               'right':[247,292],'fright':[292,337]
}

def print_sensing():
    message = ''
    for key, item in sensors.items():
        message += f'{key}:{controller.get_data(item)}\n'
    print(message)

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    controller = Controller()
    rospy.init_node('move_server')
    r = rospy.Rate(4)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    sub = rospy.Subscriber("/scan", LaserScan, controller.senesor_callback)
    odom_sub = rospy.Subscriber('/odom', Odometry, controller.update_position)
    server = rospy.Service('move', Control, controller.update_target)

    logging.info('control server is ready')

    while len(controller.sensor_data.ranges) == 0:
        continue
    while True:

        while controller.goal.x is None or controller.goal.y is None:
            pass

        while controller.distance(controller.goal.x, controller.goal.y) > delta:
            if controller.state == 'free':
                controller.go_until_obstacle()
            elif controller.state == 'follow':
                controller.follow_wall()

        cmd = Twist()
        cmd.linear.x = 0
        cmd.angular.z = 0
        pub.publish(cmd)
        logging.info("Arrived at {controller.goal.x}, {controller.goal.y}")
        while controller.distance(controller.goal.x, controller.goal.y) < delta:
            continue

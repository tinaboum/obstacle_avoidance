#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

#The robot's position is obtained using the Odometry message. 
#The position and orientation are extracted to track where the robot is.

pub = None

#x0, y0 = 1.0, 0.0  # Initial position (can be set dynamically)
xg, yg = 5.0, 0.0  # Goal position (adjust based on your map/area)
position = None
yaw = 0.0
goal_reached = False

def callback_laser(msg):
    global goal_reached

    # If the goal is reached, stop the robot
    if goal_reached:
        stop_robot()
        return

    # 120 degrees divided into 3 regions
    regions = {
        'right':  min(min(msg.ranges[0:2]), 10),
        'front':  min(min(msg.ranges[3:5]), 10),
        'left':   min(min(msg.ranges[6:9]), 10),
    }

    take_action(regions)

def callback_odometry(msg):
    global position, yaw
    position = msg.pose.pose.position
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

def take_action(regions):
    global goal_reached

    threshold_dist = 1.5
    linear_speed = 0.6
    angular_speed = 1

    msg = Twist()
    linear_x = 0
    angular_z = 0

    # Calculate distance to goal
    distance_to_goal = get_distance_to_goal()

    if distance_to_goal < 0.2:  # Distance threshold to stop the robot
        rospy.loginfo("Goal reached!")
        goal_reached = True
        stop_robot()
        return

    state_description = ''
    if regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
        state_description = 'case 1 - no obstacle'
        linear_x = linear_speed
        angular_z = 0
    elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
        state_description = 'case 7 - front and left and right'
        linear_x = -linear_speed
        angular_z = angular_speed
    elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] > threshold_dist:
        state_description = 'case 2 - front'
        linear_x = 0
        angular_z = angular_speed
    elif regions['front'] > threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
        state_description = 'case 3 - right'
        linear_x = 0
        angular_z = -angular_speed
    elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
        state_description = 'case 4 - left'
        linear_x = 0
        angular_z = angular_speed
    elif regions['front'] < threshold_dist and regions['left'] > threshold_dist and regions['right'] < threshold_dist:
        state_description = 'case 5 - front and right'
        linear_x = 0
        angular_z = -angular_speed
    elif regions['front'] < threshold_dist and regions['left'] < threshold_dist and regions['right'] > threshold_dist:
        state_description = 'case 6 - front and left'
        linear_x = 0
        angular_z = angular_speed
    elif regions['front'] > threshold_dist and regions['left'] < threshold_dist and regions['right'] < threshold_dist:
        state_description = 'case 8 - left and right'
        linear_x = linear_speed
        angular_z = 0
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)

def stop_robot():
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0
    pub.publish(msg)

def get_distance_to_goal():
    global position, xg, yg
    if position is None:
        return float('inf')
    return math.sqrt((xg - position.x) ** 2 + (yg - position.y) ** 2)

def main():
    global pub
    
    rospy.init_node('robot_navigation')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_laser = rospy.Subscriber('/robot/laser/scan', LaserScan, callback_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, callback_odometry)

    rospy.spin()

if __name__ == '__main__':
    main()

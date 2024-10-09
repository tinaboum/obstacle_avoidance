#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math


pub = None
xg, yg = 5.0, 5.0  # Fixed goal position
position = None
yaw = 0.0
goal_reached = False
initial_position_set = False
x0, y0 = None, None  # Variables to store dynamic initial position
collision_threshold = 1.5  # Distance threshold to consider a sub-region occupied (meters)
steps = 10  # Number of steps to repeat (you can adjust this value)


#Callback for LIDAR Data
def callback_laser(msg):
    global goal_reached

    # If the goal is reached, stop the robot
    if goal_reached:
        stop_robot()
        return

    # Divide 120 degrees into 3 regions: front, left, right
    regions = {
        'right':  msg.ranges[0:2],   # Example: first 2 laser scan indices
        'front':  msg.ranges[3:5],   # Example: middle 3 laser scan indices
        'left':   msg.ranges[6:9],   # Example: last 3 laser scan indices
    }
    # Filter free (collision-free) sub-regions in each region
    free_regions = filter_free_sub_regions(regions)
    
    take_action(free_regions)


#Filter Function for Collision-Free Sub-Regions
def filter_free_sub_regions(regions):
    global collision_threshold
    free_regions = {}

    # Iterate through the regions (front, left, right)
    for region_name, distances in regions.items():
        free_sub_regions = []
        # Check each distance in the sub-region to see if it's collision-free
        for i, distance in enumerate(distances):
            if distance > collision_threshold:
                free_sub_regions.append(i)  # Store the sub-region index that is free
                        # If there are free sub-regions, store them in free_regions
        if free_sub_regions:
            free_regions[region_name] = free_sub_regions

    return free_regions
#The function returns a dictionary of free sub-regions for each region.


#Callback for Odometry Data
def callback_odometry(msg):
    global position, yaw, x0, y0, initial_position_set

    # Get the robot's current position
    position = msg.pose.pose.position
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)


#Distance to goal
def get_distance_to_goal():
    global position, xg, yg
    if position is None:
        return float('inf')
    return math.sqrt((xg - position.x) ** 2 + (yg - position.y) ** 2)


    

#estimate next position
def estimate_next_position(region, sub_index):
    linear_displacement = 0.1  # Small step forward
    if region == 'front':
        return (position.x + linear_displacement, position.y)
    elif region == 'left':
        return (position.x, position.y + linear_displacement)
    elif region == 'right':
        return (position.x, position.y - linear_displacement)
    

#euclidean distance
def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


#Take Action Function
def take_action(free_regions):
    global goal_reached

    linear_speed = 0.6
    angular_speed = 1
    msg = Twist()

    # Calculate distance to goal
    distance_to_goal = get_distance_to_goal()
    if distance_to_goal < 0.2:  # Distance threshold to stop the robot
        rospy.loginfo("Goal reached!")
        goal_reached = True
        stop_robot()
        return
    
    best_region = None
    shortest_distance_to_goal = float('inf')

    # Loop through all the free sub-regions and select the one that gives the shortest distance to the goal
    for region, sub_indices in free_regions.items():
        for sub_index in sub_indices:
            # Estimate the robot's next position in this sub-region and calculate distance to goal
            next_position = estimate_next_position(region, sub_index)
            distance_to_goal = euclidean_distance(next_position[0], next_position[1], xg, yg)

            if distance_to_goal < shortest_distance_to_goal:
                shortest_distance_to_goal = distance_to_goal
                best_region = region

    # Take action based on the best sub-region (the one that gets the closest to the goal)
    if best_region == 'front':
        rospy.loginfo("Moving forward")
        msg.linear.x = linear_speed
        msg.angular.z = 0
    elif best_region == 'left':
        rospy.loginfo("Turning left")
        msg.angular.z = angular_speed
    elif best_region == 'right':
        rospy.loginfo("Turning right")
        msg.angular.z = -angular_speed
    else:
        rospy.loginfo("No valid path, backward + right")
        msg.linear.x = -linear_speed
        msg.angular.z = -angular_speed

        return

    pub.publish(msg)



def stop_robot():
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0
    pub.publish(msg)


#Main Function
def main():
    global pub

    rospy.init_node('robot_navigation')

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    sub_laser = rospy.Subscriber('/robot/laser/scan', LaserScan, callback_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, callback_odometry)

    rospy.spin()

if __name__ == '__main__':
    main()



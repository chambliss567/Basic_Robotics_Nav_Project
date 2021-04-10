#! /usr/bin/env python

import rospy                               # Import the Python library for ROS
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from ros_basics_project.srv import FindWall, FindWallRequest
from ros_basics_project.msg import OdomRecordAction, OdomRecordGoal, OdomRecordResult, OdomRecordFeedback
from std_msgs.msg import Empty
import actionlib
import time

PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

def readLaser(msg):
    twist_var = Twist()
    twist_var.linear.x = 0.09
    # Grab laser value directly to the right side of robot
    right_laser = msg.ranges[89]
    # Grab laser values directly in front of robot
    middle_laser = msg.ranges[159:201]
    # Initially check if any front facing lasers are within 0.5 meters.
    # If so, sharp left turn
    if any(laser < 0.5 for laser in middle_laser):
        twist_var.angular.z = 0.48
    # If nothing in front of robot and robot is more than 0.3 meters away from wall,
    # Gradual right turn
    elif right_laser > 0.3 :
        twist_var.angular.z = -0.2
    # If nothing in front of robot and robot is within 0.2 meters away from wall,
    # Gradual left turn
    elif right_laser < 0.2:
        twist_var.angular.z = 0.2
    # If nothing in front of robot and robot is close to wall, keep going forward
    else:
        twist_var.angular.z = 0
    # Publish twist message to robot
    pub.publish(twist_var)

def feedback_callback(feedback):
    # Log distance traveled
    rospy.loginfo("Total distance traveled: %s meters" % feedback.current_total)

# Connect to service
rospy.init_node('read_lasers',disable_signals = True)
rospy.loginfo("Calling /find_wall service")
rospy.wait_for_service('/find_wall')
# Create the connection to the service
find_wall_service = rospy.ServiceProxy('/find_wall', FindWall)
find_wall_request_object = FindWallRequest()
find_wall_request_object.request = Empty()
# Call the service
result = find_wall_service(find_wall_request_object)
print(result)
# Call the odometry recording action server
client = actionlib.SimpleActionClient('/record_odom', OdomRecordAction)
client.wait_for_server()
action_goal = OdomRecordGoal()
action_goal.goal = Empty()
client.send_goal(action_goal, feedback_cb=feedback_callback)
# Wait until action server has started
time.sleep(3)
# Start subscriber node
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)    
sub = rospy.Subscriber('/scan', LaserScan, readLaser)
# Run until robot has done full lap
state_result = client.get_state()
r = rospy.Rate(1)
rospy.loginfo("Moving around track while hugging right wall")
while state_result < DONE:
    state_result = client.get_state()
    r.sleep()
# Stop robot depending upon return of odom record action
twist_var = Twist()
twist_var.angular.z = 0
twist_var.linear.x = 0
sub.unregister()
pub.publish(twist_var)
if state_result == ERROR or state_result == WARN:
    rospy.logerr("Something went wrong - stopping robot")
else:
    rospy.loginfo("Robot succesfull completed lap - stopping robot")

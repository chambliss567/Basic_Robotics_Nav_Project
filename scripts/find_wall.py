#! /usr/bin/env python

import rospy
from ros_basics_project.srv import FindWall
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty
import numpy as np
import math
import time

class FindWallClass(object):

    def __init__(self):
        # Initialize all needed variables
        self._ss = rospy.Service('/find_wall', FindWall, self.find_wall)
        self.pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.readLaser)
        self.check_laser_init_pos = False
        self.front_laser = 0
        self.min_index = 0
        self.twist_msg = Twist()
        self.twist_msg.linear.x = 0
        self.twist_msg.linear.y = 0
        self.twist_msg.linear.z = 0
        self.twist_msg.angular.x = 0
        self.twist_msg.angular.y = 0
        self.twist_msg.angular.z = 0

    def find_wall(self, request):
        # Wait until publisher and subscriber are connected to respective topics
        rospy.loginfo("Waiting until subscriber and publisher are connected to their topics")
        while (self.sub_laser.get_num_connections() < 1 and self.pub.get_num_connections() < 1):
            time.sleep(1)
        time.sleep(1)
        rospy.loginfo("Subscriber and publisher are connected to their topics")
        #Only enter if robot has not turned to face wall
        if self.check_laser_init_pos == False:
            # Grab laser ray correspoding to closest distance to wall
            self.check_laser_init_pos = True
            # Determine how long robot should turn and whether it should turn right/left
            if self.min_index < 179:
                self.twist_msg.angular.z = -(math.pi / 16)
                time_turn = ((179 - self.min_index) * (math.pi / 180)) / (math.pi / 16)
                angle = -((179 - self.min_index) * (math.pi / 180)) * 57.2957
            elif self.min_index > 179:
                self.twist_msg.angular.z = (math.pi / 16)
                time_turn = ((self.min_index - 179) * (math.pi / 180)) / (math.pi / 16)
                angle = ((self.min_index - 179) * (math.pi / 180)) * 57.2957
            else:
                time_turn = 0
        # Turn robot to face wall
        rospy.loginfo("Turning robot to face nearest wall: Nearest wall at angle %d degrees from front of robot" % angle)
        self.pub.publish(self.twist_msg)
        time.sleep(time_turn)
        # Stop turning robot and move robot forward to wall
        rospy.loginfo("Stopping robot turn. Moving robot forward to wall")
        r = rospy.Rate(10)
        self.twist_msg.angular.z = 0
        self.twist_msg.linear.x = 0.1
        self.pub.publish(self.twist_msg)
        while self.front_laser > 0.3:
            r.sleep()
        # Stop moving robot forward and turn robot left to be move parallel to wall
        rospy.loginfo("Stopping robot moving forward. Turn robot left to be parallel wall")
        self.twist_msg.linear.x = 0
        self.twist_msg.angular.z = math.pi / 16
        self.pub.publish(self.twist_msg)
        time.sleep(8)
        #Stop turning robot
        self.twist_msg.angular.z = 0
        self.pub.publish(self.twist_msg)
        # Reset service and return service successfull
        self.check_laser_init_pos = False
        return True

    def readLaser(self, laserscan):
        # Find shortest laser ray if robot has not turned to face wall yet
        if self.check_laser_init_pos == False:
            # Look at sets of 90 rays at a time so obstacles are not mistaken for walls
            ray_chunks = [laserscan.ranges[x:x+90] for x in range(0,len(laserscan.ranges),90)]
            std_dev_chunks = np.array([np.std(ray) for ray in ray_chunks])
            mean_chunks = np.array([np.mean(ray) for ray in ray_chunks])
            std_check = std_dev_chunks / mean_chunks
            final_rays = np.array(laserscan.ranges)
            iterator = 0
            for ele in std_check:
                if ele > 0.3:
                    final_rays[(iterator * 90):((iterator + 1) * 90 - 1)] = 1000
                iterator = iterator + 1
            self.min_index = np.argmin(final_rays)
        # Always grab front laser ray value
        self.front_laser = laserscan.ranges[179]
if __name__ == '__main__':
    # Start node
    rospy.init_node('find_wall_node') 
    FindWallClass()
    rospy.spin() # maintain the service open.
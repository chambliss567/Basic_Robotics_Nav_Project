#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from ros_basics_project.msg import OdomRecordAction, OdomRecordResult, OdomRecordFeedback
from geometry_msgs.msg import Point
import time
import math
import copy
import actionlib


class RecordOdomClass(object):
    
  # create messages that are used to publish feedback/result
  _feedback = OdomRecordFeedback()
  _result   = OdomRecordResult()

  def __init__(self):
    # creates the action server and other variables
    self.sub_odom = rospy.Subscriber('/odom', Odometry, self.read_odom)
    self._as = actionlib.SimpleActionServer("/record_odom", OdomRecordAction, self.goal_callback, False)
    self._as.start()
    self.init_pos = Point()
    self.curr_pos = Point()

  def read_odom(self, odom):
    # Always grab current_position and allocate last position
    self.curr_pos.x = odom.pose.pose.position.x
    self.curr_pos.y = odom.pose.pose.position.y
    self.curr_pos.z = odom.pose.pose.orientation.w

  def goal_callback(self, goal):
    # Wait until subscriber is connected to odom topic
    rospy.loginfo("Waiting until subscriber is connected to odom topic")
    while (self.sub_odom.get_num_connections() < 1):
            time.sleep(1)
    time.sleep(1)
    # Create loop that will go until program execution is stopped
    self._result.list_of_odoms.append(copy.copy(self.curr_pos))
    self._feedback.current_total = 0
    # Grab initial position if initial position has not been checked yet
    self.init_pos = copy.copy(self.curr_pos)
    r = rospy.Rate(1)
    iterator = 0
    while not rospy.is_shutdown():
        #Calculate distance traveled in last second
        self._feedback.current_total = self._feedback.current_total + math.sqrt((self.curr_pos.x - self._result.list_of_odoms[iterator].x)**2 + (self.curr_pos.y - self._result.list_of_odoms[iterator].y)**2)
        self._as.publish_feedback(self._feedback)
        if iterator > 5:
            # Check if robot has gone one lap
            if (math.sqrt((self.curr_pos.x - self.init_pos.x)**2 + (self.curr_pos.y - self.init_pos.y)**2) < 0.15):
                # If robot is within 0.15 meters of initial position, publish all odomoetries recorded
                self._as.set_succeeded(self._result)
        # Add current position to result
        self._result.list_of_odoms.append(copy.copy(self.curr_pos))
        iterator = iterator + 1
        r.sleep()
      
if __name__ == '__main__':
  rospy.init_node('record_odom')
  RecordOdomClass()
  rospy.spin()

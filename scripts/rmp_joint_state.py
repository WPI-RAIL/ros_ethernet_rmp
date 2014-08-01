#!/usr/bin/env python

"""
Battery monitor for the Segway RMP platform.

Author:  Chris Dunkers, Worcester Polytechnic Institute
Version: June 23, 2014
"""
from ros_ethernet_rmp.msg import RMPFeedback
from battery_monitor_rmp.msg import RMPBatteryStatus 
from python_ethernet_rmp.system_defines import *
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import rospy
import math
import time
import os

class JointStateMonitor:

	def __init__(self):
		"""
		Initialize the subscriptions and publishers of the node.
		"""
		self.jointStatePub = rospy.Publisher('rmp_joint_states', JointState, queue_size = 'None')

		rospy.Subscriber("rmp_feedback", RMPFeedback, self.get_batt_state)
		
		"""
		Get the battery parameters
		"""
		self.has_two_wheels = rospy.get_param('~has_two_wheels',True)
		self.link_left_front = rospy.get_param('~link_left_front','base_link_left_wheel_joint')
		self.link_right_front = rospy.get_param('~link_right_front','base_link_right_wheel_joint')
		self.link_left_rear = rospy.get_param('~link_left_rear','base_link_left_rear_wheel_joint')
		self.link_right_rear = rospy.get_param('~link_right_rear','base_link_right_rear_wheel_joint')
		tire_diameter = rospy.get_param('/ethernet_rmp/my_tire_diameter',DEFAULT_TIRE_DIAMETER_M)
		self.circumference = math.pi*tire_diameter;
		
	def get_batt_state(self, rmp):
		"""
		Read in the current RMP feedback and publish the pose
		:param rmp: the RMP feedback message
		"""
		rmp_items = rmp.sensor_items
		rmp_values = rmp.sensor_values
		
		joint_state = JointState()
		
		names = [self.link_left_front, self.link_right_front,
					self.link_left_rear, self.link_right_rear]
		pos = [0,0,0,0]
		vel = [0,0,0,0]
		
		"""
		get the values for the feedback items needed
		"""
		for x in range(0, len(rmp_items)):
			if rmp_items[x] == 'left_front_pos_m':
				pos[0] =  ((rmp_values[x]/self.circumference) % 1.0)*(2*math.pi)
			elif rmp_items[x] == 'right_front_pos_m':
				pos[1] = ((rmp_values[x]/self.circumference) % 1.0)*(2*math.pi)
			elif rmp_items[x] == 'left_rear_pos_m':
				pos[2] = ((rmp_values[x]/self.circumference) % 1.0)*(2*math.pi)
			elif rmp_items[x] == 'right_rear_pos_m':
				pos[3] = ((rmp_values[x]/self.circumference) % 1.0)*(2*math.pi)
			elif rmp_items[x] == 'left_front_vel_mps':
				vel[0] = rmp_values[x]
			elif rmp_items[x] == 'right_front_vel_mps':
				vel[1] = rmp_values[x]
			elif rmp_items[x] == 'left_rear_vel_mps':
				vel[2] = rmp_values[x]
			elif rmp_items[x] == 'right_rear_vel_mps':
				vel[3] = rmp_values[x]
				
		if self.has_two_wheels:
			num = 2
		else:
			num = 4
			
		for x in range(0,num):
			joint_state.name.append(names[x])
			joint_state.position.append(pos[x])
			joint_state.velocity.append(vel[x])
			
		"""
		Publish the state of the wheels/joints
		"""	
		joint_state.header.stamp = rospy.Time.now()
		self.jointStatePub.publish(joint_state)

if __name__ == "__main__":
	rospy.init_node("rmp_joint_state")
	jointState = JointStateMonitor()
	rospy.loginfo("RMP Joint State Started")
	rospy.spin()

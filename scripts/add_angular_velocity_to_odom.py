#!/usr/bin/env python

from rosbag import Bag
from nav_msgs.msg import Odometry
import argparse
import time
from rospy import Time as rosTime
import numpy as np
from tf.transformations import *
import copy
import sys
import math


input_topic_name = "/imu_odom"
output_topic_name = "/recomputed_odom"


if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Add missing angular rates to odom messages in a new topic')
	parser.add_argument('--input_topic', type=str, default="", help='Name of the input odom topic')
	parser.add_argument('--output_topic', type=str, default="", help='Name of the output odom topic')
	parser.add_argument('bags', metavar='N', type=str, nargs='+',
	                    help='List of bags to process')
	
	args = parser.parse_args()

	if len(args.input_topic) != 0:
		input_topic_name = args.input_topic

	if len(args.output_topic) != 0:
		output_topic_name = args.output_topic


	for bag in args.bags:
		print("Processing", bag)
		dst = bag.replace('.bag', '.processed.bag')
		previous_att_quat = None
		previous_stamp = None

		with Bag(dst, 'w') as outbag:
			for topic, msg, t, conn_header in Bag(bag).read_messages(return_connection_header=True):
				if topic == input_topic_name:
					stamp = msg.header.stamp
					if previous_att_quat is not None:
						current_att_quat = np.array([msg.pose.pose.orientation.x,
													 msg.pose.pose.orientation.y,
													 msg.pose.pose.orientation.z,
													 msg.pose.pose.orientation.w])

						output_msg = copy.deepcopy(msg)

						delta_quat = quaternion_multiply(current_att_quat, quaternion_inverse(previous_att_quat))
						quat_vec = np.array(delta_quat[0:3])  #Not normalized yet!
						quat_vec_norm = np.linalg.norm(quat_vec)
						rot_vec_magnitude = 2 * math.acos(delta_quat[3])

						if not quat_vec_norm < 2* sys.float_info.min:
							quat_vec *= (1.0/quat_vec_norm)
						else:
							quat_vec = np.array([0.0, 0.0, 0.0])
							rot_vec_magnitude = 0.0

						rot_vec = quat_vec * rot_vec_magnitude * (1.0/((stamp-previous_stamp).to_sec()))

						output_msg.twist.twist.angular.x = rot_vec[0]
						output_msg.twist.twist.angular.y = rot_vec[1]
						output_msg.twist.twist.angular.z = rot_vec[2]

						outbag.write(output_topic_name, output_msg, t)


					previous_att_quat = np.array([msg.pose.pose.orientation.x,
												  msg.pose.pose.orientation.y,
												  msg.pose.pose.orientation.z,
												  msg.pose.pose.orientation.w])
					previous_stamp = stamp
					outbag.write(topic, msg, t, connection_header=conn_header)
				else:
					outbag.write(topic, msg, t, connection_header=conn_header)
		print "Saved as", dst

#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

import numpy as np


def prior_odom_publisher():
    pub = rospy.Publisher('fake_odom', Odometry, queue_size=10)
    rospy.init_node('fake_odom_publisher', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    odom_data = np.load('interp_position_spot_1.npy')
    odom_data_pointer = 0


    while not rospy.is_shutdown():

        current_time = rospy.Time.now().to_sec()

        if odom_data[odom_data_pointer,0] <= current_time:
            new_message = Odometry()
            new_message.header.stamp = rospy.Time.from_sec(odom_data[odom_data_pointer,0])
            new_message.header.frame_id = 'odom'
            new_message.child_frame_id = 'laser_frame_not_sure'
            new_message.pose.pose.position.x = odom_data[odom_data_pointer,1]
            new_message.pose.pose.position.y = odom_data[odom_data_pointer,2]
            new_message.pose.pose.position.z = odom_data[odom_data_pointer,3]

            pub.publish(new_message)

            odom_data_pointer += 1

            if odom_data_pointer >= odom_data.shape[0]:
                break

        rate.sleep()

if __name__ == '__main__':
    try:
        prior_odom_publisher()
    except rospy.ROSInterruptException:
        pass

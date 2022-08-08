import rosbag
import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

odom_topic = "/vectornav/Odom"
imu_topic = "/vectornav/IMU"
input_rosbag_filename = "01_first_forest_place_20Hz_pointcloud_2019-07-18-13-59-33.bag"



with rosbag.Bag("added_imu_" + input_rosbag_filename, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(input_rosbag_filename).read_messages():
        if topic == odom_topic:
            new_imu_message = Imu()
            new_imu_message.header = msg.header
            new_imu_message.orientation = msg.pose.pose.orientation

            outbag.write(topic, msg, t)
            outbag.write(imu_topic, new_imu_message, t)

        else:
            outbag.write(topic, msg, t)





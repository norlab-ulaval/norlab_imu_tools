import rosbag
from Tkinter import Tk
from tkFileDialog import askopenfilenames

def get_filename_from_user():
    root = Tk()
    root.withdraw()
    filenames = askopenfilenames(defaultextension='.bag',
                                 filetypes=[('ROS Bag files', '.bag')],
                                 title="Select .bag files",
                                 multiple=True)
    return filenames

bag_filenames = get_filename_from_user()

if len(bag_filenames) == 0:
    print('No file selected, terminating')
    exit()

for bag_filename in bag_filenames:
    print('Processing file: ' + bag_filename)

    bag_filename_stripped = bag_filename.split('/')[-1][0:-4]

    bag = rosbag.Bag(bag_filename)
    icp_odom_file = open('/'.join(bag_filename.split('/')[0:-1])+'/'+bag_filename_stripped+'_icp_odom.csv', 'w')
    utm_odom_file = open('/'.join(bag_filename.split('/')[0:-1])+'/'+bag_filename_stripped+'_utm_odom.csv', 'w')
    imu_odom_file = open('/'.join(bag_filename.split('/')[0:-1])+'/'+bag_filename_stripped+'_imu_odom.csv', 'w')

    icp_match_stats_file = open('/'.join(bag_filename.split('/')[0:-1]) + '/' + bag_filename_stripped + '_icp_matchStats.csv', 'w')
    icp_mapUp_stats_file = open('/'.join(bag_filename.split('/')[0:-1]) + '/' + bag_filename_stripped + '_icp_mapUpStats.csv', 'w')
    #measurement_joy_trigger_file = open('/'.join(bag_filename.split('/')[0:-1])+'/'+bag_filename_stripped+'_joy.csv', 'w')

    for topic, msg, t in bag.read_messages(topics=['/odom_utm', '/icp_odom', '/imu_odom', '/icp_map_iteration_statistics', '/icp_match_iteration_statistics']):
        if topic == '/icp_odom':
            icp_odom_file.write('{0},{1:09d},{2},{3},{4},{5},{6},{7},{8}\n'.format(msg.header.stamp.secs,
                                                                   msg.header.stamp.nsecs,
                                                                   msg.pose.pose.position.x,
                                                                   msg.pose.pose.position.y,
                                                                   msg.pose.pose.position.z,
                                                                   msg.pose.pose.orientation.x,
                                                                   msg.pose.pose.orientation.y,
                                                                   msg.pose.pose.orientation.z,
                                                                   msg.pose.pose.orientation.w))
	elif topic == '/imu_odom':
            imu_odom_file.write('{0},{1:09d},{2},{3},{4},{5},{6},{7},{8}\n'.format(msg.header.stamp.secs,
                                                                   msg.header.stamp.nsecs,
                                                                   msg.pose.pose.position.x,
                                                                   msg.pose.pose.position.y,
                                                                   msg.pose.pose.position.z,
                                                                   msg.pose.pose.orientation.x,
                                                                   msg.pose.pose.orientation.y,
                                                                   msg.pose.pose.orientation.z,
                                                                   msg.pose.pose.orientation.w))



        elif topic == '/odom_utm':
            utm_odom_file.write('{0},{1:09d},{2},{3},{4},{5},{6},{7},{8}\n'.format(msg.header.stamp.secs,
                                                                   msg.header.stamp.nsecs,
                                                                   msg.pose.pose.position.x,
                                                                   msg.pose.pose.position.y,
                                                                   msg.pose.pose.position.z,
                                                                   msg.pose.pose.orientation.x,
                                                                   msg.pose.pose.orientation.y,
                                                                   msg.pose.pose.orientation.z,
                                                                   msg.pose.pose.orientation.w))

        elif topic == '/icp_match_iteration_statistics':
            icp_match_stats_file.write('{0},{1:09d},{2},{3},{4},{5}\n'.format(msg.header.stamp.secs,
                                                                               msg.header.stamp.nsecs,
                                                                               msg.quaternion.x,
                                                                               msg.quaternion.y,
                                                                               msg.quaternion.z,
                                                                               msg.quaternion.w))

        elif topic == '/icp_map_iteration_statistics':
            icp_mapUp_stats_file.write('{0},{1:09d},{2},{3},{4},{5}\n'.format(msg.header.stamp.secs,
                                                                               msg.header.stamp.nsecs,
                                                                               msg.quaternion.x,
                                                                               msg.quaternion.y,
                                                                               msg.quaternion.z,
                                                                               msg.quaternion.w))

        else:
            raise ValueError('Message topic unexpected: '+topic)

    bag.close()
    icp_odom_file.close()
    utm_odom_file.close()
    imu_odom_file.close()
    icp_match_stats_file.close()
    icp_mapUp_stats_file.close()

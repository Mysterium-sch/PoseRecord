import pandas as pd
import rospy
import rosbag
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

import yuv_to_rgb

bridge = CvBridge()


imu_data_path = 'imu_data.csv'
img_data_path = 'img_data.csv'
pose_data_path = 'pose_data.csv'

imu_column_names = ['timestamp', 'ax', 'ay', 'az', 'gx', 'gy', 'gz']
img_column_names = ['timestamp', 'filename', 'type', 'width', 'height'] 
pose_column_names = ['timestamp', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'] 

imu_data = pd.read_csv(imu_data_path, header=None, names=imu_column_names)
img_data = pd.read_csv(img_data_path, header=None, names=img_column_names)
pose_data = pd.read_csv(pose_data_path, header=None, names=pose_column_names)

def find_first_close_imu_timestamp(img_timestamp, imu_data, limit):
    for imu_index, imu_timestamp in imu_data['timestamp'].items():  
        time_diff = abs(imu_timestamp - img_timestamp)
        if time_diff <= limit:
            return imu_index, imu_timestamp, time_diff
    return None, None, None

def imu_to_rosimu(timestamp_nsecs, linear, angular, seq, frame_id='map'):
    timestamp_nsecs = int(row['timestamp'])
    timestamp = rospy.Time(secs=timestamp_nsecs//int(1e9), nsecs=timestamp_nsecs%int(1e9))
    rosimu = Imu()
    rosimu.header.stamp = timestamp
    rosimu.header.frame_id = frame_id
    rosimu.header.seq = int(seq)
    rosimu.linear_acceleration.x = linear[0]
    rosimu.linear_acceleration.y = linear[1]
    rosimu.linear_acceleration.z = linear[2]
    rosimu.angular_velocity.x = angular[0]
    rosimu.angular_velocity.y = angular[1]
    rosimu.angular_velocity.z = angular[2]

    return rosimu, timestamp

def pose_to_rospose(timestamp_nsecs, position, orientation, frame_id='map'):
    timestamp_nsecs = int(row['timestamp'])
    timestamp = rospy.Time(secs=timestamp_nsecs//int(1e9), nsecs=timestamp_nsecs%int(1e9))
    rospose = PoseStamped()
    rospose.header.stamp = timestamp
    rospose.header.frame_id = frame_id
    rospose.pose.position.x = position[0]
    rospose.pose.position.y = position[1]
    rospose.pose.position.z = position[2]
    rospose.pose.orientation.x = orientation[0]
    rospose.pose.orientation.y = orientation[1]
    rospose.pose.orientation.z = orientation[2]
    rospose.pose.orientation.w = orientation[3]


    return rospose, timestamp

def img_to_rosimg(img, timestamp_nsecs):
    timestamp_nsecs = int(row['timestamp'])
    timestamp = rospy.Time(secs=timestamp_nsecs//int(1e9), nsecs=timestamp_nsecs%int(1e9))
    
    rosimage = bridge.cv2_to_compressed_imgmsg(image, dst_format='png')
   
    rosimage.header.stamp = timestamp

    return rosimage, timestamp, (image.shape[1], image.shape[0])

time_difference_limit = 0.003 * 1e9

first_img_timestamp = img_data['timestamp'][0]

imu_close_index, imu_close_timestamp, time_diff = find_first_close_imu_timestamp(first_img_timestamp, imu_data, time_difference_limit)

print(f"IMU row index: {imu_close_index}")
print(f"IMU timestamp: {imu_close_timestamp}")
print(f"Time difference: {time_diff} nanoseconds")

imu_starting_index = 800
img_starting_index = 0
seq = 0

img_topic = "/cam0/image_raw"
bag_path = './data.bag'

try:
    bag = rosbag.Bag(bag_path, 'w', compression='lz4')
    for index, row in imu_data.iterrows():
        rosimu, timestamp = imu_to_rosimu(row['timestamp'], [row['ax'], row['ay'], row['az']], [row['gx'], row['gy'], row['gz']], seq, 'map')
        seq += 1
        bag.write('/arcore/imu', rosimu, timestamp)

    for index, row in pose_data.iterrows():
        rospose, timestamp = pose_to_rospose(row['timestamp'], [row['tx'], row['ty'], row['tz']], [row['qx'], row['qy'], row['qz'], row['qw']], 'map')
        bag.write('/arcore/pose', rospose, timestamp)


    for index, row in img_data.iterrows():
        image_path = './images/' + row['filename']
        image = yuv_to_rgb.convert(image_path, None)
        rosimg, timestamp, resolution = img_to_rosimg(image, row['timestamp'])
        bag.write('/arcore/image_raw/compressed', rosimg, timestamp)
        
finally:
    bag.close()

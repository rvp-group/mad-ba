import numpy as np
from subprocess import call
import numpy as np
import os
import rosbag
from rospy import rostime
from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, PoseStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
import math
from tqdm import tqdm

# User parameters
bag_file_paths = []
for i in range(1):
    bag_file_paths.append('./bag/kitti_01_odom.bag')
    print(bag_file_paths[i])
tum_file_path = './init_tum/loam_opensource.tum'
odom_time_offset = 0.01
# User parameters for added topic
topic_name = "/loam_opensource"
frame_id = "map"
child_frame_id = "loam_opensource"
out_bag_name = './bag/kitti_01.bag'

def reorder_timestamps(rosbag_file_paths):

    bags = []
    for rosbag_file_path in rosbag_file_paths:
        try:
            bag = rosbag.Bag(rosbag_file_path, "r")
            bags.append(bag)
        except rosbag.bag.ROSBagException as e:
            print(e)
            if e.value == "empty file":
                return False
            else:
                return False
        except Exception as e:
            print(e)
            return False

    with rosbag.Bag(out_bag_name, 'w', 'lz4') as outbag:
        for bag in bags:
            for topic, msg, t in tqdm(bag.read_messages()):
                if topic == "/tf" and msg.transforms:
                    outbag.write(topic, msg, msg.transforms[0].header.stamp)
                elif topic == '/kitti/velo/pointcloud':
                    outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
    return True

def add_odom_topic(rosbag_file_path, tum_file_path, topic_name, frame_id, child_frame_id, odom_time_offset):

    print('Adding odometry from {} to {} on topic {}'.format(tum_file_path, rosbag_file_path, topic_name))
   # Try to open the bag file
    try:
        bag = rosbag.Bag(rosbag_file_path, "r")
        messages_num = bag.get_message_count()
    except rosbag.bag.ROSBagException as e:
        print(e)
        if e.value == "empty file":
            messages_num = 0
        else:
            return
    except Exception as e:
        print(e)
        return

    if messages_num > 0:
        bag = rosbag.Bag(rosbag_file_path, "a")
    else:
        bag = rosbag.Bag(rosbag_file_path, "w")

    # Try to read odom file
    with open(tum_file_path) as fp:
        lines = fp.read().splitlines()
        timestamps = [] # To check if two messages have the same time
        for i,line in tqdm(enumerate(lines)):
            # Convert str to float
            if '#' in line:
                continue 
            el = [float(el) for el in line.split()]
            if el[0] in timestamps:
                print("Two messages with the same timestamps. Skipping the second one: " + str(el[0]))
                continue
            # Add timestamp to list
            timestamps.append(el[0])
            # Offset for writing in .bag The timestamps itself stays the same
            timeOff = rostime.Duration(math.floor(odom_time_offset), odom_time_offset % 1 * 1e9)

            # Create odometry message
            odom = Odometry()
            odom.header.frame_id = frame_id
            odom.child_frame_id = child_frame_id
            odom.header.stamp.secs = math.floor(el[0])
            odom.header.stamp.nsecs = int((el[0] - math.floor(el[0])) * 1e9)
            odom.header.seq = i # Allows to detect whether timestamps are not sequentially ordered
            odom.pose.pose.position.x = el[1]
            odom.pose.pose.position.y = el[2]
            odom.pose.pose.position.z = el[3]
            odom.pose.pose.orientation.x = el[4]
            odom.pose.pose.orientation.y = el[5]
            odom.pose.pose.orientation.z = el[6]
            odom.pose.pose.orientation.w = el[7]
            time = rostime.Time(odom.header.stamp.secs, odom.header.stamp.nsecs)
            bag.write(topic_name, odom, time - timeOff)

    bag.close()


if __name__ == "__main__":

    # Reorder messages - creates new .bag.out file
    if reorder_timestamps(bag_file_paths):
        # Add odom topic
        pass
        add_odom_topic(out_bag_name, tum_file_path, topic_name, frame_id, child_frame_id, odom_time_offset)

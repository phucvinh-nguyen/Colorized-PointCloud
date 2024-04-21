#!/usr/bin/env python3
import rospy
import message_filters
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs.msg import CompressedImage
from livox_ros_driver.msg import CustomMsg
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import struct
import std_msgs.msg
import os

# Initialization
bridge = CvBridge()
output_directory = "/home/phucvinh/process_point/data/colored_pcd/"
count = 0
def pad_zeros(val, num_digits=6):
    return f"{val:0{num_digits}d}" 

def save_point_cloud_to_pcd(points_list, filename):
    with open(filename, 'w') as file:
        file.write("VERSION .7\n")
        file.write("FIELDS x y z rgb\n")
        file.write("SIZE 4 4 4 4\n")
        file.write("TYPE F F F U\n")
        file.write("COUNT 1 1 1 1\n")
        file.write("WIDTH {}\n".format(len(points_list)))
        file.write("HEIGHT 1\n")
        file.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        file.write("POINTS {}\n".format(len(points_list)))
        file.write("DATA ascii\n")
        for point in points_list:
            x, y, z, rgb = point
            r, g, b = (rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF
            packed_rgb = (r << 16) | (g << 8) | b  # pack RGB the way PCL expects
            file.write(f"{x} {y} {z} {packed_rgb}\n")



def process_data(image_data, lidar_data):
    global count
    try:
        # Sử dụng hàm này cho CompressedImage
        cv_image = bridge.compressed_imgmsg_to_cv2(image_data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
        return

    # Camera and transformation matrices
    camera_matrix = np.array([[863.590518437255, 0, 863.100180533059], [0, 621.666074631063, 533.971978652819], [0, 0, 1]])
    transformation_matrix = np.array([
        [0.00162756, -0.999991, 0.00390957, 0.0409257],
        [-0.0126748, -0.00392989, -0.999912, 0.0318424],
        [0.999918, 0.00157786, -0.012681, -0.0927219],
        [0.0, 0.0, 0.0, 1.0]
    ])

    points_list = []
    for point in lidar_data.points:  # Access points from CustomMsg directly
        x, y, z = point.x, point.y, point.z
        point_3d = np.array([x, y, z, 1])
        point_3d_camera = transformation_matrix.dot(point_3d)

        if point_3d_camera[2] > 0:
            point_2d = camera_matrix.dot(point_3d_camera[:3])
            point_2d /= point_2d[2]
            x, y = int(point_2d[0]), int(point_2d[1])

            if 0 <= x < cv_image.shape[1] and 0 <= y < cv_image.shape[0]:
                b, g, r = cv_image[y, x]
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
                new_point = [point.x, point.y, point.z, rgb]
                points_list.append(new_point)

    # Publish colored point cloud
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()  # or use lidar_data.header.stamp if synchronized
    header.frame_id = 'livox_frame'  # Change according to your frame setup
    fields = [pc2.PointField(name, offset, datatype, count) for name, offset, datatype, count in [
        ('x', 0, pc2.PointField.FLOAT32, 1),
        ('y', 4, pc2.PointField.FLOAT32, 1),
        ('z', 8, pc2.PointField.FLOAT32, 1),
        ('rgb', 12, pc2.PointField.UINT32, 1),
    ]]
    colored_pc2 = pc2.create_cloud(header, fields, points_list)
    pub.publish(colored_pc2)
    pcd_color_file = output_directory + pad_zeros(count) + ".pcd"
    save_point_cloud_to_pcd(points_list, pcd_color_file)
    count+=1
# ROS Node and Subscribers
rospy.init_node('pointcloud_colorizer')
image_sub = message_filters.Subscriber('/right_camera/image/compressed', CompressedImage)
pointcloud_sub = message_filters.Subscriber('/livox/lidar', CustomMsg)

# Using approximate time policy to synchronize the topics
ts = message_filters.ApproximateTimeSynchronizer([image_sub, pointcloud_sub], 10, 0.1, allow_headerless=True)
ts.registerCallback(process_data)

pub = rospy.Publisher('colored_pointcloud', PointCloud2, queue_size=10)

rospy.spin()


#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import struct
import std_msgs.msg

bridge = CvBridge()
cv_image = None

def rgb_to_float(color):
    return struct.unpack('f', struct.pack('I', (color[2] << 16) | (color[1] << 8) | color[0]))[0]

def image_callback(image_data):
    global cv_image, brightness_factor
    try:
        # convert img_msg to opencv
        cv_image_original = bridge.imgmsg_to_cv2(image_data, "bgr8")
        # apply adjust brightness
        cv_image = cv2.convertScaleAbs(cv_image_original, alpha=1.5, beta=0)
    except CvBridgeError as e:
        print(e)

def lidar_callback(lidar_data):
    global cv_image
    if cv_image is None:
        return  
    
    camera_matrix = np.array([[264.2125, 0, 341.635], [0, 264.155, 183.993], [0, 0, 1]])
    transformation_matrix = np.array([
        [0.99885234, -0.04786419, 0.00173618, 0.12228088],
        [0.00135912, -0.00790913, -0.9999678, -3.28497173],
        [0.04787638, 0.99882254, -0.007835, -0.68262552],
        [0.0, 0.0, 0.0, 1.0]
    ])

    points_list = []

    for point in pc2.read_points(lidar_data, field_names=("x", "y", "z"), skip_nans=True):
        point_3d = np.array([point[0], point[1], point[2], 1])
        point_3d_camera = transformation_matrix.dot(point_3d)

        if point_3d_camera[2] > 0:
            point_2d = camera_matrix.dot(point_3d_camera[:3])
            point_2d /= point_2d[2]

            x, y = int(point_2d[0]), int(point_2d[1])
            
            if 0 <= x < cv_image.shape[1] and 0 <= y < cv_image.shape[0]:
                # add RGB from image
                b, g, r = cv_image[y, x]
                # add r, g, b to one pack
                rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, 0))[0]
                new_point = [point[0], point[1], point[2], rgb]
                points_list.append(new_point)

    header = std_msgs.msg.Header()
    header.stamp = lidar_data.header.stamp
    header.frame_id = lidar_data.header.frame_id
    
    fields = [
        pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
        pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
        pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
        pc2.PointField('rgb', 12, pc2.PointField.UINT32, 1),
    ]

    colored_pc2 = pc2.create_cloud(header, fields, points_list)
    pub.publish(colored_pc2)

rospy.init_node('pointcloud_colorizer')
image_sub = rospy.Subscriber('/zed2/camera/right/image_raw', Image, image_callback)
pointcloud_sub = rospy.Subscriber('/velodyne_points', PointCloud2, lidar_callback)

pub = rospy.Publisher('colored_pointcloud', PointCloud2, queue_size=10)

rospy.spin()

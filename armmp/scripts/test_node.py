import rospy
import struct
import tf
import numpy as np
from sensor_msgs.msg import Image, PointCloud2

datatype = {1:1, 2:1, 3:2, 4:2, 5:4, 6:4, 7:4, 8:8}

def get_xyz(point_2d, pc_msg):
        arrayPosition = point_2d[0]*pc_msg.row_step + point_2d[1]*pc_msg.point_step # point_2d: y,x
        pos_x = arrayPosition + pc_msg.fields[0].offset # X has an offset of 0
        len_x = datatype[pc_msg.fields[0].datatype]
        pos_y = arrayPosition + pc_msg.fields[1].offset # Y has an offset of 4
        len_y = datatype[pc_msg.fields[1].datatype]
        pos_z = arrayPosition + pc_msg.fields[2].offset # Z has an offset of 8
        len_z = datatype[pc_msg.fields[2].datatype]
	
        print(pos_x)
        print(pos_y)
        print(pos_z)

        try:
            x = struct.unpack('f', pc_msg.data[pos_x: pos_x+len_x])[0] # read 4 bytes as a float number
            y = struct.unpack('f', pc_msg.data[pos_y: pos_y+len_y])[0]
            z = struct.unpack('f', pc_msg.data[pos_z: pos_z+len_z])[0]
            return [x,y,z]
        except:
            return None

rospy.init_node('test_node', anonymous=True)
pc_msg = rospy.wait_for_message('/realsense2/depth/color/points', PointCloud2)
print(pc_msg.width)
print(pc_msg.height)
point_3d = get_xyz([583, 654], pc_msg)
print(point_3d)

robot_link = 'base_link'
camera_link = 'realsense2_color_optical_frame'
listener = tf.TransformListener()
listener.waitForTransform(robot_link, camera_link, rospy.Time(), rospy.Duration(4.0))
translation, rotation = listener.lookupTransform(robot_link, camera_link, rospy.Time(0))
cam2rob = np.dot(tf.transformations.translation_matrix(translation), tf.transformations.quaternion_matrix(rotation))
point_3d_rob = np.dot(cam2rob, np.array(point_3d+[1]))[:3]
print(point_3d_rob)


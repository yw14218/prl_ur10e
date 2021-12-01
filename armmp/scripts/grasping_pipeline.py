#!/usr/bin/env python

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from robotiq_3f_gripper_articulated_msgs.msg import Robotiq3FGripperRobotOutput
from Robotiq3FGripperSimpleController import genCommand
import roslib;
roslib.load_manifest('robotiq_3f_gripper_control')
import struct
import tf
import numpy as np
from sensor_msgs.msg import Image, PointCloud2

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

datatype = {1:1, 2:1, 3:2, 4:2, 5:4, 6:4, 7:4, 8:8}


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupGrasp(object):
    """MoveGroupGrasp"""

    def __init__(self):
        super(MoveGroupGrasp, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_grasp", anonymous=True)

        rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, self.callback_darknet)

        pub_gripper = rospy.Publisher('Robotiq3FGripperRobotOutput', Robotiq3FGripperRobotOutput)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints). 
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        print("#######")
        print(move_group.get_current_pose())

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.pub_gripper = pub_gripper
        self.xCenterObject = 0
        self.yCenterObject = 0

    def callback_darknet(self, data):
        for box in data.bounding_boxes:
            if box.Class == "bottle":
                self.xCenterObject = int((box.xmax-box.xmin)/2) + box.xmin
                self.yCenterObject = int((box.ymax-box.ymin)/2) + box.ymin
                #print("X: " + str(self.xCenterObject) + " Y: " + str(self.yCenterObject) )
                #rospy.loginfo(
                #    box.Class + ": " + 
                #    "Xmin: {}, Xmax: {} Ymin: {}, Ymax: {}".format(
                #        box.xmin, box.xmax, box.ymin, box.ymax
                #    )
                #)
        #print(data.xmin)

    def get_xyz(self, point_2d, pc_msg):
        arrayPosition = point_2d[0]*pc_msg.row_step + point_2d[1]*pc_msg.point_step # point_2d: y,x
        pos_x = arrayPosition + pc_msg.fields[0].offset # X has an offset of 0
        len_x = datatype[pc_msg.fields[0].datatype]
        pos_y = arrayPosition + pc_msg.fields[1].offset # Y has an offset of 4
        len_y = datatype[pc_msg.fields[1].datatype]
        pos_z = arrayPosition + pc_msg.fields[2].offset # Z has an offset of 8
        len_z = datatype[pc_msg.fields[2].datatype]

        try:
            x = struct.unpack('f', pc_msg.data[pos_x: pos_x+len_x])[0] # read 4 bytes as a float number
            y = struct.unpack('f', pc_msg.data[pos_y: pos_y+len_y])[0]
            z = struct.unpack('f', pc_msg.data[pos_z: pos_z+len_z])[0]
            return [x,y,z]
        except:
            return None

    def close_gripper(self):
        command = Robotiq3FGripperRobotOutput();
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150   
        command = genCommand('c', command)
        print(command)
        self.pub_gripper.publish(command)


    def open_gripper(self):
        command = Robotiq3FGripperRobotOutput();
        command.rACT = 1
        command.rGTO = 1
        command.rSPA = 255
        command.rFRA = 150   
        command = genCommand('o', command)
        print(command)
        self.pub_gripper.publish(command)


    def go_to_joint_state_home(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = -1.7797206083880823
        joint_goal[1] = -0.9760516446879883
        joint_goal[2] = -2.015587329864502
        joint_goal[3] = -3.7978626690306605
        joint_goal[4] = -1.8281634489642542
        joint_goal[5] = 0.7467105388641357
        move_group.go(joint_goal, wait=True)
        move_group.stop()
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    
    def plan_cartesian_path(self, point_3d_rob, scale=1):
        move_group = self.move_group
        waypoints = []
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = -0.193427085918
        pose_goal.orientation.y = -0.738633488235
        pose_goal.orientation.z = -0.568130650499
        pose_goal.orientation.w = 0.306975726165
        # TODO
        pose_goal.position.x = point_3d_rob[0]
        pose_goal.position.y = point_3d_rob[1]
        pose_goal.position.z = point_3d_rob[2]


        waypoints.append(pose_goal)

        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)  
        self.display_trajectory(plan)
        input("============ Press `Enter` to dispay trajectory ...")
        move_group.execute(plan, wait=True)
        return plan, fraction

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def get_xy(self):
        return [self.yCenterObject, self.xCenterObject]

    def cam2roboframe(self, point_3d):
        offset = 0.1
        robot_link = 'base_link'
        camera_link = 'realsense2_color_optical_frame'
        listener = tf.TransformListener()
        listener.waitForTransform(robot_link, camera_link, rospy.Time(), rospy.Duration(4.0))
        translation, rotation = listener.lookupTransform(robot_link, camera_link, rospy.Time(0))
        cam2rob = np.dot(tf.transformations.translation_matrix(translation), tf.transformations.quaternion_matrix(rotation))
        point_3d_rob = np.dot(cam2rob, np.array(point_3d+[1]))[:3]
        return point_3d_rob

    
def main():
    try:
        grasp = MoveGroupGrasp()

        input("============ Press `Enter` to continue ...")
        #grasp.go_to_joint_state_home()
        pc_msg = rospy.wait_for_message('/realsense2/depth/color/points', PointCloud2)
        point_2d = grasp.get_xy()
        point_3d = grasp.get_xyz(point_2d, pc_msg)
        point_3d_rob = grasp.cam2roboframe(point_3d)
        print(point_3d_rob)


        grasp.plan_cartesian_path(point_3d_rob)
        grasp.close_gripper()

        input("============ Press `Enter` to move back ...")
        grasp.go_to_joint_state_home()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

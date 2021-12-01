#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from darknet_ros_msgs.msg import BoundingBoxes

def callback_darknet(data):
    for box in data.bounding_boxes:
        if box.Class == "bottle":
            xCenterObject = int((box.xmax-box.xmin)/2) + box.xmin
            yCenterObject = int((box.ymax-box.ymin)/2) + box.ymin
            print("X: " + str(xCenterObject) + " Y: " + str(yCenterObject) )
            rospy.loginfo(
                box.Class + ": " + 
                "Xmin: {}, Xmax: {} Ymin: {}, Ymax: {}".format(
                    box.xmin, box.xmax, box.ymin, box.ymax
                )
            )
    #print(data.xmin)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, callback_darknet)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
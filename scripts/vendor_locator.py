#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
import tf
import math
from enum import Enum

'''
# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup (hw2+)
use_gazebo = rospy.get_param("sim")

# how is nav_cmd being decided -- human manually setting it, or rviz
rviz = rospy.get_param("rviz")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")


# threshold at which we consider the robot at a location
POS_EPS = .1
THETA_EPS = .3

# time to stop at a stop sign
STOP_TIME = 3

# minimum distance from a stop sign to obey it
STOP_MIN_DIST = .5

# time taken to cross an intersection
CROSSING_TIME = 3

# state machine modes, not all implemented
class Mode(Enum):
    IDLE = 1
    POSE = 2
    STOP = 3
    CROSS = 4
    NAV = 5
    MANUAL = 6


print "supervisor settings:\n"
print "use_gazebo = %s\n" % use_gazebo
print "rviz = %s\n" % rviz
print "mapping = %s\n" % mapping

'''

object_dict = {}

def object_detected_callback(msg):
    # Process object seen
    # If new object: mark location and add to publish list
    (trans1, rot1) = trans_listener.lookupTransform('/base_footprint', '/map', rospy.Time(0))
    global object_dict
    if (msg.name == "cat"):
        #theta = tf.transformations.euler_from_quaternion(rot1)[2]
        rospy.loginfo("ITS ACTUALLY A CAT")
        object_dict["cat"] = (trans1, rot1)

if __name__ == '__main__':

    # Initialization
    rospy.init_node('vendor_locator', anonymous=True)
    # initialize variables

    rospy.loginfo("MAIN TESTING")

    # Setup subscribers
    trans_listener = tf.TransformListener()

    # Setup publishers
    tf_broadcaster = tf.TransformBroadcaster()

    # Subscribers
    # Cat subscriber
    rospy.Subscriber('/detector/cat', DetectedObject, object_detected_callback)

    # Main loop
    while not rospy.is_shutdown():
        for object_name, position in object_dict.items():
            tf_broadcaster.sendTransform(position[0], position[1], rospy.Time(0), "/map", object_name)
            #tf_broadcaster.sendTransform((0,1,0),(0,0,0,1), rospy.Time.now(), "map", "cat")
            rospy.loginfo("SENT  TRANSFORM")
        rospy.sleep(1.0)

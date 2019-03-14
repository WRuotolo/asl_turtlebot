#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
import tf
import math
from enum import Enum

food_options = ['dog', 'cup', 'cat', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'pizza', 'donut', 'cake']
object_dict = {}

def object_detected_callback(msg):
    # Process object seen
    # If new object: mark location and add to publish list
    (trans1, rot1) = trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
    global object_dict
    if msg.confidence > 0.65:
        if msg.name in food_options:
            object_dict[msg.name] = (trans1, rot1)

def location_callback(msg):
    (trans, rot) = trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
    object_dict[msg.data] = (trans, rot)

if __name__ == '__main__':

    # Global variables
    #global object_dict

    # Initialization
    rospy.init_node('vendor_locator', anonymous=True)
    # initialize variables

    # Setup subscribers
    trans_listener = tf.TransformListener()

    # Setup publishers
    #cat_broadcaster = tf.TransformBroadcaster()
    #dog_broadcaster = tf.TransformBroadcaster()
    obj_broadcaster = tf.TransformBroadcaster()

    # Subscribers
    # Create subscribers
    for food in food_options:
        rospy.Subscriber('detector/' + food, DetectedObject, object_detected_callback)
    # rospy.Subscriber('/detector/cat', DetectedObject, object_detected_callback)
    # rospy.Subscriber('/detector/dog', DetectedObject, object_detected_callback)

    rospy.Subscriber('manual_location', String, location_callback)

    rospy.loginfo("VENDOR LOCATOR INITIALIZED")

    # Main loop
    while not rospy.is_shutdown():
        for object_name, position in object_dict.items():
            rospy.loginfo("VENDOR BROADCASTED: " + object_name)
            obj_broadcaster.sendTransform(position[0], position[1], rospy.Time(0), object_name, "/map")
            rospy.sleep(0.1)
        rospy.sleep(1.0)

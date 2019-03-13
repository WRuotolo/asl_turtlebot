#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D, PoseStamped
import string
from std_msgs.msg import String
import tf
from visualization_msgs.msg import Marker, MarkerArray


def picked_callback(data):
    picked = string.split(',')
    marker_array = MarkerArray()

    for i, item in enumerate(picked):
    # quaternion = tf.transformations.quaternion_from_euler(0,0,th_g)
        marker = Marker()

        marker.header.frame_id = "/base_footprint"
        marker.type = marker.SPHERE
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.g = 0.75
        marker.color.b = 0.25
        marker.color.a = 1.0
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 2*0.05*(i-1) + 0.05
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1

        marker_array.markers.append(marker)

    picked_pub.publish(marker_array)

if __name__ == '__main__':
    rospy.init_node('visualizer')
    rate = rospy.Rate(10)

    food_options = ['banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'pizza', 'donut', 'cake']

    listener = tf.TransformListener()

    picked_pub = rospy.Publisher('/picked_markers', MarkerArray, queue_size = 1)
    vendor_pub = rospy.Publisher('/vendor_markers', MarkerArray, queue_size = 1)

    picked_sub = rospy.Subscriber('/picked_up', String, picked_callback)

    while not rospy.is_shutdown():
        marker_array = MarkerArray()

        for item in food_options:
            try:
                (trans, rot) = listener.lookupTransform('/' + item, '/map', rospy.Time().now())
                marker = Marker()

                marker.type = marker.SPHERE
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.color.g = 0.75
                marker.color.b = 0.25
                marker.color.a = 1.0
                marker.pose.position.x = trans[0]
                marker.pose.position.y = trans[1]
                marker.pose.position.z = 0
                marker.pose.orientation.x = 0
                marker.pose.orientation.y = 0
                marker.pose.orientation.z = 0
                marker.pose.orientation.w = 1

                marker_array.markers.append(marker)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        vendor_pub.publish(marker_array)


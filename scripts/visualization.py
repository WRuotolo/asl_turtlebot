#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D, PoseStamped
import tf
from visualization_msgs.msg import Marker

def callback(data):
    x_g = data.x
    y_g = data.y
    th_g = data.theta

    quaternion = tf.transformations.quaternion_from_euler(0,0,th_g)
    marker_g = Marker()

    marker_g.header.frame_id = "/map"
    # marker_g.type = marker.SPHERE
    # marker_g.scale.x = 0.15
    # marker_g.scale.y = 0.15
    # marker_g.scale.z = 0.15
    # marker_g.color.g = 0.5
    # marker_g.color.b = 0.5
    # marker_g.color.a = 1.0
    # marker_g.pose.position.x = x_g
    # marker_g.pose.position.y = y_g
    # marker_g.pose.position.z = 0
    marker_g.type = marker.ARROW
    marker_g.scale.x = 0.15
    marker_g.scale.y = 0.05
    marker_g.scale.z = 0.0
    marker_g.color.g = 0.75
    marker_g.color.b = 0.25
    marker_g.color.a = 1.0
    marker_g.pose.position.x = x_g
    marker_g.pose.position.y = y_g
    marker_g.pose.position.z = 0
    marker_g.pose.orientation.x = quaternion[0]
    marker_g.pose.orientation.y = quaternion[1]
    marker_g.pose.orientation.z = quaternion[2]
    marker_g.pose.orientation.w = quaternion[3]

    pub_goal.publish(marker_g)

if __name__ == '__main__':
    rospy.init_node('visualizer')
    rate = rospy.Rate(10)

    pub_turtle = rospy.Publisher('visualization_turtle', Marker, queue_size = 1.0)
    pub_goal = rospy.Publisher('visualization_goal', Marker, queue_size = 1.0)
    sub = rospy.Subscriber('/cmd_pose', Pose2D, callback)


    marker = Marker()

    marker.header.frame_id = "/base_footprint"
    marker.type = marker.SPHERE
    marker.scale.x = 0.15
    marker.scale.y = 0.15
    marker.scale.z = 0.15
    marker.color.r = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0

    while not rospy.is_shutdown():
        pub_turtle.publish(marker)


#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D, PoseStamped
import string
from std_msgs.msg import String
import tf
from visualization_msgs.msg import Marker, MarkerArray


def picked_callback(msg):
    picked = msg.data.split(',')
    marker_array = MarkerArray()
    if msg.data == '':
        return

    for i, item in enumerate(picked):
    # quaternion = tf.transformations.quaternion_from_euler(0,0,th_g)
        marker = Marker()

        color = color_dict[item]

        marker.header.frame_id = "/base_footprint"
        marker.type = marker.SPHERE
        marker.id = i
        scale = 0.2
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        marker.pose.position.x = -(2*0.05*(i-1) + 0.05)
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1

        marker_array.markers.append(marker)

    picked_pub.publish(marker_array)

def status_callback(msg):
	status_mode = msg.data

	if status_mode == 'Mode.EXP_IDLE':
		status = 'Idling while exploring'
	elif status_mode == 'Mode.EXP_NAV':
		status = 'Exploring...'
	elif status_mode == 'Mode.STOP':
		status = 'Stopped at stop sign'
	elif status_mode == 'Mode.CROSS':
		status = 'Crossing intersection'
	elif status_mode == 'Mode.DEL_IDLE':
		status = 'Waiting for delivery request'
	elif status_mode == 'Mode.DEL_PICKUP':
		status = 'Picking up object'
	elif status_mode == 'Mode.DEL_NAV_HOME':
		status = 'Delivering food to home'
	elif status_mode == 'Mode.FOLLOW_DOG':
		status = 'I am distracted...following a cute animal'
	else:
		status = status_mode


	#text status
	statusMarker = Marker()
	statusMarker.id = 10
	statusMarker.type = statusMarker.TEXT_VIEW_FACING
	statusMarker.header.frame_id = '/map'
	statusMarker.scale.z = 0.3
	statusMarker.pose.position.x = 2
	statusMarker.pose.position.y = 4
	statusMarker.pose.position.z = 0
	statusMarker.pose.orientation.x = 0
	statusMarker.pose.orientation.y = 0
	statusMarker.pose.orientation.z = 0
	statusMarker.pose.orientation.w = 1
	statusMarker.color.r = 0.0
	statusMarker.color.g = 1.0
	statusMarker.color.b = 0.0
	statusMarker.color.a = 0.75
	statusMarker.text = status
	status_pub.publish(statusMarker)


if __name__ == '__main__':
    rospy.init_node('visualizer')
    rate = rospy.Rate(10)

    food_options = ['banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'pizza', 'donut', 'cake']
    color_dict = {}
    color_dict['banana'] = (1.0, 1.0, 0.0)
    color_dict['broccoli'] = (0.0, 1.0, 0.0)
    color_dict['carrot'] = (1.0, 0.5, 0.2)
    color_dict['pizza'] = (1.0, 0.0, 0.0)
    color_dict['donut'] = (1.0, 0.4, 0.7)
    color_dict['cake'] = (0.4, 1.0, 1.0)

    listener = tf.TransformListener()

    harlan_pub = rospy.Publisher('/harlan_marker', Marker, queue_size = 1)
    picked_pub = rospy.Publisher('/picked_markers', MarkerArray, queue_size = 1)
    vendor_pub = rospy.Publisher('/vendor_markers', MarkerArray, queue_size = 1)

    picked_sub = rospy.Subscriber('/picked_up', String, picked_callback)
    status_msg = rospy.Subscriber('/status', String, status_callback)
    status_pub = rospy.Publisher('status_marker', Marker, queue_size = 1)

    while not rospy.is_shutdown():
        marker_array = MarkerArray()

        marker = Marker()
        marker.id = 0
        marker.type = marker.CUBE
        marker.header.frame_id = '/base_footprint'
        marker.scale.x = 0.25
        marker.scale.y = 0.2
        marker.scale.z = 0.01
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.75
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1

        harlan_pub.publish(marker)

        for i, item in enumerate(food_options):
            try:
                (trans, rot) = listener.lookupTransform('/map', '/' + item, rospy.Time(0))
                (trans_h, rot_h) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))

                color = color_dict[item]

                marker = Marker()
                marker.id = i
                marker.type = marker.SPHERE
                marker.header.frame_id = "/map"
                scale = 0.3
                marker.scale.x = scale
                marker.scale.y = scale
                marker.scale.z = scale
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                marker.color.a = 0.9
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




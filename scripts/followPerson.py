#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D, PoseStamped
import tf

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")

distance, cornerx, cornery, cornerdx, cornerdy = 0.0, 0.0, 0.0, 0.0, 0.0
thetaL, thetaR = 0.0, 0.0


class GoalPoseCommander:

    def __init__(self):
        rospy.init_node('goal_pose_commander', anonymous=True)
        # initialize variables
        self.x_g = None
        self.y_g = None
        self.theta_g = None
        self.goal_pose_received = False
        self.trans_listener = tf.TransformListener()
        # command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        rospy.Subscriber('/detector/person', DetectedObject, self.personCallback)
	trans_listener = tf.TransformListener()

    def personCallback(self, data):
	global distance, cornerx, cornery, cornerdx, cornerdy
	global thetaL, thetaR
	distance = data.distance
	thetaL = data.thetaleft
	thetaR = data.thetaright
	cornery, cornerx, cornerdy, cornerdx = data.corners

    def publish_goal_pose(self):
        """ sends the current desired pose to the pose controller """
	# in global coords? 
        pose_g_msg = Pose2D()
        pose_g_msg.x = 0.0
        pose_g_msg.y = 0.0
        pose_g_msg.theta = self.theta_g
        self.pose_goal_publisher.publish(pose_g_msg)

    def loop(self):
	global distance, cornerx, cornery, cornerdx, cornerdy
	global thetaL, thetaR
	camPose = PoseStamped()
	camPose.header.frame_id = '/base_footprint'
	camPose.pose.position.x = 
	camPose.pose.position.y = 
	obX_cam = 0.0
	oby_cam = 0.0
	self.theta_g = thetaR+(thetaL-thetaR)/2.0
        self.publish_goal_pose()

    def run(self):
        rate = rospy.Rate(3)
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = GoalPoseCommander()
    sup.run()

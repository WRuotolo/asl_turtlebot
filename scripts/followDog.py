#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
import tf
from asl_turtlebot.msg import DetectedObject


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
        self.trans_broadcast = tf.TransformBroadcaster()
        # command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        rospy.Subscriber('/detector/dog', DetectedObject, self.dogCallback)
	   #trans_listener = tf.TransformListener()

    def dogCallback(self, data):
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
        pose_g_msg.x = self.x_g
        pose_g_msg.y = self.y_g
        pose_g_msg.theta = self.theta_g
        self.pose_goal_publisher.publish(pose_g_msg)

    def loop(self):
    	global distance, cornerx, cornery, cornerdx, cornerdy
    	global thetaL, thetaR
    	camPose = PoseStamped()
    	self.trans_broadcast.sendTransform((0.0,0.0,0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "/test_tf", "/map")
    	camPose.header.frame_id = '/base_footprint'
    	#camPose.pose.position.x = 
    	#camPose.pose.position.y = 
        x_middle = (768/2)-(cornerx+cornerdx/2)
        d = 1.0
        rospy.loginfo(self.trans_listener.lookupTransform("test_tf", "map", rospy.Time.now()))
        try:
        	t = self.trans_listener.getLatestCommonTime("/odom", "/base_footprint")
        	(translation, rotation) = self.trans_listener.lookupTransform("/map","/base_footprint",t)
        	th_goal = tf.transformations.euler_from_quaternion(rotation)[2]
        	self.theta_g = th_goal + np.atan2(x_middle,d)
        	self.publish_goal_pose()
      	except: 
	        #obX_cam = 0.0
	        #oby_cam = 0.0
	        rospy.loginfo("hi")

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = GoalPoseCommander()
    sup.run()

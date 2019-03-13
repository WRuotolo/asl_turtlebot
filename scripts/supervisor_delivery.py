#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String, Bool
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject
import tf
import math
from enum import Enum
import time

# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup (hw2+)
use_gazebo = rospy.get_param("sim")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")


# threshold at which we consider the robot at a location
POS_EPS = .15
THETA_EPS = .3

# time to stop at a stop sign
STOP_TIME = 10

# minimum distance from a stop sign to obey it
STOP_MIN_DIST = .5

# time taken to cross an intersection
CROSSING_TIME = 3

# dog
distance, cornerx, cornery, cornerdx, cornerdy = 0.0, 0.0, 0.0, 0.0, 0.0
thetaL, thetaR = 0.0, 0.0

# state machine modes, not all implemented
class Mode(Enum):
    EXP_IDLE = 1
    EXP_POSE = 2
    STOP = 3
    CROSS = 4
    EXP_NAV = 5
    MANUAL = 6

    DEL_IDLE = 7
    DEL_POSE = 8
    DEL_NAV_OBJ = 9
    DEL_NAV_HOME = 10
    DEL_PICKUP = 11

    FOLLOW_DOG = 12

    
print "supervisor settings:\n"
print "use_gazebo = %s\n" % use_gazebo
print "mapping = %s\n" % mapping

class Supervisor:

    def __init__(self):
        rospy.init_node('turtlebot_supervisor_nav', anonymous=True)
        # initialize variables
        self.x = 0
        self.y = 0
        self.theta = 0

        self.x_g = 0
        self.y_g = 0
        self.theta_g = 0
        self.mode = Mode.EXP_IDLE
        self.last_mode_printed = None
        self.trans_listener = tf.TransformListener()
        # command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        # nav pose for controller
        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        # command vel (used for idling)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.explore = True

        self.delivery_obj_names = []
        self.obj_idx = 0
        self.obj_tot = 0
        # subscribers
        # stop sign detector
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)
        # high-level navigation pose
        rospy.Subscriber('/nav_pose', Pose2D, self.nav_pose_callback)
        # if using gazebo, we have access to perfect state
        if use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        # we can subscribe to nav goal click
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        rospy.Subscriber('/state', Bool, self.state_callback)
        rospy.Subscriber('/delivery_request', String, self.delivery_request_callback)
        rospy.Subscriber('/detector/dog', DetectedObject, self.dogCallback)

    def dogCallback(self, data):
    	global distance, cornerx, cornery, cornerdx, cornerdy
    	global thetaL, thetaR
    	distance = data.distance
    	thetaL = data.thetaleft
    	thetaR = data.thetaright
    	cornery, cornerx, cornerdy, cornerdx = data.corners
    	self.mode = Mode.FOLLOW_DOG

    def delivery_request_callback(self, msg):
        self.delivery_obj_names = msg.data.split(',')
        self.obj_tot = len(self.delivery_obj_names)
        self.mode = Mode.DEL_NAV_OBJ
        rospy.loginfo("Got delivery request")
        self.nav_to_obj(self.obj_idx)

    def state_callback(self, msg):
        if msg.data != self.explore:
            if msg.data:
                self.mode = Mode.EXP_IDLE
            else:
                self.mode = Mode.DEL_IDLE

            self.explore = msg.data

    def gazebo_callback(self, msg):
        pose = msg.pose[msg.name.index("turtlebot3_burger")]
        twist = msg.twist[msg.name.index("turtlebot3_burger")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]

    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """
        origin_frame = "/map" if mapping else "/odom"
        print("rviz command received!")
        try:
            
            nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
            self.x_g = nav_pose_origin.pose.position.x
            self.y_g = nav_pose_origin.pose.position.y
            quaternion = (
                    nav_pose_origin.pose.orientation.x,
                    nav_pose_origin.pose.orientation.y,
                    nav_pose_origin.pose.orientation.z,
                    nav_pose_origin.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            self.theta_g = euler[2]
            rospy.loginfo("Setting goal to be %f, %f, %f", self.x_g, self.y_g, self.theta_g)
            rospy.loginfo("Setting to stop")
            cmd_msg = Twist()
            cmd_msg.linear.x = 0
            cmd_msg.angular.z = 0
            self.cmd_vel_publisher.publish(cmd_msg)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        self.mode = Mode.EXP_NAV
        if not self.explore:
            self.explore = True

    def nav_pose_callback(self, msg):
        self.x_g = msg.x
        self.y_g = msg.y
        self.theta_g = msg.theta
        self.mode = Mode.EXP_NAV

    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign
        dist = msg.distance

        # if close enough and in nav mode, stop
        if dist > 0 and dist < STOP_MIN_DIST and self.mode == Mode.NAV:
            self.init_stop_sign()

    def go_to_pose(self):
        """ sends the current desired pose to the pose controller """

        pose_g_msg = Pose2D()
        pose_g_msg.x = self.x_g
        pose_g_msg.y = self.y_g
        pose_g_msg.theta = self.theta_g
        #print("Publishing pose")
        self.pose_goal_publisher.publish(pose_g_msg)

    def nav_to_pose(self):
        """ sends the current desired pose to the naviagtor """

        nav_g_msg = Pose2D()
        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.theta_g
        #print("Publishing nav")
        self.nav_goal_publisher.publish(nav_g_msg)

    def stay_idle(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()
        #print("Idling")
        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self,x,y,theta,pos_eps, theta_eps):
        """ checks if the robot is at a pose within some threshold """
        return (abs(x-self.x)<pos_eps and abs(y-self.y)<pos_eps and abs(theta-self.theta)<theta_eps)

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """
        #print("initiating stop sign")
        self.stop_sign_start = rospy.get_rostime()
        self.mode = Mode.STOP

    def has_stopped(self):
        """ checks if stop sign maneuver is over """

        return (self.mode == Mode.STOP and (rospy.get_rostime()-self.stop_sign_start)>rospy.Duration.from_sec(STOP_TIME))
        
    def init_crossing(self):
        """ initiates an intersection crossing maneuver """

        self.cross_start = rospy.get_rostime()
        self.mode = Mode.CROSS

    def has_crossed(self):
        """ checks if crossing maneuver is over """

        return (self.mode == Mode.CROSS and (rospy.get_rostime()-self.cross_start)>rospy.Duration.from_sec(CROSSING_TIME))


    def nav_to_obj(self, idx):
        #rospy.loginfo("inside nav to obj")
        #rospy.loginfo("index is %d", idx)
        (trans, rot) = self.trans_listener.lookupTransform('/map', self.delivery_obj_names[idx], rospy.Time(0))
        self.x_g = trans[0]
        self.y_g = trans[1]
        euler = tf.transformations.euler_from_quaternion(rot)
        self.theta_g = euler[2]
        #rospy.loginfo("object ind %f", idx)
        #rospy.loginfo("x goal %f", self.x_g)
        nav_g_msg = Pose2D()
        nav_g_msg.x = trans[0]
        nav_g_msg.y = trans[1]
        nav_g_msg.theta = (self.theta_g + self.theta)/2
        self.nav_goal_publisher.publish(nav_g_msg)

    def nav_to_home(self):
        self.x_g = 0.0
        self.y_g = 0.0
        self.theta_g = 0.1
        nav_g_msg = Pose2D()
        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.theta_g
        self.nav_goal_publisher.publish(nav_g_msg)

    def follow_dog(self):
    	global distance, cornerx, cornery, cornerdx, cornerdy
    	global thetaL, thetaR
    	camPose = PoseStamped()
    	self.trans_broadcast.sendTransform((0.0,0.0,0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), "/test_tf", "/map")
    	camPose.header.frame_id = '/base_footprint'
    	#camPose.pose.position.x = 
    	#camPose.pose.position.y = 
        x_middle = (768/2)-(cornerx+cornerdx/2)
        d = 1.0
        try:
        	t = self.trans_listener.getLatestCommonTime("/map", "/base_footprint")
        	(translation, rotation) = self.trans_listener.lookupTransform("/map","/base_footprint",t)
        	th_goal = tf.transformations.euler_from_quaternion(rotation)[2]
        	self.theta_g = th_goal + np.atan2(x_middle,d)
        	self.x_g = translation[0]+np.cos(self.theta_g)
        	self.y_g = translation[1]+np.sin(self.theta_g)
        	self.go_to_pose()
      	except: 
	        #obX_cam = 0.0
	        #oby_cam = 0.0
	        rospy.loginfo("cant follow dog")


    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        if not use_gazebo:
            try:
                origin_frame = "/map" if mapping else "/odom"
                (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        # logs the current mode
        if not(self.last_mode_printed == self.mode):
            rospy.loginfo("EXPLORE: Current Mode: %s", self.mode)
            self.last_mode_printed = self.mode

        # checks wich mode it is in and acts accordingly
        if self.mode == Mode.EXP_IDLE:
            # send zero velocity
            self.stay_idle()

        elif self.mode == Mode.EXP_POSE:
            # moving towards a desired pose
            if self.close_to(self.x_g,self.y_g,self.theta_g, 0.15, 0.3):
                self.mode = Mode.EXP_IDLE
            else:
                self.go_to_pose()

        elif self.mode == Mode.STOP:
            # at a stop sign
            if self.has_stopped():
                self.init_crossing()
            else:
                self.stay_idle()

        elif self.mode == Mode.CROSS:
            # crossing an intersection
            if self.has_crossed():
                if self.explore:
                    self.mode = Mode.EXP_NAV
                else:
                    self.mode = Mode.DEL_NAV_OBJ
            else:
                self.nav_to_pose()

        elif self.mode == Mode.EXP_NAV:
            if self.close_to(self.x_g,self.y_g,self.theta_g, 0.15, 0.3):
                self.mode = Mode.EXP_IDLE
            else:
                self.nav_to_pose()

        elif self.mode == Mode.DEL_IDLE:
            self.stay_idle()

        elif self.mode == Mode.DEL_NAV_OBJ:
            if self.close_to(self.x_g, self.y_g, self.theta_g, 0.2, 1.0):
                self.mode = Mode.DEL_PICKUP
            else:
                self.nav_to_obj(self.obj_idx)

        elif self.mode == Mode.DEL_PICKUP:
            rospy.loginfo("Sleeping")
            time.sleep(5)
            rospy.loginfo("Done sleeping")
            if self.obj_idx == (self.obj_tot-1):
                self.mode = Mode.DEL_NAV_HOME
                self.nav_to_home()
            else:
                self.obj_idx += 1
                self.mode = Mode.DEL_NAV_OBJ
                self.nav_to_obj(self.obj_idx)

        elif self.mode == Mode.DEL_NAV_HOME:
            if self.close_to(self.x_g, self.y_g, self.theta_g, 0.2, 0.5):
                self.mode = Mode.DEL_IDLE
            else:
                self.nav_to_home()

        elif self.mode == Mode.FOLLOW_DOG:
        	if self.explore:
        		self.mode = Mode.EXP_NAV
        	else:
        		self.mode = Mode.DEL_NAV_OBJ
        	self.follow_dog()


        else:
            raise Exception('This mode is not supported: %s'
                % str(self.mode))


    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
#!/usr/bin/env python
import rospy
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
#from turtle_regulation_Norbert_Adrien.srv import setWaypointService, setWaypointServiceRequest
import math
from math import atan2, atan

global waypoint

waypoint = Pose()
waypoint = [7,7]

def pose_callback(data):
	global turlePose
	turtlePose = data

def rotate(pub, rate, kp):
	msg = Twist()
	theta = angle()
	error_angle = error(theta)
	u = kp*error_angle
	msg.angular.z = u
	print("theta:", theta, "| u:",u, "| msg:", msg)

	pub.publish(msg)
	rate.sleep()

def angle():
	return atan2(waypoint[1] - turtlePose.y, waypoint[0] - turtlePose.x)

def error(theta):
	return atan(math.tan((theta - turtlePose.theta)/2))

def main():
	rospy.init_node('set_way_point')
	rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
	global turtlePose
	turtlePose = Pose()
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)
	rate = rospy.Rate(30)
	kp = 1.0
	msg = Twist()
	while not rospy.is_shutdown():
		if turtlePose is not None:
			theta_desired = math.atan2(waypoint[1] - turtlePose.y, waypoint[0]  - turtlePose.x)
			theta = turtlePose.theta
			e = math.atan(math.tan(theta_desired-theta))
			u = kp * e
			print(u)
			msg.angular.z = u
		pub.publish(msg)
		#if turtlePose is not None:
		#rotate(pub, rate, kp)


if __name__ == '__main__':
	main()

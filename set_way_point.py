import rospy
from turtlesim.msg import Pose
from std_msgs import String, Bool
from geometry_msgs.msg import Twist
from turtle_regulation_Norbert_Adrien.srv import setWaypointService, setWaypointServiceRequest
import math

turtle_pose = Pose()

kpl = 1.0

distance_tolerance = 0.2

def change_waypoint(request):
	waypoint = request
	return setWaypointServiceResponse(True)

def pose_callback(data):
	global turtle_pose
	turtle_pose = data

def main():
	rospy.init_node('set_way_point')
	rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
	cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size = 10)

	is_moving = Bool()
	cmd_vel_msg = Twist()

	is_moving.data = True

	moving_pub = rospy.Publisher('is_moving', Bool, queue_size = 1)
	srv = rospy.Service('set_waypoint_srv', setWaypointService, change_waypoint)

	moving_pub.publish(is_moving)

	global waypoint
	waypoint = Pose()
	waypoint.x = 7
	waypoint.y = 7

	err_lin = 0

	while not rospy.is_shutdown():
		if is_moving.data == True:
			theta_desired = math.atan2(waypoint.y - turtle_pose.y, waypoint.x - tutle_pose.x)
			eu_dist = math.sqrt(math.pow(waypoint.y - turtle_pose.y, 2) + math.pow(waypoint))

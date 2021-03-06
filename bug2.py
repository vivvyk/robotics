import rospy
import tf
import math
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
from math import radians, copysign, sqrt, pow, pi
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle

class Bug2():

	def __init__(self):
		global g_ahead_range
		rospy.init_node("bug2", anonymous=False)
		rospy.on_shutdown(self.shutdown)

		self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=5)

		#Update Time
		self.rate = 20
		self.r = rospy.Rate(self.rate)

		#Speeds
		linear_speed = 0.2
		angular_speed = 1.0


		#TF listener
		self.tf_listener = tf.TransformListener()
		rospy.sleep(2) #For tf to fill buffer

		#ODOM frame
		self.odom_frame = '/odom'
		try:
			self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
			self.base_frame = '/base_footprint'
		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
			try:
				self.tf_listener.waitForTransform(self.odom_frame,'/base_link', rospy.Time(), rospy.Duration(1.0))
				self.base_frame = '/base_link'
			except (tf.Exception, tf.ConnectivityException, tf.LookupException):
				rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
				rospy.signal_shutdown("tf Exception")


		#Position
		position = Point()

		#Goals
		#goal_distance = 1.0
		#goal_angle = pi

		#LaserValue
		self.g_ahead_range = None
		self.g_right_range = None
		self.g_left_range = None
		scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)


		#MOTION/ALGORITHM

		move_cmd = Twist()
		driving_forward = True
		hit_points = set()

		while True:
			(position, rotation) = self.get_odom()
			if self.isgoal(position.x):
				print "GOAL REACHED!"
				break

			if driving_forward:
				move_cmd.linear.x = linear_speed
				if self.g_ahead_range != None and self.g_ahead_range < 0.8:
					#if self.check_hitpoint(position, hit_points):
					#	print "HIT POINT REVISITED, NO PATH POSSIBLE."
					#	break
					driving_forward = False

			else:
				print "obstacle detected!"
				self.cmd_vel.publish(Twist())
				self.r.sleep()

				(position, rotation) = self.get_odom()
				print "HIT POINT: " + str(position.x) + ", " + str(position.y)
				hit_points.add(position)

				#Turn LEFT until obstacle is no longer detected.
				self.rotate(30)
				while not math.isnan(self.g_right_range):
					if self.g_right_range > 1:
						break
					self.rotate(15)

				#Follow object
				while True:
					self.translate(0.2) #originally 0.15

					#Turn right until edge
					while math.isnan(self.g_right_range) or self.g_right_range > 0.8:
						self.rotate(-10)
						#self.translate(0.15)

					#Turn incrementally left.
					while self.g_right_range < 0.8:
						self.rotate(25) #otherwise, 15
						#self.translate(0.15)

					(position, rotation) = self.get_odom()
					if self.mline(position.x, position.y):
						print "mline found!"
						driving_forward = True
						rot_deg = -1 * rotation * 180/pi
						self.rotate(rot_deg)
						break

			self.cmd_vel.publish(move_cmd)
			self.r.sleep()

	def get_odom(self):
		try:
			(trans, rot) = self.tf_listener.lookupTransform(self.odom_frame,self.base_frame, rospy.Time(0))
		except (tf.Exception, tf.ConnectivityException, tf.LookupException):
			rospy.loginfo("TF Exception")
			return
		return (Point(*trans), quat_to_angle(Quaternion(*rot)))

	@staticmethod
	def isgoal(x):
		if (9.9 < x < 10.1):
			return True
		return False

	@staticmethod
	def mline(x, y):
		if abs(y) <= 0.1 and (0.1 <= x <= 10.1): #used to be 0.65
			return True
		return False

	def check_hitpoint(self, position, hit_point):
		cur_x = position.x
		cur_y = position.y

		for point in hit_point:
			if abs(point.x - cur_x) > 0.1:
				continue
			if abs(point.y - cur_y) > 0.1:
				continue
			return True

		return False


	def shutdown(self):
		rospy.loginfo("Stopping the robot...")
		self.cmd_vel.publish(Twist())
		rospy.sleep(1)

	def scan_callback(self, msg):
		self.g_left_range = msg.ranges[-1]
		self.g_right_range = msg.ranges[0]
		self.g_ahead_range =  msg.ranges[len(msg.ranges)/2]

	def translate(self, dist):
		goal_distance = dist
		linear_speed = 0.5
		if dist < 0:
			linear_speed *= -1
		linear_duration = goal_distance/linear_speed


		move_cmd = Twist()
		move_cmd.linear.x = linear_speed
		ticks = int(linear_duration * self.rate)
		for t in range(ticks):
			self.cmd_vel.publish(move_cmd)
			self.r.sleep()

		move_cmd = Twist()
		self.cmd_vel.publish(move_cmd)
		rospy.sleep(0.4)

	def rotate(self, deg):
		goal_angle = deg * pi / 180.0
		angular_speed = 0.5
		if deg < 0:
			angular_speed *= -1
		angular_duration = goal_angle / angular_speed

		move_cmd = Twist()
		move_cmd.angular.z = angular_speed

		ticks = int(angular_duration * self.rate)
		for t in range(ticks):
			self.cmd_vel.publish(move_cmd)
			self.r.sleep()

		move_cmd = Twist()
		self.cmd_vel.publish(move_cmd)

		rospy.sleep(0.4)

		self.cmd_vel.publish(Twist())

if __name__ == '__main__':
    try:
        Bug2()
    except:
        rospy.loginfo("Bug2 node terminated.")

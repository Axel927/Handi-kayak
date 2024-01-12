import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
import gps
import numpy as np
import serial

class MyNode(Node):

	"""
	__init__ initialises the global processes and variables
	"""
	def __init__(self):
		super().__init__('Gps_readings')
		self.frequency = 0.1 													#Period between callbacks
		self.publisher_ = self.create_publisher(NavSatFix, 'Gps_readings', 10)
		self.timer_ = self.create_timer(self.frequency, self.timer_callbacks)
		self.get_logger().info('Node initialised')
		#self.ser = serial.Serial('/dev/ttyAMA0',baudrate=9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE)
		

	"""
	ned_to_enu takes a list in the NED (North, East, Down) format and switch it to the ENU (East, North, Up) format
	@param ned a list of coordinates in the NED format
	@return enu a list of coordinates in the ENU format
	"""
	def ned_to_enu(self,ned):
		enu = [0, 0, 0]
		enu[0], enu[1], enu[2] = ned[1], ned[0], -ned[2]
		return enu

	"""
	assign_2_vect takes a numpy array and returns a Vector3
	@param array3 a numpy array of size 3
	@return vect3 an object of type Vector3
	"""
	def assign_2_vect(self, array3: np.array) -> Vector3:

		vect3 = Vector3()

		vect3.x = array3[0]
		vect3.y = array3[1]
		vect3.z = array3[2]

		return vect3

	"""
	assign_2_quat assign Q to a Quaternion
	@return quat an object of type Quaternion
	"""
	def assign_2_quat(self) -> Quaternion:

		quat = Quaternion()

		quat.x = self.Q[0]
		quat.y = self.Q[1]
		quat.z = self.Q[2]
		quat.w = self.Q[3]

		return quat

	"""
	gps_treatement gets the data from the gps, traet it and returns it in the gps format
	@return gps the gps message
	"""
	def gps_treatement(self):

		#Initialise variables
		gps = NavSatFix()

		gps.latitude = 0.0
		gps.longitude = 0.0
		gps.altitude = 0.0

		#Get time

		time_stamp = self.get_clock().now().to_msg()

		gps.header.stamp = time_stamp

		return (gps)
		
	"""
	timer_callbacks takes the gps data and publishes it on the node gps_readings
	"""
	def timer_callbacks(self):
		gps = NavSatFix()
		gps = self.gps_treatement()
		
		self.publisher_.publish(gps)

	

def main(args=None):
	rclpy.init(args=args)
	node = MyNode()
	rclpy.spin(node)
	sensor.use_I2C()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

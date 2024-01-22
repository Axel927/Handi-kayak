import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
import gps
import numpy as np
import serial
import adafruit_gps

class MyNode(Node):

	"""
	__init__ initialises the global processes and variables
	"""
	def __init__(self):
		super().__init__('Gps_readings')
		self.frequency = 1.0													#Period between callbacks
		self.publisher_ = self.create_publisher(NavSatFix, 'Gps_readings', 10)
		self.timer_ = self.create_timer(self.frequency, self.timer_callbacks)
		self.get_logger().info('Node initialised')
		self.ser = serial.Serial('/dev/ttyS0',baudrate=9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE)
		self.gps = adafruit_gps.GPS(self.ser, debug=False)  # Use UART/pyserial
		self.gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
		# Set update rate to once a second (1hz) which is what you typically want.
		self.gps.send_command(b"PMTK220,1000")
		self.gps_position = NavSatFix()

	def __del__(self):
		self.ser.close()
		
	"""
	gps_treatement gets the data from the gps, traet it and returns it in the gps format
	@return gps the gps message
	"""
	def gps_treatement(self):

		self.gps.update()

		if self.gps.has_fix:
			self.gps_position.latitude = self.gps.latitude
			self.gps_position.longitude = self.gps.longitude
		if self.gps.altitude_m:
			self.gps_position.altitude = self.gps.altitude_m
			
		#Get time

		time_stamp = self.get_clock().now().to_msg()

		self.gps_position.header.stamp = time_stamp
		
	"""
	timer_callbacks takes the gps data and publishes it on the node gps_readings
	"""
	def timer_callbacks(self):
		self.gps_treatement()
		
		#print(self.gps_position)

		self.publisher_.publish(self.gps_position)

	

def main(args=None):
	rclpy.init(args=args)
	node = MyNode()
	rclpy.spin(node)
	sensor.use_I2C()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

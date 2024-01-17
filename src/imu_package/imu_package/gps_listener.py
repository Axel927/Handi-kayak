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
		self.gps.send_command(b'PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
		# Set update rate to once a second (1hz) which is what you typically want.
		self.gps.send_command(b"PMTK220,1000")
		self.gps_position = NavSatFix()

	def __del__(self):
		self.ser.close()
		

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

		self.gps.update()

		if self.gps.has_fix:
			self.gps_position.latitude = self.gps.latitude
			self.gps_position.longitude = self.gps.longitude
			self.gps_position.altitude = self.gps.altitude
			
		#Get time

		time_stamp = self.get_clock().now().to_msg()

		self.gps_position.header.stamp = time_stamp
		
	"""
	timer_callbacks takes the gps data and publishes it on the node gps_readings
	"""
	def timer_callbacks(self):
		self.gps_treatement()
		
		print(self.gps_position)

		self.publisher_.publish(self.gps_position)
		

		"""
		self.gps.update()

		print("=" * 40)  # Print a separator line.
		if not self.gps.has_fix:
			# Try again if we don't have a fix yet.
			print("Waiting for fix...")
			return

		print(
			"Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}".format(
				self.gps.timestamp_utc.tm_mon,  # Grab parts of the time from the
				self.gps.timestamp_utc.tm_mday,  # struct_time object that holds
				self.gps.timestamp_utc.tm_year,  # the fix time.  Note you might
				self.gps.timestamp_utc.tm_hour,  # not get all data like year, day,
				self.gps.timestamp_utc.tm_min,  # month!
				self.gps.timestamp_utc.tm_sec,
			)
		)
		print("Latitude: {0:.6f} degrees".format(self.gps.latitude))
		print("Longitude: {0:.6f} degrees".format(self.gps.longitude))
		print(
			"Precise Latitude: {:2.}{:2.4f} degrees".format(
				self.gps.latitude_degrees, self.gps.latitude_minutes
			)
		)
		print(
			"Precise Longitude: {:2.}{:2.4f} degrees".format(
				self.gps.longitude_degrees, self.gps.longitude_minutes
			)
		)
		print("Fix quality: {}".format(self.gps.fix_quality))
		# Some attributes beyond latitude, longitude and timestamp are optional
		# and might not be present.  Check if they're None before trying to use!
		if self.gps.satellites is not None:
			print("# satellites: {}".format(self.gps.satellites))
		if self.gps.altitude_m is not None:
			print("Altitude: {} meters".format(self.gps.altitude_m))
		if self.gps.speed_knots is not None:
			print("Speed: {} knots".format(self.gps.speed_knots))
		if self.gps.track_angle_deg is not None:
			print("Track angle: {} degrees".format(self.gps.track_angle_deg))
		if self.gps.horizontal_dilution is not None:
			print("Horizontal dilution: {}".format(self.gps.horizontal_dilution))
		if self.gps.height_geoid is not None:
			print("Height geoid: {} meters".format(self.gps.height_geoid))

		#print(self.ser.read(10))
		"""

	

def main(args=None):
	rclpy.init(args=args)
	node = MyNode()
	rclpy.spin(node)
	sensor.use_I2C()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

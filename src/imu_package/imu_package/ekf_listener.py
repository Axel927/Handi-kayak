import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
import numpy as np

class MyNode(Node):

	"""
	__init__ initialises the global processes and variables
	"""
	def __init__(self):
		super().__init__('ekf_listener')
		self.frequency = 0.1 													#Period between callbacks
		#self.publisher_ = self.create_publisher(Imu, 'Imu_readings', 10)
		self.subscribtion_ = self.create_subscription(Odometry, 'odometry/filtered', self.callback, 10) 
        #self.timer_ = self.create_timer(self.frequency, self.timer_callbacks)
		self.get_logger().info('Node initialised')
		

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
	imu_treatement gets the data from the IMU, traet it and returns it in the IMU format
	@return imu the IMU message
	"""
	def imu_treatement(self):

		#Initialise variables
		imu = Imu()
		vec3_acc   = Vector3()
		vec3_gyro  = Vector3()

		#Get the IMU data
		acc_gyro_measurement = self.sensor.read_accelerometer_gyro_data() 

		acc_measurement  = self.ned_to_enu( acc_gyro_measurement[0:3] )
		gyro_measurement = self.ned_to_enu( acc_gyro_measurement[3:] )
		mag_measurement  = self.ned_to_enu( self.sensor.read_magnetometer_data() )

		acc_measurement  = [ acc_measurement[0] * 9.81, acc_measurement[1] * 9.81, acc_measurement[2] * 9.81 ]

		#Get time

		time_stamp = self.get_clock().now().to_msg()

		#Assign the imu data to vect3
		vec3_acc  = self.assign_2_vect(acc_measurement)
		vec3_gyro = self.assign_2_vect(gyro_measurement)

		#Determine the quaternarion
		self.Q = self.madgwick.updateMARG(self.Q, gyr = gyro_measurement,acc = acc_measurement,mag = mag_measurement)

		#Assign the value to imu

		imu.linear_acceleration = vec3_acc
		imu.linear_acceleration_covariance = [ 0.01359075, 0.0, 0.0, 0.0, 0.00339754, 0.0, 0.0, 0.0, 0.0077804]

		imu.angular_velocity = vec3_gyro
		imu.angular_velocity_covariance = [ 0.01569848, 0., 0., 0., 0.13585773, 0., 0., 0., 0.07959719]

		imu.orientation = self.assign_2_quat()
		imu.header.stamp = time_stamp

		return (imu)

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
	timer_callbacks takes the imu data and publishes it on the node Imu_readings
	"""
	def timer_callbacks(self):

		self.publisher_.publish(self.imu_treatement())

	def callback(self, msg):
		print(msg.pose)

def main(args=None):
	rclpy.init(args=args)
	node = MyNode()
	rclpy.spin(node)
	sensor.use_I2C()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
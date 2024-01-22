import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32
import numpy as np
from .submodules.kayak_function import KayakFunctions
from tf_transformations import euler_from_quaternion

class MyNode(Node):

	"""
	__init__ initialises the global processes and variables
	"""
	def __init__(self):
		super().__init__('ekf_listener')
		self.frequency = 0.1 													#Period between callbacks
		self.publisher_ = self.create_publisher(Float32, 'buzzer_instruction', 10)
		self.subscribtion_ = self.create_subscription(Odometry, 'odometry/filtered', self.callback, 10) 
		self.timer_ = self.create_timer(self.frequency, self.timer_callbacks)
		self.get_logger().info('Node initialised')
		self.position = Vector3()
		self.euler = Vector3()
		

	def quat_2_euler(self, quat: Quaternion) -> Vector3:
		euler = Vector3()

		orientation_list = [quat.x, quat.y, quat.z, quat.w]

		(roll, pitch, yaw) = euler_from_quaternion(orientation_list)

		euler.x, euler.y, euler.z =roll, pitch, yaw 
		
		return euler


	"""
	timer_callbacks takes the imu data and publishes it on the node Imu_readings
	"""
	def timer_callbacks(self):
		functions = KayakFunctions()
		buzzer_command = Float32()
		buzzer_command.data = functions.getOrder(self.euler.z, self.position.y,  5., 20., 10.)
		self.get_logger().info(f"Received angle: roll {self.euler.x},pitch {self.euler.y},yaw {self.euler.z}")

		self.publisher_.publish(buzzer_command)

	def callback(self, msg):
		quaternion = Quaternion()

		position = msg.pose.pose.position.y
		quaternion = msg.pose.pose.orientation
		#self.get_logger().info(f"Received quaternion: {quaternion}")
		self.euler = self.quat_2_euler(quaternion)

def main(args=None):
	rclpy.init(args=args)
	node = MyNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == '__main__':
	main()
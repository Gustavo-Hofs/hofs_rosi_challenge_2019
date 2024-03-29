#!/usr/bin/env python
import rospy
import numpy as np
from rosi_defy.msg import RosiMovement
from rosi_defy.msg import RosiMovementArray
from std_msgs.msg import Float32MultiArray

class RosiNodeClass():

	# class attributes
	# how to obtain these values? see Mandow et al. COMPLETE THIS REFERENCE
	var_lambda = 0.965
	wheel_radius = 0.1324
	ycir = 0.531

	# class constructor
	def __init__(self):

		# initializing some attributes
		self.omega_left = -0
		self.omega_right = 0
		self.arm_front_rotSpeed = 0
		self.arm_rear_rotSpeed = -0

		# computing the kinematic A matrix
		self.kin_matrix_A = self.compute_kinematicAMatrix(self.var_lambda, self.wheel_radius, self.ycir)

		# sends a message to the user
		rospy.loginfo('Rosi_joy node started')
		rospy.loginfo('Modificado por hofs')

		# registering to publishers
		self.pub_traction = rospy.Publisher('/rosi/command_traction_speed', RosiMovementArray, queue_size=1)
		self.pub_arm = rospy.Publisher('/rosi/command_arms_speed', RosiMovementArray, queue_size=1)

		# registering to subscribers
		self.sub_tracao = rospy.Subscriber('/controlador_tracao', Float32MultiArray, self.callback_tracao)
		self.sub_bracos = rospy.Subscriber('/controlador_bracos', Float32MultiArray, self.callback_bracos)

		# defining the eternal loop frequency
		node_sleep_rate = rospy.Rate(10)

		# eternal loop (until second order)
		while not rospy.is_shutdown():

			arm_command_list = RosiMovementArray()
			traction_command_list = RosiMovementArray()

			# mounting the lists
			for i in range(4):

				# ----- treating the traction commands
				traction_command = RosiMovement()

				# mount traction command list
				traction_command.nodeID = i+1

				# separates each traction side command
				if i < 2:
					traction_command.joint_var = self.omega_right / self.wheel_radius
				else:
					traction_command.joint_var = self.omega_left / self.wheel_radius

				# appending the command to the list
				traction_command_list.movement_array.append(traction_command)

				# ----- treating the arms commands
				arm_command = RosiMovement()

				# mounting arm command list
				arm_command.nodeID = i+1

				# separates each arm side command
				if i == 0 or i == 2:
					arm_command.joint_var = self.arm_front_rotSpeed
				else:
					arm_command.joint_var = self.arm_rear_rotSpeed

				# appending the command to the list
				arm_command_list.movement_array.append(arm_command)

			# publishing
			self.pub_arm.publish(arm_command_list)
			self.pub_traction.publish(traction_command_list)

			# sleeps for a while
			node_sleep_rate.sleep()

	def callback_tracao(self, msg):

		# Recebe velocidade linear e angular
		vel_linear_x = msg.data[0]
		vel_angular_z = msg.data[1]

		# -- computes traction command - kinematic math
		# b matrix
		b = np.array([[vel_linear_x],[vel_angular_z]])

		# finds the joints control
		# x = np.linalg.lstsq(self.kin_matrix_A, b, rcond=-1)[0]
		x = np.dot(np.linalg.inv(self.kin_matrix_A), b)

		# query the sides velocities
		self.omega_right = x[0][0]
		self.omega_left = x[1][0]

	def callback_bracos(self, msg):

		# Recebe velocidade frontal e traseira dos bracos
		vel_angular_frente = msg.data[0]
		vel_angular_tras = msg.data[1]

		# query the velocities
		self.arm_front_rotSpeed = vel_angular_frente
		self.arm_rear_rotSpeed = vel_angular_tras

	# ---- Support Methods --------

	# -- Method for compute the skid-steer A kinematic matrix
	@staticmethod
	def compute_kinematicAMatrix(var_lambda, wheel_radius, ycir):

		# kinematic A matrix
		matrix_A = np.array([[var_lambda*wheel_radius/2, var_lambda*wheel_radius/2],
							[(var_lambda*wheel_radius)/(2*ycir), -(var_lambda*wheel_radius)/(2*ycir)]])

		return matrix_A

# instaciate the node
if __name__ == '__main__':

	# initialize the node
	rospy.init_node('controlador_de_movimento')

	# instantiate the class
	try:
		node_obj = RosiNodeClass()
	except rospy.ROSInterruptException: pass

#!/usr/bin/env python

import rospy
import ASUS.GPIO as GPIO
import math

from geometry_msgs.msg import Twist

class GpioTogglePin():
	def __init__(self, pin):
		self.pin = pin
		self.state = GPIO.LOW
		GPIO.setup(self.pin, GPIO.OUT)
		GPIO.output(self.pin, self.state)

	def set_state(new_state):
		if new_state != self.state:
			self.state = new_state
			GPIO.output(self.pin, self.state)

class SingleMotorController():
	def __init__(self, pwm_pin, n1_pin, n2_pin, tick_pin, m_pr_tick, pwm_speed=100,control_loop_hz=100.0):
		# pins
		self.pwm_pin = pwm_pin
		self.n1_pin = n1_pin
		self.n2_pin = n2_pin
		self.tick_pin = tick_pin
		# options
		self.pwm_speed = pwm_speed
		self.control_loop_hz = control_loop_hz
		self.m_pr_tick = m_pr_tick

		self.p_gain = 1.0
		self.i_gain = 0.0

		# variables
		self.last_tick = 0.0
		self._set_speed = 0.0
		self.speed = 0.0
		self.direction = 0.0
		self.p_reg = 0.0
		self.i_reg = 0.0
		self.last_pwm = 0.0
		self.got_tick = False

		# Pin setup
		self.N1 = GpioTogglePin(self.n1_pin)
		self.N2 = GpioTogglePin(self.n2_pin)
		GPIO.setup(self.pwm_pin,GPIO.OUT)
		GPIO.setup(self.tick_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

		self.pwm = GPIO.PWM(self.pwm_pin,self.pwm_speed)
		self.pwm.start(0.0)

		GPIO.add_event_detect(self.tick_pin, GPIO.FALLING, callback=self.tick_callback,bouncetime=1)

		rospy.Timer(rospy.Duration(1.0/self.control_loop_hz), self.control_loop)
		rospy.Timer(rospy.Duration(0.1), self.speed_resetter)

	def speed_resetter(self,event):
		if not self.got_tick:
			self.speed = 0.0
		self.got_tick = False

	def tick_callback(self,channel):
		new_time = rospy.get_rostime()
		self.got_tick = True
		elapsed = self.new_time - self.last_tick
		hz = (1.0/elapsed.to_sec())/self.ticks_pr_round
		self.speed = self.m_pr_tick*hz
		self.last_tick = new_time

	def limit_i(self):
		pass

	def control_loop(self,event):
		diff = self._set_speed - self.speed
		self.p_reg = diff * self.p_gain
		self.i_reg += diff * self.i_gain

		reg = self.p_reg + self.i_reg
		self.last_pwm = reg
		print "pwm",self.last_pwm
		#self.pwm.ChangeDutyCycle(reg)
		#self.set_direction_pins(reg)

	def set_direction_pins(self,direction):
		if direction == 0.0:
			self.N1.set_state( GPIO.LOW )
			self.N2.set_state( GPIO.LOW )
		if direction > 0.0:
			self.N1.set_state( GPIO.LOW )
			self.N2.set_state( GPIO.HIGH )
		if direction < 0.0:
			self.N1.set_state( GPIO.HIGH )
			self.N2.set_state( GPIO.LOW )
		
	def set_speed(self,speed):
		self._set_speed = speed

class DifferentialDriveMotorController():
	def __init__(self, left_motor, right_motor, wheel_distance):
		self.left_motor = left_motor
		self.right_motor = right_motor
		self.wheel_distance = wheel_distance

		rospy.Subscriber("cmd_vel", Twist, self.got_twist)

	def got_twist(self,msg):
		forward = msg.linear.x
		turn = msg.angular.z
		self.left_motor.set_speed(forward)
                #print forward,turn


class MotorControllerNode():
	def __init__(self):
		rospy.init_node('l298_motorcontroller_node')
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.ASUS)


		wheel_radius = 0.1
		ticks_pr_round = 2.0
		gear = 50.0
		wheel_circumference = 2.0 * math.pi * wheel_radius * gear
		m_pr_tick = wheel_circumference/ticks_pr_round

		left_motor = SingleMotorController( 238, 185, 224, 168, m_pr_tick )
		right_motor = SingleMotorController( 239, 223, 187, 188, m_pr_tick )
		DifferentialDriveMotorController(left_motor, right_motor, 0.6)

nc = MotorControllerNode()
rospy.spin()







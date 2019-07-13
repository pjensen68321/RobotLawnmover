#!/usr/bin/env python

import rospy
import smbus
import ASUS.GPIO as GPIO

bus = smbus.SMBus(1)

inputs = [[0x48,0],[0x48,1],[0x48,2]]

class AdcNode():
	def __init__(self, inputs = [[0x48,0]], rdy_pin = 17 ):
		self.inputs = inputs
		self.rdy_pin = rdy_pin
		self.current_input = 0
		self.last_val = 0

		rospy.init_node('adc_node')
		GPIO.setwarnings(False)
		GPIO.setmode(GPIO.ASUS)
		GPIO.setup(self.rdy_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
		GPIO.add_event_detect(self.rdy_pin, GPIO.FALLING, callback=self.reading_ready,bouncetime=1)
		self.waiting_for_ready = False
		self.get_next_reading()


	def reading_ready(self, channel):
		t = rospy.get_rostime()
		if self.waiting_for_ready:
			print "reading ready"
			adr = self.inputs[self.current_input][0]
			self.last_val = read_word(adr,0x00)
			print "value is",self.last_val

			self.waiting_for_ready = False
			self.get_next_reading()

	def get_next_reading(self):
		ind = self.inputs[self.current_input]
		print "starting conversion on",ind
		adr = ind[0]
		ainp = ind[1]
		c_h = 0b10000011 + (ainp << 4)
		c_l = 0b10000100
		self.waiting_for_ready = True
		bus.write_i2c_block_data(adr, 0x01, [c_h,c_l])

a = AdcNode()
rospy.spin()

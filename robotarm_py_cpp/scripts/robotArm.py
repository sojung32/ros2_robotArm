#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from object_srv.srv import Detection
import time
import RPi.GPIO as GPIO

finger = 11 #GPIO17
wrist = 13 #GPIO27
wristV = 15 #GPIO22   
elbow = 12 #GPIO18
shoulder = 16 #GPIO23
pShoulder = 18 #GPIO24

color = {'red':1, 'blue':2, 'green':3, 'yellow':4}
colors = {1:'red', 2:'blue', 3:'green', 4:'yellow'}

class ServoControl(Node):
	def __init__(self):
		super().__init__('picking_service')
		self.initServo()
		self.servoCamera()
		self.srv = self.create_service(Detection, 'picking', self.motor)
		self.get_logger().info("create service")
	def motor(self, request, response):
		response.result = request.objname
		self.get_logger().info('Detected object : car')
		self.get_logger().info("Robot Arm working...")
		self.servoCamera()
		self.servoPicking(request)
		self.servoSort(request.objname)
		self.servoCamera()
		self.servoFinish()
		
		return response

	def initServo(self):
		GPIO.setmode(GPIO.BOARD)
		GPIO.setwarnings(False)
		GPIO.setup(finger, GPIO.OUT)
		GPIO.setup(wrist, GPIO.OUT)
		GPIO.setup(wristV, GPIO.OUT)
		GPIO.setup(elbow, GPIO.OUT)
		GPIO.setup(shoulder, GPIO.OUT)
		GPIO.setup(pShoulder, GPIO.OUT)
		self.fin = GPIO.PWM(finger, 50)
		self.wri = GPIO.PWM(wrist, 50)
		self.wriV = GPIO.PWM(wristV, 50)
		self.elb = GPIO.PWM(elbow, 50)
		self.shd = GPIO.PWM(shoulder, 50)
		self.pShd = GPIO.PWM(pShoulder, 50)
		self.fin.start(0)
		self.wri.start(0)
		self.wriV.start(0)
		self.elb.start(0)
		self.shd.start(0)
		self.pShd.start(0)
	

	def servoPicking(self, req):
		self.fin.ChangeDutyCycle(4.8) #3 open / 7.5 close / 4.5grab
		#time.sleep(1)
		self.wri.ChangeDutyCycle(1) #1-12(clock) 1 normal
		#time.sleep(1)
		self.wriV.ChangeDutyCycle(req.wristv) #1-7 4 normal
		time.sleep(1)
		self.elb.ChangeDutyCycle(req.elbow) #1-6 / 1 180degree / 6 40degree
		#time.sleep(1)
		self.shd.ChangeDutyCycle(req.shoulder) #2 10degree / 12 180(front)
		#time.sleep(1)
		self.pShd.ChangeDutyCycle(req.pshoulder) #2 back / 12 front
		time.sleep(3)
		

	def servoSort(self, detect):
		if detect==1:
			self.pShd.ChangeDutyCycle(12) #2 back / 12 front
			time.sleep(3)
		elif detect == 2:
			self.pShd.ChangeDutyCycle(7.5) #2 back / 12 front
			time.sleep(3)
		elif detect == 3:
			self.pShd.ChangeDutyCycle(4.5) #2 back / 12 front
			time.sleep(3)
		elif detect == 4:
			self.pShd.ChangeDutyCycle(2) #2 back / 12 front
			time.sleep(3)

		self.wriV.ChangeDutyCycle(2.5) #1-7 4 normal
		self.elb.ChangeDutyCycle(10) #1-6 / 1 180degree / 6 40degree
		self.shd.ChangeDutyCycle(6.5) #2 10degree / 12 180(front)
		self.pShd.ChangeDutyCycle(7) #2 back / 12 front	
		self.fin.ChangeDutyCycle(4.8) #3 open / 7.5 close / 4.5grab
		time.sleep(1)
		
	
	def servoCamera(self):
		self.wriV.ChangeDutyCycle(2.5) #1-7 4 normal
		self.elb.ChangeDutyCycle(10) #1-6 / 1 180degree / 6 40degree
		self.shd.ChangeDutyCycle(6.5) #2 10degree / 12 180(front)
		self.pShd.ChangeDutyCycle(7) #2 back / 12 front
		time.sleep(3)
		
	

	def servoFinish(self):
		
		self.fin.stop()
		self.wri.stop()
		self.wriV.stop()
		self.elb.stop()
		self.shd.stop()
		self.pShd.stop()
		GPIO.cleanup()
		self.get_logger().info("Done")

def main(args=None):
	rclpy.init(args=args)
	servo_control = ServoControl()
	rclpy.spin(servo_control)
	rclpy.shutdown()


if __name__ == '__main__':
	main()

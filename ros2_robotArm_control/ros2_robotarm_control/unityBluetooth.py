#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from color_srv.srv import Detection
#import cv2
import numpy as np
from bluetooth import *
import time

class Unity_bluetooth(Node):
	def __init__(self):
		super().__init__('picking_client')
		self.clnt = self.create_client(Detection, 'picking')
		while not self.clnt.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available')
		self.req = Detection.Request()
		self.data = ""
		self.data_split = []
		
	def bluetooth_conn(self):
		print("bluetooth_conn()")
		server_socket = BluetoothSocket(RFCOMM)
		server_socket.bind(('', 1))
		server_socket.listen(1)

		client_socket, address = server_socket.accept()
		
		self.data = client_socket.recv(1024).decode('ascii')
		self.data_split = self.data.split('/')
		self.req.color = self.data_split[0]
		self.req.pshoulder = int(self.data_split[1])
		self.req.shoulder = int(self.data_split[2])
		self.req.elbow = int(self.data_split[3])
		self.req.wristv = int(self.data_split[4])
		self.req.wrist = int(self.data_split[5])

		self.future = self.clnt.call_async(self.req)
		print("bluetooth_connf")
		time.sleep(2)
		client_socket.close()
		server_socket.close()

		

def main(args=None):

	rclpy.init(args=args)

	unity_bluetooth = Unity_bluetooth()
	unity_bluetooth.bluetooth_conn()
	

	while rclpy.ok():
		rclpy.spin_once(unity_bluetooth)
		if unity_bluetooth.future.done():
			try:
				response = unity_bluetooth.future.result()
			except Exception as e:
				unity_bluetooth.get_logger().info('service fail')
		else:
			unity_bluetooth.get_logger().info('color: %s / x: %d / y: %d' % (unity_bluetooth.req.color, unity_bluetooth.req.wrist, unity_bluetooth.req.shoulder))
		break
	unity_bluetooth.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

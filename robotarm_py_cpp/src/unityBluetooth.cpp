#include <chrono>
#include <cstdlib>
#include <memory>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/rfcomm.h>
#include "color_srv/srv/detection.hpp"
#include "rclcpp/rclcpp.hpp"
#include <typeinfo>
using Detection = color_srv::srv::Detection;

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("picking_client");
	auto client = node->create_client<Detection>("picking");
	auto request = std::make_shared<Detection::Request>();

	struct sockaddr_rc servAddr, clntAddr;
	bdaddr_t tmp = {};                                      
	char msg[1024] = {0};
	int s, clnt;
	socklen_t opt = sizeof(servAddr);
	
	s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
	
	servAddr.rc_family = AF_BLUETOOTH;
	servAddr.rc_bdaddr = tmp;
	servAddr.rc_channel = 1;
	bind(s, (struct sockaddr *)&servAddr, sizeof(servAddr));
	
	listen(s, 1);
	
	clnt = accept(s, (struct sockaddr *)&clntAddr, &opt);
	
	ba2str(&clntAddr.rc_bdaddr, msg);
	memset(msg, 0, sizeof(msg));
	
	recv(clnt, msg, sizeof(msg), 0);
	RCLCPP_INFO(node->get_logger(), "bluetooth connection %c",msg);
	
	request->color = "red";
	request->wrist = 10;
	request->wristv = 20;
	request->elbow = 30;
	request->shoulder = 40;
	request->pshoulder = 50;
	
	while (!client->wait_for_service(std::chrono::seconds(1)))
	{
		if(!rclcpp::ok()){
			RCLCPP_ERROR(node->get_logger(), "client interrupted");
			return 1;
		}
		RCLCPP_INFO(node->get_logger(), "waiting for service to appear");
	}
	auto result_future = client->async_send_request(request);
	if(rclcpp::spin_until_future_complete(node, result_future) != rclcpp::executor::FutureReturnCode::SUCCESS)
	{
		RCLCPP_ERROR(node->get_logger(), "service call failed");
		return 1;
	}
	auto result = result_future.get();
	RCLCPP_INFO(node->get_logger(), "result %d", result->result);
	rclcpp::shutdown();
	return 0;
}

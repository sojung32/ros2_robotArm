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
#include "object_srv/srv/detection.hpp"
#include "rclcpp/rclcpp.hpp"
#include <typeinfo>
#include <string>
#include <vector>
#include <iostream>
#include <sstream>


using Detection = object_srv::srv::Detection;
using namespace std;
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
	RCLCPP_INFO(node->get_logger(), "Bluetooth connection");
	RCLCPP_INFO(node->get_logger(), "Send to service...");
	
	string sendedMsg;
	for(int i=0;i<50;i++) sendedMsg+=msg[i];
	
	char splitSymbol='/';
	stringstream f(sendedMsg);
	vector<string> split;
	string angleMsg;
	
	while(getline(f,angleMsg,splitSymbol)){
		split.push_back(angleMsg);
	}
	float angles[5] = {0};
	int i=0;
	string str;
	for(vector<string>::iterator it=split.begin();it!=split.end();++it){
		str = *it;
		angles[i++] = atof(str.c_str());
	}
	
	request->objname = angles[0];
	request->wristv = angles[4];
	request->elbow = angles[3];
	request->shoulder = angles[2];
	request->pshoulder = angles[1];
	
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
	RCLCPP_INFO(node->get_logger(), "Sent successfully");
	rclcpp::shutdown();
	return 0;
}

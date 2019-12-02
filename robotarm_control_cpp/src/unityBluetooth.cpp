#include <chrono>
#include <cstdlib>
#include <memory>
#include "color_srv/srv/detection.hpp"
#include "rclcpp/rclcpp.hpp"

using Detection = color_srv::srv::Detection;

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("picking_client");
	auto client = node->create_client<Detection>("picking");
	auto request = std::make_shared<Detection::Request>();
	request->color = "red";
	request->wrist = 10;
	request->wristv = 20;
	request->elbow = 30;
	request->shoulder = 40;
	request->pshoulder = 50;
	while (!client->wait_for_service(std::chrono::seconds(1)))
	{
		if(!rclcpp::ok()){
			RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
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

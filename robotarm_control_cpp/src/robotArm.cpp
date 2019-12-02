#include <iostream>
#include <wiringPi.h>
#include <memory>
#include "color_srv/srv/detection.hpp"
#include "rclcpp/rclcpp.hpp"

using Detection = color_srv::srv::Detection;
rclcpp::Node::SharedPtr servoControl = nullptr;

int finger = 11, wrist = 13, wristV = 15;
int elbow = 12, shoulder = 16, pShoulder = 18;
int angle;
std::string color[4] = {"red","blue","green","yellow"};	



static void motor(const std::shared_ptr<Detection::Request> request,
		  const std::shared_ptr<Detection::Response> response)
{
	RCLCPP_INFO(servoControl->get_logger(),"Detect: %d", request->wrist, request->elbow);
	response->result = request->wrist;
}

void initServo()
{
	wiringPiSetup();
	pinMode(finger, PWM_OUTPUT);
	pinMode(wrist, PWM_OUTPUT);
	pinMode(wristV, PWM_OUTPUT);
	pinMode(elbow, PWM_OUTPUT);
	pinMode(shoulder, PWM_OUTPUT);
	pinMode(pShoulder, PWM_OUTPUT);
}

void servoPicking(int wrist/*, int wristV, int elbow, int shoulder, int pShoulder*/)
{
	for(angle=0;angle<wrist;++angle)
	{
		pwmWrite(wrist, angle);
		delay(1);
	}
	delay(3000);
}

void servoSort(int detect)
{
	if (detect==1)
	{
		for(angle=0;angle<30;++angle)
		{
			pwmWrite(pShoulder, angle);
			delay(1);
		}
		delay(3000);
	}
}

void servoCamera()
{
	for(angle=0;angle<30;++angle)
	{
		pwmWrite(elbow, angle);
		delay(1);
	}
	delay(3000);
}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	servoControl = rclcpp::Node::make_shared("picking_service");

	initServo();

	auto server = servoControl->create_service<Detection>("picking", &motor);

	RCLCPP_INFO(servoControl->get_logger(),"create service");
	
	rclcpp::spin(servoControl);
	rclcpp::shutdown();
	servoControl = nullptr;
	return 0;
}

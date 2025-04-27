#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class Control
{
public:
	virtual void setLaserData(const std::vector<float>& data) = 0;
	virtual void setRobotPose(double x, double y, double theta) = 0;
	virtual void getControl(double& v, double& w) = 0;

	virtual std::string getName() = 0;

	virtual ~Control() = default;
};
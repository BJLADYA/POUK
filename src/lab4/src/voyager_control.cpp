#include <lab4/voyager_control.hpp>

void VoyagerControl::setLaserData(const std::vector<float> &data)
{
	obstacle = false;
	for (size_t i = 0; i < data.size(); i++)
	{
		if (data[i] < min_range)
		{
			obstacle = true;
			// RCLCPP_WARN(logger_, "OBSTACLE!!!");
			break;
		}
	}
}

void VoyagerControl::getControl(double &v, double &w)
{
	if (obstacle) {
		v = 0;
		w = max_omega;
	}
	else {
		v = max_vel;
		w = 0;
	}
}
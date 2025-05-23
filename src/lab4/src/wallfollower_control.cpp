#include <lab4/wallfollower_control.hpp>

void WallfollowerControl::setLaserData(const std::vector<float> &data)
{
	obstacle = false;
    right_distance = data[data.size() / 4]; // расстояние справа от робота
	for (size_t i = 0; i < data.size(); i++)
	{
		if (data[i] < wall_offset)
		{
			obstacle = true;
			RCLCPP_WARN(logger_, "OBSTACLE!!!");
			break;
		}
	}
}

void WallfollowerControl::getControl(double &v, double &w)
{
    if (obstacle) {
        v = 0;
        w = 0.5;
    }
    else {
        if (right_distance == wall_offset){
            v = 1;
            w = 0;
        }
        else{
            v = 0.7;
            w = wall_offset - right_distance;
        }
    }
	RCLCPP_INFO(logger_, "v: %f", v);
	RCLCPP_INFO(logger_, "w: %f", w);
}

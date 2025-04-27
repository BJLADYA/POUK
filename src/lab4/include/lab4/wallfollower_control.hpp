#pragma once

#include <rclcpp/rclcpp.hpp>

#include <lab4/control.hpp>

class WallfollowerControl final : public Control
{
private:
	bool obstacle;
	double wall_offset;
	rclcpp::Logger logger_;

public:
	// установка данных лазера
	void setLaserData(const std::vector<float> &data) override;

	// установка текущей позиции робота - для данного вида управления не требуется - поэтому пустая
	void setRobotPose(double x, double y, double theta) override {}

	// получение управления
	void getControl(double &v, double &w) override;

	std::string getName() override { return "WallFollower"; }

	WallfollowerControl(rclcpp::Logger logger, double wall_offset) : 
		logger_(logger),
		obstacle(false),
		wall_offset(wall_offset)
	{}
};
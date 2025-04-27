#pragma once 

#include <rclcpp/rclcpp.hpp>

#include <lab4/control.hpp>

class DummyControl final : public Control
{
public:
    //установка данных лазера
    void setLaserData(const std::vector<float>& data) override;

    //установка текущей позиции робота - для данного вида управления не требуется - поэтому пустая
    void setRobotPose(double x, double y, double theta) override {}

    //получение управления
    void getControl(double& v, double& w) override;

    std::string getName() override { return "Dummy"; }

	DummyControl(rclcpp::Logger logger) :
		logger_(logger)
	{}
	
private:
	rclcpp::Logger logger_;
    bool obstacle = false;
};
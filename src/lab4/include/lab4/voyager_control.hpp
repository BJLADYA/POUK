#pragma once

#include <rclcpp/rclcpp.hpp>

#include <lab4/control.hpp>

class VoyagerControl final : public Control
{
private:
    double min_range;
    bool obstacle = false;
    double max_vel;
    double max_omega;

	rclcpp::Logger logger_;

public:
    //установка данных лазера
    void setLaserData(const std::vector<float>& data) override;

    //установка текущей позиции робота - для данного вида управления не требуется - поэтому пустая
    void setRobotPose(double x, double y, double theta) override {}

    //получение управления
    void getControl(double& v, double& w) override;

    std::string getName() override { return "Voyager"; }

    VoyagerControl(rclcpp::Logger logger, double range = 1.0, double maxv = 0.5, double maxw = 0.5):
		logger_(logger),
        min_range(range),
        max_vel(maxv),
        max_omega(maxw)
    {}
};
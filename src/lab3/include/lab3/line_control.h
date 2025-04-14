#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <math.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class LineControl final : public rclcpp::Node
{
public:
    LineControl();
// секция приватных функций
private:
    //функция вычисления ошибки управления для движения вдоль прямой
    double cross_track_err_line();

    //функция вычисления ошибки управления для движения вдоль окружности
    double cross_track_err_circle();

    //функция вычисления ошибки управления для движения вдоль овала
    double cross_track_err_oval();

    /**
     * Функция, которая будет вызвана
     * при получении данных от лазерного дальномера
     */
    void laserCallback(const sensor_msgs::msg::LaserScan& msg);

    /**
     * Функция, которая будет вызвана при
     * получении сообщения с текущем положением робота
     */
    void poseCallback(const nav_msgs::msg::Odometry& msg);

    /**
     * функция обработчик таймера
     */
    void timerCallback();

    // функция публикации ошибки
    void publish_error(double e);

// секция приватных членов
 private:
    std::string figure;
    // заданная координата линии, вдоль которой должен двигаться робот
    double line_y;
    double cx, cy, R;
    // заданная скорость движения
    double task_vel;
    // пропрциональный коэффициент регулятора обратной связи
    double prop_factor;
    // интегральный коэффициент регулятора
    double int_factor;
    // дифференциальный коэффициент регулятора
    double diff_factor;
    // интеграл ошибки
    double int_error;
    // старое значение ошибки
    double old_error;
    // минимально допустимое значение расстояния до препятствия
    double min_obstacle_range;
    // флаг наличия препятствия
    bool obstacle;
    // положение робота
    double x, y, theta;

    // объекты, обеспечивающие связь с ros - должны существовать все время жизни приложения
    // публикатор команд управления
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;

    //  публикатор текущей ошибки управления
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr err_pub;

    // подписчики
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub;
    rclcpp::TimerBase::SharedPtr timer_;

    int count_;
};
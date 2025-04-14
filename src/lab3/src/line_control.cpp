#include <lab3/line_control.h>

LineControl::LineControl() 
:   Node("line_control"), 
    count_(0), 
    int_error(0.0),
    old_error(0.0),
    obstacle(false) 
{
    RCLCPP_INFO(this->get_logger(), "LineControl initialisation");
    figure = this->declare_parameter("figure", "line");
    line_y = this->declare_parameter("line_y", -10.0);
    cx = this->declare_parameter("cx", -6);
    cy = this->declare_parameter("cy", 0);
    R = this->declare_parameter("R", 6);
    task_vel = this->declare_parameter("task_vel", 1.0);
    prop_factor = this->declare_parameter("prop_factor", 0.1);
    int_factor = this->declare_parameter("int_factor", 0.0);
    diff_factor = this->declare_parameter("diff_factor", 0.0);
    min_obstacle_range = this->declare_parameter("min_obstacle_range", 1.0);
    double dt = this->declare_parameter("dt", 0.1);

    laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&LineControl::laserCallback, this, _1));
    pose_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&LineControl::poseCallback, this, _1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&LineControl::timerCallback, this));
    cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    err_pub = this->create_publisher<std_msgs::msg::Float64> ("/err", 10);
}

double LineControl::cross_track_err_line()
{
    return line_y - y;
}

double LineControl::cross_track_err_circle()
{
    double dx = cx - x;
    double dy = cy - y;
    double e = sqrt(dx*dx + dy*dy) - R;
    return  e;
}

double LineControl::cross_track_err_oval()
{
    return  0.0;
}

void LineControl::laserCallback(const sensor_msgs::msg::LaserScan &msg)
{
    // проверим нет ли вблизи робота препятствия
    const double kMinObstacleDistance = 0.3;
    for (size_t i = 0; i<msg.ranges.size(); i++)
    {
        if ( msg.ranges[i] < kMinObstacleDistance )
        {
            obstacle = true;
            RCLCPP_WARN(this->get_logger(), "OBSTACLE!!!");
            break;
        }
    }
}

void LineControl::poseCallback(const nav_msgs::msg::Odometry &msg)
{
    RCLCPP_INFO(this->get_logger(), "Pose callback");
    //     " y = "<<msg.pose.pose.position.y<<
    //     " theta = "<<2*atan2(msg.pose.pose.orientation.z,
    //             msg.pose.pose.orientation.w) );
    // обновляем переменные класса, отвечающие за положение робота
    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    theta = 2*atan2(msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w);
}

void LineControl::timerCallback()
{
    RCLCPP_INFO(this->get_logger(), "on timer ");
    // сообщение с помощью которого задается
    // управление угловой и линейной скоростью
    geometry_msgs::msg::Twist cmd;
    // при создании структура сообщения заполнена нулевыми значениями

    // если вблизи нет препятствия то задаем команды
    if ( !obstacle )
    {
        double err = 0;
        //  вычислим текущую ошибку управления
        if (figure == "line") {
            err = cross_track_err_line();
        }

        if (figure == "circle") {
            err = cross_track_err_circle();
        }

        if (figure == "oval") {
            err = cross_track_err_oval();
        }
        
        //  публикация текущей ошибки
        publish_error(err);
        //  интегрируем ошибку
        int_error += err;
        //  диффференцируем ошибку
        double diff_error = err - old_error;
        //   запоминаем значение ошибки для следующего момента времени
        old_error = err;
        cmd.linear.x = task_vel;
        //  ПИД регулятор угловой скорости w = k*err + k_и * инт_err + k_д * дифф_err
        cmd.angular.z = prop_factor * err + int_factor*int_error + diff_error * diff_factor;
        // ROS_DEBUG_STREAM("error = "<<err<<" cmd v="<<cmd.linear.x<<" w = "<<cmd.angular.z);
    }
    //  отправляем (публикуем) команду
    cmd_pub->publish(cmd);
}

void LineControl::publish_error(double e)
{
    std_msgs::msg::Float64 err;
    err.data = e;
    err_pub->publish(err);
}

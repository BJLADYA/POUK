#include <lab5/mapper.h>

Mapper::Mapper() : Node("mapper_node")
{
	map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/simple_map", 10);
	laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"/base_scan",
		10,
		[this](const sensor_msgs::msg::LaserScan msg) {
			this->laserCallback(msg);
		});

	map_frame_ = this->declare_parameter<std::string>("map_frame", "odom");
	map_resolution_ = this->declare_parameter<double>("map_resolution", 0.1);
	map_width_ = this->declare_parameter<int>("map_width", 100);
	map_height_ = this->declare_parameter<int>("map_height", 100);

	tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void Mapper::laserCallback(const sensor_msgs::msg::LaserScan& scan)
{
    geometry_msgs::msg::TransformStamped scanTransform;
    const std::string& laser_frame = scan.header.frame_id;
    const rclcpp::Time& laser_stamp = scan.header.stamp;

    if (!determineScanTransform(scanTransform, laser_stamp, laser_frame)) {
        return;
    }

    nav_msgs::msg::OccupancyGrid map_msg;
    prepareMapMessage(map_msg, laser_stamp);

    // положение центра дальномера в СК дальномера
    geometry_msgs::msg::Point zero_pose;
	zero_pose.x = 0;
	zero_pose.y = 0;
	zero_pose.z = 0;

	tf2::Quaternion lidar_quat;
    tf2::fromMsg(scanTransform.transform.rotation, lidar_quat);

    // положение дальномера в СК карты
	geometry_msgs::msg::Point scan_pose;
	tf2::doTransform(zero_pose, scan_pose, scanTransform);

    //задаем начало карты так, чтобы сканнер находился в центре карты
    map_msg.info.origin.position.x = scan_pose.x - map_width_ * map_resolution_ / 2.0;
    map_msg.info.origin.position.y = scan_pose.y - map_height_ * map_resolution_ / 2.0;

    //индексы карты, соответствующие положению центра лазера
    int y = (scan_pose.y - map_msg.info.origin.position.y ) / map_resolution_;
    int x = (scan_pose.x - map_msg.info.origin.position.x ) / map_resolution_;

    // В клетку карты соотвтествующую центру лазера - записываем значение 0
    map_msg.data[y * map_width_ + x] = 0;

	for (size_t i = 0; i < scan.ranges.size(); ++i) {
        if (std::isinf(scan.ranges[i]) || std::isnan(scan.ranges[i])) {
            continue; // Игнорируем бесконечные и NaN значения
        }
		
        // Угол луча в СК робота
        float angle = scan.angle_min + i * scan.angle_increment;

        // Координаты точки в СК робота
        float local_x = scan.ranges[i] * cos(angle);
        float local_y = scan.ranges[i] * sin(angle);

		tf2::Vector3 ray_global = tf2::quatRotate(lidar_quat, tf2::Vector3(local_x, local_y, 0));

        // Преобразование в СК карты
        int map_x = static_cast<int>(x + ray_global.x() / map_resolution_);
        int map_y = static_cast<int>(y + ray_global.y() / map_resolution_);

		// Закрашиваем линию от робота до точки
        DrawLine(map_msg, x, y, map_x, map_y, 0);

        // Проверка границ карты
        if (map_x >= 0 && map_x < map_width_ && map_y >= 0 && map_y < map_height_ && scan.ranges[i] != 5.0) {
            map_msg.data[map_y * map_width_ + map_x] = 100; // 100 = занято
        }
    }

    // публикуем сообщение с построенной картой
    map_pub_->publish(map_msg);
}

void Mapper::prepareMapMessage(nav_msgs::msg::OccupancyGrid &map_msg, const rclcpp::Time &stamp)
{
	map_msg.header.frame_id = map_frame_;
	map_msg.header.stamp = stamp;

	map_msg.info.height = map_height_;
	map_msg.info.width = map_width_;
	map_msg.info.resolution = map_resolution_;

	map_msg.data.resize(map_height_ * map_width_, -1);
}

void Mapper::DrawLine(
    nav_msgs::msg::OccupancyGrid& map,
    float x0, float y0, float x1, float y1,
    int8_t value)
{
    int x0_cell = static_cast<int>(x0);
    int y0_cell = static_cast<int>(y0);
    int x1_cell = static_cast<int>(x1);
    int y1_cell = static_cast<int>(y1);

    int dx = abs(x1_cell - x0_cell);
    int dy = -abs(y1_cell - y0_cell);
    int sx = x0_cell < x1_cell ? 1 : -1;
    int sy = y0_cell < y1_cell ? 1 : -1;
    int err = dx + dy;

    while (true) {
        if (x0_cell >= 0 && x0_cell < map.info.width && 
            y0_cell >= 0 && y0_cell < map.info.height) {
            map.data[y0_cell * map_width_ + x0_cell] = value;
        }

        if (x0_cell == x1_cell && y0_cell == y1_cell) break;
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0_cell += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0_cell += sy;
        }
    }
}

bool Mapper::determineScanTransform(geometry_msgs::msg::TransformStamped &scanTransform,
									const rclcpp::Time &stamp,
									const std::string &laser_frame)
{
	try
	{
		scanTransform = tf_buffer_->lookupTransform(
			map_frame_,
			laser_frame,
			stamp,
			rclcpp::Duration::from_seconds(0.1)); // timeout
		return true;
	}
	catch (tf2::TransformException &e)
	{
		RCLCPP_WARN(this->get_logger(), "Failed to get transform: %s", e.what());
		return false;
	}
}
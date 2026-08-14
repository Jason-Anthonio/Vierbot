#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"

class SLLidarNode : public rclcpp::Node
{
public:
  SLLidarNode() : Node("sllidar_node")
  {
    this->declare_parameter("serial_port", "/dev/rplidar");
    this->declare_parameter("serial_baudrate", 115200);
    this->declare_parameter("frame_id", "laser_frame");
    this->declare_parameter("inverted", false);
    this->declare_parameter("angle_compensate", true);
    this->declare_parameter("scan_mode", "Sensitivity");

    laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    RCLCPP_INFO(this->get_logger(), "SLLidar node initialized on port: %s", 
      this->get_parameter("serial_port").as_string().c_str());
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SLLidarNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

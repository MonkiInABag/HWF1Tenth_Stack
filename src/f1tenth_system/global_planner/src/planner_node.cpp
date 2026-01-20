#include <memory> //for message handling

#include "rclcpp/rclcpp.hpp" //core ROS2 Api
#include "sensor_msgs/msg/laser_scan.hpp" //what /scan publishes
#include "nav_msgs/msg/odometry.hpp" //what /odom publishes
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp" //what we publish to /drive

class GlobalPlannerNode : public rclcpp::Node 
{
public:
  GlobalPlannerNode() : Node("global_planner_node") 
  {

    this->declare_parameter<double>("default_speed", 0.0);
    this->declare_parameter<double>("max_steering_angle", 0.4);

   
    default_speed_ = this->get_parameter("default_speed").as_double();
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr m){ odom_ = m; step(); });

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, [this](sensor_msgs::msg::LaserScan::SharedPtr m){ scan_ = m; step(); });

    pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
  }

private:
  void step() 
  {
    if (!odom_ || !scan_) return;        // wait until we have both

    // --- YOUR PLANNER GOES HERE ---
    auto cmd = ackermann_msgs::msg::AckermannDriveStamped();
    cmd.header.stamp = now();
    cmd.drive.steering_angle = 0.0;
    cmd.drive.speed = 0.0;              // safe default
    pub_->publish(cmd);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_;

  nav_msgs::msg::Odometry::SharedPtr odom_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;

  double default_speed_{0.0};
  double max_steering_angle_{0.4};

};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}

#include <memory>
#include <vector>

#include "global_planner/tta_planner.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

//class definition
class GlobalPlannerNode : public rclcpp::Node
{
public:
  //contructor
  GlobalPlannerNode()
  : Node("global_planner_node")
  {
    //parameters
    this->declare_parameter<double>("default_speed", 0.0);
    this->declare_parameter<double>("max_steering_angle", 0.4);
    this->declare_parameter<std::string>("frame_id", "map");

    default_speed_ = this->get_parameter("default_speed").as_double();
    max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();

    //subscriptions and publishers
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](nav_msgs::msg::Odometry::SharedPtr m) {odom_ = m; step();}
    );

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      [this](sensor_msgs::msg::LaserScan::SharedPtr m) {scan_ = m; step();}
    );

    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);


    //path publisher with latching
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.transient_local();   // latch last message
    qos.reliable();

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/global_centerline", qos
    );

    //timer for step function
    timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&GlobalPlannerNode::step, this)
    );

  }

private:
  //main step function
  void step()
  {
    //RCLCPP_WARN(get_logger(), "STEP CALLED");

    //check for necessary data from subscriptions
    if (!odom_ || !scan_) {
      return;
    }

    //temp walls
    std::vector<BoundaryPoint> left = {
      {0.0, 0.0},
      {0.0, 2.0},
      {0.0, 4.0}
    };

    std::vector<BoundaryPoint> right = {
      {2.0, 0.0},
      {2.0, 2.0},
      {2.0, 4.0}
    };

    std::vector<BoundaryPoint> centerline;

    //calls planner to compute centerline
    if (planner_.computeCenterline(left, right, centerline)) {
      //convert centerline to path message and publish
      nav_msgs::msg::Path path_msg;
      path_msg.header.stamp = now();
      path_msg.header.frame_id = frame_id_;

      for (const auto & pt : centerline) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now();
        pose.header.frame_id = frame_id_;
        pose.pose.position.x = pt.x;
        pose.pose.position.y = pt.y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
      }
      path_pub_->publish(path_msg);

      RCLCPP_INFO(
        get_logger(),
        "Published global centerline with %zu points (first = %.2f, %.2f)",
        centerline.size(), centerline[0].x, centerline[0].y
      );
    }

    ackermann_msgs::msg::AckermannDriveStamped cmd;
    cmd.header.stamp = now();
    //placeholder to stop car doing anything
    cmd.drive.steering_angle = 0.0;
    cmd.drive.speed = 0.0;
    drive_pub_->publish(cmd);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry::SharedPtr odom_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;


  TTAPlanner planner_;

  double default_speed_{0.0};
  double max_steering_angle_{0.4};
  std::string frame_id_{"map"};

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}

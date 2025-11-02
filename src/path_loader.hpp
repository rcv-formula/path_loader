#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <fstream>
#include <string>
#include <utility>
#include <vector>
#include "nav_msgs/msg/path.hpp"

struct pathInformation {
  double x;
  double y;
  double v;
  double s;
  double d;
  double distance;
  double left_void;
  double right_void;
};

class CordinateConverter {
public:
  CordinateConverter(rclcpp::Node::SharedPtr node, bool node_mode,
                     std::string config_file_path, const std::string path_type);
  ~CordinateConverter();

  std::vector<std::vector<double>> getGlobalPath();

private:

  void readPath(std::string path_file_path);

  double calcPathToPathHeading(int idx);

  void path_publisher();

  void path_msg_generator();

  bool node_mode_;

  bool path_recives;

  std::vector<std::vector<double>> global_path_;
  std::vector<pathInformation> path_;

  rclcpp::Node::SharedPtr node_;
  nav_msgs::msg::Path::SharedPtr global_path_msg_;
  nav_msgs::msg::Path::SharedPtr global_path_vis_msg_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vis_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

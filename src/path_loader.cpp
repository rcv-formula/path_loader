#include "path_loader.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

CordinateConverter::CordinateConverter(rclcpp::Node::SharedPtr node,
                                       bool node_mode = false,
                                       std::string config_file_path = "",
                                       const std::string path_type = "ring") {
  node_ = node;
  node_mode_ = node_mode;

  if (node_mode_) {
    if (config_file_path == "") {
      config_file_path =
          ament_index_cpp::get_package_share_directory("path_loader") +
          "/config/config.yaml";
    }
    std::string path_file_path, pgm_file_path, map_yaml_file_path;

    std::ifstream file(config_file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(node->get_logger(), "cannot open path file : %s",
                   config_file_path.c_str());
    } else {
      std::string yaml_line;
      while (std::getline(file, yaml_line)) {
        std::istringstream ss(yaml_line);
        std::string key;
        ss >> key;
        if (key == "path:") {
          ss >> path_file_path;}
        else if( key == "name:"){
            std::string name;
            ss >> name;
            path_file_path = ament_index_cpp::get_package_share_directory("path_loader") + "/path/" +name +".csv";
        }
      }

      readPath(path_file_path);
      path_msg_generator();

      file.close();

      rclcpp::QoS qos(rclcpp::KeepLast(1));
      qos.transient_local();
      qos.reliable();

      publisher_ =
          node_->create_publisher<nav_msgs::msg::Path>("global_path", qos);
      timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this]() {
        path_publisher();
        timer_->cancel();
      });
    }
  }
}

CordinateConverter::~CordinateConverter() {
}

void CordinateConverter::readPath(std::string path_file_path) {
  std::ifstream path_file(path_file_path);
  std::string line_temp;
  while (getline(path_file, line_temp)) {
    std::stringstream ss(line_temp);
    std::string cell;
    std::vector<double> row;
    int col_counter = 0;
    while (getline(ss, cell, ',') && col_counter < 3) {
      row.push_back(stod(cell));
    }
    global_path_.push_back(row);
  }

  RCLCPP_INFO(node_->get_logger(),
              "successfully load csv path file, %ld points from %s",
              global_path_.size(), path_file_path.c_str());

}

double CordinateConverter::calcPathToPathHeading(int idx) {
  idx = idx % global_path_.size();
  int next_idx = (idx + 1) % global_path_.size();

  double cur_x = global_path_[idx][0], cur_y = global_path_[idx][1];
  double next_x = global_path_[next_idx][0], next_y = global_path_[next_idx][1];

  double dx = next_x - cur_x, dy = next_y - cur_y;
  double theta = atan2(dy, dx);
  return theta;
}

void CordinateConverter::path_publisher() {
  publisher_->publish(*global_path_msg_);
  RCLCPP_INFO(node_->get_logger(), "Published Path with %zu points",
              global_path_msg_->poses.size());
}

void CordinateConverter::path_msg_generator() {
  global_path_msg_ = std::make_shared<nav_msgs::msg::Path>();
  global_path_msg_->header.stamp = node_->get_clock()->now();
  global_path_msg_->header.frame_id = "map";

  for (int i = 0; i < global_path_.size(); i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = node_->get_clock()->now();
    pose.header.frame_id = "map";
    pose.pose.position.x = global_path_[i][0];
    pose.pose.position.y = global_path_[i][1];
    pose.pose.position.z = global_path_[i][2];
    pose.pose.orientation.w = 1.0; // 단순한 예제이므로 회전 없음
    global_path_msg_->poses.push_back(pose);
  }
  global_path_msg_;
}

std::vector<std::vector<double>> CordinateConverter::getGlobalPath() {
  return global_path_;
}



int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("path_loader");
  CordinateConverter c(node, true);
  rclcpp::spin(node);
}
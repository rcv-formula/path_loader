cmake_minimum_required(VERSION 3.8)
project(path_loader)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 필요한 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  "${PROJECT_SOURCE_DIR}/src"
)

add_executable(path_loader src/path_loader.cpp)

ament_target_dependencies(path_loader rclcpp std_msgs nav_msgs geometry_msgs)

#target_link_libraries(rf_publisher_node ${LIBSERIAL_LIB})

install(TARGETS
  path_loader
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY path/
  DESTINATION share/${PROJECT_NAME}/path
)


#install(DIRECTORY launch/
#  DESTINATION share/${PROJECT_NAME}/launch
#)
ament_export_include_directories(include)
ament_package()

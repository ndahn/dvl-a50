cmake_minimum_required(VERSION 3.5)
project(dvl_a50)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(marine_acoustic_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)


include_directories(include
  ${rclcpp_INCLUDE_DIRS}
)


# Library
add_library(${PROJECT_NAME} SHARED src/dvl_a50.cpp)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

INSTALL(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


# ROS2 Node
ADD_EXECUTABLE(${PROJECT_NAME}_node 
    src/dvl_a50_ros2.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp
  rclcpp_lifecycle
  std_msgs
  geometry_msgs
  marine_acoustic_msgs
  tf2_geometry_msgs
)
# Link node to library compiled above and install it in the correct location
TARGET_LINK_LIBRARIES(${PROJECT_NAME}_node ${PROJECT_NAME})
INSTALL(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}/
)


# Launch and config files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()

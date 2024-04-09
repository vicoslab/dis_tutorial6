cmake_minimum_required(VERSION 3.5)
project(dis_tutorial6)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

set(dependencies rclcpp std_msgs geometry_msgs sensor_msgs visualization_msgs tf2 tf2_geometry_msgs pcl_conversions)

add_executable(pcl_forwarder src/forwarder.cpp)
ament_target_dependencies(pcl_forwarder ${dependencies})

add_executable(planes src/planes.cpp)
ament_target_dependencies(planes ${dependencies})

add_executable(cylinder_segmentation src/cylinder_segmentation.cpp)
ament_target_dependencies(cylinder_segmentation ${dependencies})


target_link_libraries(pcl_forwarder
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

target_link_libraries(planes
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

target_link_libraries(cylinder_segmentation
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

install(TARGETS
  pcl_forwarder
  planes
  cylinder_segmentation
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/detect_rings.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
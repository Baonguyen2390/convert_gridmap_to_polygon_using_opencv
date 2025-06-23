cmake_minimum_required(VERSION 3.8) 
project(gridmap_to_polygon)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang") 
  add_compile_options(-Wall -Wextra -Wpedantic) 
endif()


find_package(ament_cmake REQUIRED) 
find_package(rclcpp REQUIRED) 
find_package(nav_msgs REQUIRED) 
find_package(geometry_msgs REQUIRED) 
find_package(OpenCV REQUIRED) 
find_package(yaml-cpp REQUIRED)

include_directories( ${rclcpp_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS} )


add_executable(gridmap_to_polygon gridmap_to_polygon.cpp)


ament_target_dependencies(gridmap_to_polygon rclcpp nav_msgs geometry_msgs OpenCV ) 
target_link_libraries(gridmap_to_polygon ${YAML_CPP_LIBRARIES} )


install(TARGETS gridmap_to_polygon DESTINATION lib/${PROJECT_NAME} )


install(DIRECTORY config maps DESTINATION share/${PROJECT_NAME}/ )

if(BUILD_TESTING) 
  find_package(ament_lint_auto REQUIRED) 
  ament_lint_auto_find_test_dependencies() 
endif()

ament_package()
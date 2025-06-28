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
find_package(gridmap_to_polygon_msg REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(
  include
)

add_executable(standalone_node standalone_node.cpp)

add_executable(action_server action_server.cpp)
ament_target_dependencies(action_server
  gridmap_to_polygon_msg
  rclcpp
  rclcpp_action
)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
target_link_libraries(action_server
  gridmap_to_polygon
)

set(dependencies
  ament_cmake
  rclcpp
  nav_msgs
  geometry_msgs
  OpenCV
  yaml-cpp
)

add_library(gridmap_to_polygon SHARED
  gridmap_to_polygon.cpp
)

ament_target_dependencies(gridmap_to_polygon 
  ${dependencies}
)

ament_target_dependencies(standalone_node
  ${dependencies}
)

target_link_libraries(gridmap_to_polygon ${YAML_CPP_LIBRARIES} )

target_link_libraries(standalone_node
  gridmap_to_polygon
)

install(TARGETS standalone_node DESTINATION lib/${PROJECT_NAME} )
install(TARGETS action_server DESTINATION lib/${PROJECT_NAME} )


install(DIRECTORY config maps DESTINATION share/${PROJECT_NAME}/ )

install(TARGETS gridmap_to_polygon
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(gridmap_to_polygon)
ament_export_dependencies(${dependencies})

if(BUILD_TESTING) 
  find_package(ament_lint_auto REQUIRED) 
  ament_lint_auto_find_test_dependencies() 
endif()

ament_package()
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

include_directories(
  include
)

add_executable(standalone_node standalone_node.cpp)

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
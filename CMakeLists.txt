cmake_minimum_required(VERSION 3.8)
project(polaris_sensor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(serial REQUIRED)

find_package(Boost COMPONENTS system filesystem REQUIRED)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

add_library(${PROJECT_NAME}
   src/polaris_sensor.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC include ${Boost_INCLUDE_DIR})

ament_target_dependencies(${PROJECT_NAME}
   rclcpp
   std_msgs
   geometry_msgs
   sensor_msgs
   tf2_ros
   serial)

target_link_libraries(${PROJECT_NAME}
   ${Boost_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/polaris_ros.cpp)
# add_executable(${PROJECT_NAME}_double_node src/polaris_ros_double.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${Boost_LIBRARIES} ${PROJECT_NAME}
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  rclcpp_action
  tf2_geometry_msgs
  tf2_ros
  serial
  sensor_msgs
  geometry_msgs
)

# target_link_libraries(${PROJECT_NAME}_double_node
   # ${Boost_LIBRARIES} ${PROJECT_NAME}
# )

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION share
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE
# )
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rom DESTINATION share/${PROJECT_NAME})
ament_package()

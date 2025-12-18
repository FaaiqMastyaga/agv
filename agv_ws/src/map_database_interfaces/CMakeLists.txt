cmake_minimum_required(VERSION 3.8)
project(map_database_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Define the service files
set(srv_files
  srv/LoadMap.srv
  srv/GetMap.srv
  srv/GetMapName.srv
  srv/SaveMap.srv
  srv/RemoveMap.srv
)

# Generate the service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES aruco_msgs std_msgs
)

ament_package()

cmake_minimum_required(VERSION 3.5)
project(pacsim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(rclcpp_components REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(Eigen3 REQUIRED)


rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/StampedScalar.msg"
  "msg/Wheels.msg"
  "msg/Landmark.msg"
  "msg/PerceptionDetections.msg"
  "msg/GNSS.msg"
  "msg/Track.msg"
  "srv/ClockTriggerAbsolute.srv"
  "srv/ClockTriggerRelative.srv"
  DEPENDENCIES
   geometry_msgs
   std_msgs
   sensor_msgs
)

file(GLOB_RECURSE pacsim_SRCS
        "${PROJECT_SOURCE_DIR}/src/*.cpp"
        "${PROJECT_SOURCE_DIR}/src/*.c"
        )


add_executable(${PROJECT_NAME}_node ${pacsim_SRCS})
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_srvs visualization_msgs geometry_msgs tf2_ros yaml-cpp)
ament_export_dependencies(std_msgs geometry_msgs rosidl_default_runtime)

rosidl_target_interfaces(${PROJECT_NAME}_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_include_directories(${PROJECT_NAME}_node PUBLIC
${EIGEN3_INCLUDE_DIRS}
"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
"$<INSTALL_INTERFACE:include>")

target_link_libraries(
  ${PROJECT_NAME}_node 
  ${YAML_CPP_LIBRARIES}
  )


install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

file(GLOB_RECURSE pacsim_TRACKS
        "${PROJECT_SOURCE_DIR}/tracks/*.yaml"
)
file(GLOB_RECURSE pacsim_CONFIGS
        "${PROJECT_SOURCE_DIR}/config/*.yaml"
)
file(GLOB_RECURSE modells
        "${PROJECT_SOURCE_DIR}/urdf/*"
)
install(FILES ${modells}
  DESTINATION share/${PROJECT_NAME}/urdf
)
install(FILES ${pacsim_TRACKS}
  DESTINATION tracks/${PROJECT_NAME}
)
install(FILES ${pacsim_CONFIGS}
  DESTINATION config/${PROJECT_NAME}
)
ament_auto_package(INSTALL_TO_SHARE
  tracks
  config
  launch
)
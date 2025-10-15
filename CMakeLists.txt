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

find_package(Python3 REQUIRED COMPONENTS Development)
find_package(pybind11 REQUIRED)

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



file(GLOB_RECURSE pacsim_SRCS
        "${PROJECT_SOURCE_DIR}/src/*.cpp"
        "${PROJECT_SOURCE_DIR}/src/*.c"
        )

list(REMOVE_ITEM pacsim_SRCS "${PROJECT_SOURCE_DIR}/src/pacsim_main.cpp")
list(REMOVE_ITEM pacsim_SRCS "${PROJECT_SOURCE_DIR}/src/ros2Helpers.cpp")
# list(REMOVE_ITEM pacsim_SRCS "${PROJECT_SOURCE_DIR}/src/competitionLogic.cpp")

message(PROJECT_SOURCE_DIR="${pacsim_SRCS}")

add_library(pacsim_lib SHARED ${pacsim_SRCS})
# target_link_libraries(pacsim_lib PUBLIC rclcpp::rclcpp)

target_include_directories(pacsim_lib PUBLIC
${EIGEN3_INCLUDE_DIRS}
"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
"$<INSTALL_INTERFACE:include>")

# rosidl_target_interfaces(pacsim_lib
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(pacsim_lib rclcpp std_srvs visualization_msgs geometry_msgs tf2_ros yaml-cpp)

pybind11_add_module(example src/bindings.cpp "${PROJECT_SOURCE_DIR}/src/configParser.cpp" ${pacsim_SRCS} "${PROJECT_SOURCE_DIR}/src/sensorModels/imuSensor.cpp" "${PROJECT_SOURCE_DIR}/src/track/trackLoader.cpp")

###HACK : ament_target_dependencies cannot declare its deps as private, which is needed for pybind11.. so copypaste
function(ament_target_dependencies_private target)
  if(NOT TARGET ${target})
    message(FATAL_ERROR "ament_target_dependencies() the first argument must be a valid target name")
  endif()
  if(${ARGC} GREATER 0)
    set(definitions "")
    set(include_dirs "")
    set(libraries "")
    set(link_flags "")
    foreach(package_name ${ARGN})
      if(NOT ${${package_name}_FOUND})
        message(FATAL_ERROR "ament_target_dependencies() the passed package name '${package_name}' was not found before")
      endif()
      list_append_unique(definitions ${${package_name}_DEFINITIONS})
      list_append_unique(include_dirs ${${package_name}_INCLUDE_DIRS})
      list(APPEND libraries ${${package_name}_LIBRARIES})
      list_append_unique(link_flags ${${package_name}_LINK_FLAGS})
    endforeach()
    target_compile_definitions(${target}
      PUBLIC ${definitions})
    ament_include_directories_order(ordered_include_dirs ${include_dirs})
    target_include_directories(${target}
      PUBLIC ${ordered_include_dirs})
    ament_libraries_deduplicate(unique_libraries ${libraries})
    target_link_libraries(${target} PRIVATE
      ${unique_libraries})
    foreach(link_flag IN LISTS link_flags)
      set_property(TARGET ${target} APPEND_STRING PROPERTY LINK_FLAGS " ${link_flag} ")
    endforeach()
  endif()
endfunction()



set_target_properties(example PROPERTIES PREFIX "${PYTHON_MODULE_PREFIX}"
                                         SUFFIX "${PYTHON_MODULE_EXTENSION}")



# target_link_libraries(example PUBLIC Eigen3::Eigen rclcpp::rclcpp)
# ament_target_dependencies(example rclcpp std_srvs visualization_msgs geometry_msgs tf2_ros yaml-cpp)
target_link_libraries(example PUBLIC Eigen3::Eigen yaml-cpp rclcpp::rclcpp)

target_include_directories(example PUBLIC
${EIGEN3_INCLUDE_DIRS}
"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
"$<INSTALL_INTERFACE:include>")

install(TARGETS example DESTINATION .)
# target_include_directories(example PUBLIC
# "/usr/include/python3.12"
# )


# set(PythonModuleName "example.${Python3_SOABI}.so")
set(PythonModuleName "example.so")
file(TOUCH ${CMAKE_CURRENT_BINARY_DIR}/${PythonModuleName})

ament_python_install_module(${CMAKE_CURRENT_BINARY_DIR}/${PythonModuleName})

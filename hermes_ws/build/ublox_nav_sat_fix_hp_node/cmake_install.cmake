# Install script for directory: /home/raivo/Hermes/hermes_ws/src/gps_publisher/ublox_nav_sat_fix_hp_node

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/raivo/Hermes/hermes_ws/install/ublox_nav_sat_fix_hp_node")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ublox_nav_sat_fix_hp_node/ublox_nav_sat_fix_hp" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ublox_nav_sat_fix_hp_node/ublox_nav_sat_fix_hp")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ublox_nav_sat_fix_hp_node/ublox_nav_sat_fix_hp"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/ublox_nav_sat_fix_hp_node" TYPE EXECUTABLE FILES "/home/raivo/Hermes/hermes_ws/build/ublox_nav_sat_fix_hp_node/ublox_nav_sat_fix_hp")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ublox_nav_sat_fix_hp_node/ublox_nav_sat_fix_hp" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ublox_nav_sat_fix_hp_node/ublox_nav_sat_fix_hp")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ublox_nav_sat_fix_hp_node/ublox_nav_sat_fix_hp"
         OLD_RPATH "/home/raivo/ros2_humble/install/rclcpp_components/lib:/home/raivo/ros2_humble/install/rclcpp/lib:/home/raivo/ros2_humble/install/libstatistics_collector/lib:/home/raivo/ros2_humble/install/rcl/lib:/home/raivo/ros2_humble/install/rmw_implementation/lib:/home/raivo/ros2_humble/install/rcl_logging_spdlog/lib:/home/raivo/ros2_humble/install/rcl_logging_interface/lib:/home/raivo/ros2_humble/install/rcl_yaml_param_parser/lib:/home/raivo/ros2_humble/install/libyaml_vendor/lib:/home/raivo/ros2_humble/install/rosgraph_msgs/lib:/home/raivo/ros2_humble/install/statistics_msgs/lib:/home/raivo/ros2_humble/install/tracetools/lib:/home/raivo/ros2_humble/install/class_loader/lib:/home/raivo/ros2_humble/install/console_bridge_vendor/lib:/home/raivo/ros2_humble/install/ament_index_cpp/lib:/home/raivo/ros2_humble/install/composition_interfaces/lib:/home/raivo/ros2_humble/install/rcl_interfaces/lib:/home/raivo/ros2_humble/install/builtin_interfaces/lib:/home/raivo/ros2_humble/install/rosidl_typesupport_fastrtps_c/lib:/home/raivo/ros2_humble/install/rosidl_typesupport_fastrtps_cpp/lib:/home/raivo/ros2_humble/install/rmw/lib:/home/raivo/ros2_humble/install/fastcdr/lib:/home/raivo/ros2_humble/install/rosidl_typesupport_introspection_cpp/lib:/home/raivo/ros2_humble/install/rosidl_typesupport_introspection_c/lib:/home/raivo/ros2_humble/install/rosidl_typesupport_cpp/lib:/home/raivo/ros2_humble/install/rosidl_typesupport_c/lib:/home/raivo/ros2_humble/install/rcpputils/lib:/home/raivo/ros2_humble/install/rosidl_runtime_c/lib:/home/raivo/ros2_humble/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/ublox_nav_sat_fix_hp_node/ublox_nav_sat_fix_hp")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libublox_nav_sat_fix_hp_components.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libublox_nav_sat_fix_hp_components.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libublox_nav_sat_fix_hp_components.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/raivo/Hermes/hermes_ws/build/ublox_nav_sat_fix_hp_node/libublox_nav_sat_fix_hp_components.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libublox_nav_sat_fix_hp_components.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libublox_nav_sat_fix_hp_components.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libublox_nav_sat_fix_hp_components.so"
         OLD_RPATH "/home/raivo/ros2_humble/install/rclcpp_components/lib:/home/raivo/ros2_humble/install/sensor_msgs/lib:/home/raivo/Hermes/hermes_ws/install/ublox_ubx_msgs/lib:/home/raivo/Hermes/hermes_ws/install/ublox_ubx_interfaces/lib:/home/raivo/ros2_humble/install/rclcpp/lib:/home/raivo/ros2_humble/install/libstatistics_collector/lib:/home/raivo/ros2_humble/install/rcl/lib:/home/raivo/ros2_humble/install/rmw_implementation/lib:/home/raivo/ros2_humble/install/rcl_logging_spdlog/lib:/home/raivo/ros2_humble/install/rcl_logging_interface/lib:/home/raivo/ros2_humble/install/rcl_yaml_param_parser/lib:/home/raivo/ros2_humble/install/libyaml_vendor/lib:/home/raivo/ros2_humble/install/rosgraph_msgs/lib:/home/raivo/ros2_humble/install/statistics_msgs/lib:/home/raivo/ros2_humble/install/tracetools/lib:/home/raivo/ros2_humble/install/ament_index_cpp/lib:/home/raivo/ros2_humble/install/class_loader/lib:/home/raivo/ros2_humble/install/console_bridge_vendor/lib:/home/raivo/ros2_humble/install/composition_interfaces/lib:/home/raivo/ros2_humble/install/rcl_interfaces/lib:/home/raivo/ros2_humble/install/geometry_msgs/lib:/home/raivo/ros2_humble/install/std_msgs/lib:/home/raivo/ros2_humble/install/builtin_interfaces/lib:/home/raivo/ros2_humble/install/rosidl_typesupport_fastrtps_c/lib:/home/raivo/ros2_humble/install/rosidl_typesupport_introspection_cpp/lib:/home/raivo/ros2_humble/install/rosidl_typesupport_introspection_c/lib:/home/raivo/ros2_humble/install/rosidl_typesupport_fastrtps_cpp/lib:/home/raivo/ros2_humble/install/fastcdr/lib:/home/raivo/ros2_humble/install/rmw/lib:/home/raivo/ros2_humble/install/rosidl_typesupport_cpp/lib:/home/raivo/ros2_humble/install/rosidl_typesupport_c/lib:/home/raivo/ros2_humble/install/rcpputils/lib:/home/raivo/ros2_humble/install/rosidl_runtime_c/lib:/home/raivo/ros2_humble/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libublox_nav_sat_fix_hp_components.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/raivo/Hermes/hermes_ws/build/ublox_nav_sat_fix_hp_node/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/ublox_nav_sat_fix_hp_node")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/raivo/Hermes/hermes_ws/build/ublox_nav_sat_fix_hp_node/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/ublox_nav_sat_fix_hp_node")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ublox_nav_sat_fix_hp_node/environment" TYPE FILE FILES "/home/raivo/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ublox_nav_sat_fix_hp_node/environment" TYPE FILE FILES "/home/raivo/Hermes/hermes_ws/build/ublox_nav_sat_fix_hp_node/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ublox_nav_sat_fix_hp_node/environment" TYPE FILE FILES "/home/raivo/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ublox_nav_sat_fix_hp_node/environment" TYPE FILE FILES "/home/raivo/Hermes/hermes_ws/build/ublox_nav_sat_fix_hp_node/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ublox_nav_sat_fix_hp_node" TYPE FILE FILES "/home/raivo/Hermes/hermes_ws/build/ublox_nav_sat_fix_hp_node/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ublox_nav_sat_fix_hp_node" TYPE FILE FILES "/home/raivo/Hermes/hermes_ws/build/ublox_nav_sat_fix_hp_node/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ublox_nav_sat_fix_hp_node" TYPE FILE FILES "/home/raivo/Hermes/hermes_ws/build/ublox_nav_sat_fix_hp_node/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ublox_nav_sat_fix_hp_node" TYPE FILE FILES "/home/raivo/Hermes/hermes_ws/build/ublox_nav_sat_fix_hp_node/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ublox_nav_sat_fix_hp_node" TYPE FILE FILES "/home/raivo/Hermes/hermes_ws/build/ublox_nav_sat_fix_hp_node/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/raivo/Hermes/hermes_ws/build/ublox_nav_sat_fix_hp_node/ament_cmake_index/share/ament_index/resource_index/packages/ublox_nav_sat_fix_hp_node")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/rclcpp_components" TYPE FILE FILES "/home/raivo/Hermes/hermes_ws/build/ublox_nav_sat_fix_hp_node/ament_cmake_index/share/ament_index/resource_index/rclcpp_components/ublox_nav_sat_fix_hp_node")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ublox_nav_sat_fix_hp_node/cmake" TYPE FILE FILES
    "/home/raivo/Hermes/hermes_ws/build/ublox_nav_sat_fix_hp_node/ament_cmake_core/ublox_nav_sat_fix_hp_nodeConfig.cmake"
    "/home/raivo/Hermes/hermes_ws/build/ublox_nav_sat_fix_hp_node/ament_cmake_core/ublox_nav_sat_fix_hp_nodeConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ublox_nav_sat_fix_hp_node" TYPE FILE FILES "/home/raivo/Hermes/hermes_ws/src/gps_publisher/ublox_nav_sat_fix_hp_node/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/raivo/Hermes/hermes_ws/build/ublox_nav_sat_fix_hp_node/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")

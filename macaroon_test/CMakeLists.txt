cmake_minimum_required(VERSION 3.5)
project(macaroon_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_PREFIX_PATH /home/broomstick/libmacaroons/.libs:${CMAKE_PREFIX_PATH})
set(MACAROONS_PATH /home/broomstick/libmacaroons)
find_library(MACAROONS_LIBRARY NAMES libmacaroons macaroons PATHS /home/broomstick/libmacaroons/.libs)

include_directories(include ${MACAROONS_PATH})

add_library(composition_nodes SHARED
            src/publisher_node.cpp
            src/subscriber_node.cpp
            src/macaroon.cpp
            src/macaroon_verifier.cpp)
target_compile_definitions(composition_nodes
  PRIVATE "MINIMAL_COMPOSITION_DLL")
ament_target_dependencies(composition_nodes rclcpp rclcpp_components std_msgs)

# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif()

add_executable(composition_publisher src/standalone_publisher.cpp)
target_link_libraries(composition_publisher composition_nodes ${MACAROONS_LIBRARY})
ament_target_dependencies(composition_publisher
  rclcpp composition_nodes)

add_executable(composition_subscriber src/standalone_subscriber.cpp)
target_link_libraries(composition_subscriber composition_nodes ${MACAROONS_LIBRARY})
ament_target_dependencies(composition_subscriber
  rclcpp composition_nodes)

add_executable(composition_composed src/composed.cpp)
target_link_libraries(composition_composed composition_nodes ${MACAROONS_LIBRARY})
ament_target_dependencies(composition_composed rclcpp class_loader)

add_executable(macaroon_test src/macaroon_test.cpp)
target_link_libraries(macaroon_test composition_nodes ${MACAROONS_LIBRARY})
ament_target_dependencies(macaroon_test rclcpp class_loader)

install(TARGETS
  composition_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  composition_publisher
  composition_subscriber
  composition_composed
  macaroon_test
  DESTINATION lib/${PROJECT_NAME})

ament_package()

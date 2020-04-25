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
find_package(macaroons REQUIRED)
find_library(MACAROONS_LIB macaroons)

add_library(macaroon_classes SHARED
            src/macaroon.cpp
            src/macaroon_verifier.cpp)
target_compile_definitions(macaroon_classes
  PRIVATE "MINIMAL_COMPOSITION_DLL")
ament_target_dependencies(macaroon_classes rclcpp rclcpp_components std_msgs macaroons)

# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif()

target_include_directories(macaroon_classes PRIVATE include)
target_include_directories(macaroon_classes PRIVATE src)

add_executable(macaroon_test src/macaroon_test.cpp)
target_link_libraries(macaroon_test macaroon_classes ${MACAROONS_LIB})
ament_target_dependencies(macaroon_test rclcpp class_loader)
target_include_directories(macaroon_test PRIVATE include)
target_include_directories(macaroon_test PRIVATE src)

add_executable(macaroon_owner src/macaroon_owner.cpp)
target_link_libraries(macaroon_owner macaroon_classes ${MACAROONS_LIB})
ament_target_dependencies(macaroon_owner rclcpp class_loader)
target_include_directories(macaroon_owner PRIVATE include)
target_include_directories(macaroon_owner PRIVATE src)

add_executable(macaroon_user src/macaroon_user.cpp)
target_link_libraries(macaroon_user macaroon_classes ${MACAROONS_LIB})
ament_target_dependencies(macaroon_user rclcpp class_loader)
target_include_directories(macaroon_user PRIVATE include)
target_include_directories(macaroon_user PRIVATE src)

install(TARGETS
  macaroon_classes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  macaroon_test
  macaroon_owner
  macaroon_user
  DESTINATION lib/${PROJECT_NAME})

ament_package()
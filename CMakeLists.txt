cmake_minimum_required(VERSION 3.5)
project(sg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Required by minimal_example and the sg_controller
find_package(rclcpp REQUIRED)
find_package(abb_librws REQUIRED)


## minimal_example
add_executable(minimal_example src/minimal_example.cpp)
ament_target_dependencies(minimal_example
                          abb_librws
)
install(TARGETS 
          minimal_example
          DESTINATION lib/${PROJECT_NAME}
)


# Only required by the sg_controller
find_package(std_msgs REQUIRED)

## sg_controller_node
add_executable(sg_controller_node src/sg_controller.cpp)
ament_target_dependencies(sg_controller_node
                          rclcpp
                          abb_librws
                          std_msgs
)
install(TARGETS 
          sg_controller_node
          DESTINATION lib/${PROJECT_NAME}
)


ament_package()

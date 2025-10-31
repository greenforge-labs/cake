cmake_minimum_required(VERSION 3.22)
project(cake_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED DIRECTORY nodes)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

# NOTE: first argument to generate_parameter_library is "LIB_NAME" - impacts the name of the generated header file
generate_parameter_library(my_node_parameters nodes/my_node/parameters.yaml)
target_link_libraries(${PROJECT_NAME} my_node_parameters)

rclcpp_components_register_node(${PROJECT_NAME} PLUGIN "cake_example::my_node::MyNode" EXECUTABLE "my_node")

ament_auto_package()

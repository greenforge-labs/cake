cmake_minimum_required(VERSION 3.22)
project(cake_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED DIRECTORY nodes)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

# Automatically register all nodes in the nodes/ directory
cake_auto_package()

ament_auto_package()

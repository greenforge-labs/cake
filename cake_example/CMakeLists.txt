cmake_minimum_required(VERSION 3.22)
project(cake_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(cake REQUIRED)

# Automatically handle all package setup
cake_auto_package()

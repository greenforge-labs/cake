cmake_minimum_required(VERSION 3.28)
project(cake)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Install the code generation script
install(PROGRAMS scripts/generate_node_interface.py DESTINATION lib/${PROJECT_NAME})

# Install the templates directory
install(DIRECTORY scripts/templates DESTINATION lib/${PROJECT_NAME})

# Install and export the CMake module
install(FILES cmake/cake_codegen.cmake DESTINATION share/${PROJECT_NAME}/cmake)

# Export the CMake module so other packages can use it
list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS cmake/cake_codegen.cmake)

ament_auto_package()

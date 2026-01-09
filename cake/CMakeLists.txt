cmake_minimum_required(VERSION 3.28)
project(cake)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Install the code generation script
install(PROGRAMS scripts/generate_node_interface.py DESTINATION lib/${PROJECT_NAME})

# Install the templates directory
install(DIRECTORY scripts/templates DESTINATION lib/${PROJECT_NAME})

# Install and export the CMake modules install(FILES cmake/cake_codegen.cmake cmake/cake_auto_package.cmake DESTINATION
# share/${PROJECT_NAME}/cmake)
install(FILES cmake/cake_auto_package.cmake cmake/cake_auto_interface_package.cmake
        DESTINATION share/${PROJECT_NAME}/cmake
)

# Export the CMake modules so other packages can use them
list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS cmake/cake_auto_package.cmake cmake/cake_auto_interface_package.cmake)

ament_python_install_package(${PROJECT_NAME})

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE schemas)

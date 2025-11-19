# ============================================================================
# cake_auto_interface_package.cmake
#
# Automated build system for ROS 2 interface packages. Auto-discovers .msg, .srv, and .action files and generates the
# corresponding ROS 2 interfaces.
#
# Simplifies interface package CMakeLists.txt to just: find_package(cake REQUIRED) cake_auto_interface_package()
#
# The macro handles: - Finding ament_cmake_auto and build dependencies - Auto-discovering msg/*.msg, srv/*.srv,
# action/*.action files - Calling rosidl_generate_interfaces() with discovered files - Finalizing with
# ament_auto_package()
#
# All dependencies should be declared in package.xml and will be automatically discovered via
# ament_auto_find_build_dependencies().
# ============================================================================

# Main entry point for cake interface package build system Automates the entire build process for ROS 2 interface
# packages. Requires: At least one of msg/, srv/, or action/ directories with interface files
macro(cake_auto_interface_package)
    # Find the message code generation library
    find_package(rosidl_default_generators REQUIRED)

    # Find ament_cmake_auto and discover dependencies from package.xml
    find_package(ament_cmake_auto REQUIRED)
    ament_auto_find_build_dependencies()

    # Auto-discover interface files
    file(GLOB ${PROJECT_NAME}_msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*)
    file(GLOB ${PROJECT_NAME}_srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*)
    file(GLOB ${PROJECT_NAME}_action_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} action/*)

    # Combine all interface files
    set(_cake_interface_all_files ${${PROJECT_NAME}_msg_files} ${${PROJECT_NAME}_srv_files}
                                  ${${PROJECT_NAME}_action_files}
    )

    # Validate that we have at least one interface file
    if(NOT _cake_interface_all_files)
        message(FATAL_ERROR "cake: No interface files found in msg/, srv/, or action/ directories")
    endif()

    # Count and report discovered files
    list(LENGTH ${PROJECT_NAME}_msg_files _cake_msg_count)
    list(LENGTH ${PROJECT_NAME}_srv_files _cake_srv_count)
    list(LENGTH ${PROJECT_NAME}_action_files _cake_action_count)

    message(
        STATUS
            "cake: Discovered ${_cake_msg_count} message(s), ${_cake_srv_count} service(s), ${_cake_action_count} action(s)"
    )

    # Generate interfaces using discovered dependencies from package.xml
    rosidl_generate_interfaces(
        ${PROJECT_NAME} ${${PROJECT_NAME}_msg_files} ${${PROJECT_NAME}_srv_files} ${${PROJECT_NAME}_action_files}
        DEPENDENCIES ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
    )

    message(STATUS "cake: Generated interfaces for '${PROJECT_NAME}'")

    # Finalize package
    ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
endmacro()

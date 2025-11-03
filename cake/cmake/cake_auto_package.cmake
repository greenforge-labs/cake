# CMake helper for automatic cake package setup
#
# This module provides the cake_auto_package() macro which automates the process of setting up cake-based ROS2 packages
# with multiple nodes.

# Internal: Converts snake_case string to PascalCase
#
# Args: OUTPUT_VAR: Name of the variable to store the result INPUT_STRING: The snake_case string to convert (e.g.,
# "my_node")
#
# Example: _cake_snake_to_pascal(RESULT "my_node") # RESULT will be "MyNode"
#
function(_cake_snake_to_pascal OUTPUT_VAR INPUT_STRING)
    string(REPLACE "_" ";" WORD_LIST ${INPUT_STRING})
    set(RESULT "")

    foreach(WORD ${WORD_LIST})
        # Capitalize first letter
        string(SUBSTRING ${WORD} 0 1 FIRST_CHAR)
        string(TOUPPER ${FIRST_CHAR} FIRST_CHAR_UPPER)
        string(SUBSTRING ${WORD} 1 -1 REST)
        string(APPEND RESULT "${FIRST_CHAR_UPPER}${REST}")
    endforeach()

    set(${OUTPUT_VAR} ${RESULT} PARENT_SCOPE)
endfunction()

# Internal: Registers a single node with its interface and parameters
#
# This function handles: - Generating node interface library if interface.yaml exists - Generating parameters library if
# parameters.yaml exists - Linking generated libraries to the target library - Registering the node as an rclcpp
# component
#
# Args: TARGET_LIBRARY: The main library target to link against (e.g., ${PROJECT_NAME}) NODE_NAME: The name of the node
# directory (e.g., "my_node") NODE_DIR: The full path to the node directory
#
function(_cake_register_single_node TARGET_LIBRARY NODE_NAME NODE_DIR)
    set(INTERFACE_YAML "${NODE_DIR}/interface.yaml")
    set(PARAMETERS_YAML "${NODE_DIR}/parameters.yaml")

    # Generate and link interface library if interface.yaml exists
    if(EXISTS ${INTERFACE_YAML})
        set(INTERFACE_LIB_NAME "${NODE_NAME}_interface")
        cake_generate_node_interface(${INTERFACE_LIB_NAME} "nodes/${NODE_NAME}/interface.yaml")
        target_link_libraries(${TARGET_LIBRARY} ${INTERFACE_LIB_NAME})
        message(STATUS "cake: Generated interface library for node '${NODE_NAME}'")
    endif()

    # Generate and link parameters library if parameters.yaml exists
    if(EXISTS ${PARAMETERS_YAML})
        set(PARAMETERS_LIB_NAME "${NODE_NAME}_parameters")
        generate_parameter_library(${PARAMETERS_LIB_NAME} "nodes/${NODE_NAME}/parameters.yaml")
        target_link_libraries(${TARGET_LIBRARY} ${PARAMETERS_LIB_NAME})
        message(STATUS "cake: Generated parameters library for node '${NODE_NAME}'")
    endif()

    # Register the node as an rclcpp component Convention: ${PROJECT_NAME}::${NODE_NAME}::${NodeNamePascal}
    _cake_snake_to_pascal(NODE_CLASS_NAME ${NODE_NAME})
    set(PLUGIN_CLASS "${PROJECT_NAME}::${NODE_NAME}::${NODE_CLASS_NAME}")

    rclcpp_components_register_node(${TARGET_LIBRARY} PLUGIN ${PLUGIN_CLASS} EXECUTABLE ${NODE_NAME})
    message(STATUS "cake: Registered component '${PLUGIN_CLASS}' with executable '${NODE_NAME}'")
endfunction()

# Internal: Scans the nodes/ directory and registers all found nodes
#
# This function: - Looks for the nodes/ directory in CMAKE_CURRENT_SOURCE_DIR - Iterates over all subdirectories in
# nodes/ - Calls _cake_register_single_node for each node found
#
# Args: TARGET_LIBRARY: The main library target to link against (e.g., ${PROJECT_NAME})
#
function(_cake_auto_register_nodes TARGET_LIBRARY)
    set(NODES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/nodes")

    if(NOT EXISTS ${NODES_DIR})
        message(WARNING "cake: nodes/ directory not found at ${NODES_DIR}")
        return()
    endif()

    # Get all subdirectories in nodes/
    file(GLOB NODE_DIRS RELATIVE ${NODES_DIR} ${NODES_DIR}/*)

    if(NOT NODE_DIRS)
        message(WARNING "cake: No nodes found in ${NODES_DIR}")
        return()
    endif()

    message(STATUS "cake: Auto-registering nodes from ${NODES_DIR}")

    foreach(NODE_ENTRY ${NODE_DIRS})
        set(NODE_PATH "${NODES_DIR}/${NODE_ENTRY}")

        # Only process directories
        if(IS_DIRECTORY ${NODE_PATH})
            _cake_register_single_node(${TARGET_LIBRARY} ${NODE_ENTRY} ${NODE_PATH})
        endif()
    endforeach()
endfunction()

# Main entry point: Automates complete cake package setup
#
# This macro automates the entire setup process for cake-based packages: - Finds all build dependencies via ament_auto -
# Creates the main library from all source files in nodes/ - Sets C++20 standard requirement - Automatically discovers
# and registers all nodes in the nodes/ directory - Generates interface and parameter libraries for each node -
# Registers nodes as rclcpp components - Finalizes package with ament_auto_package()
#
# Usage: cmake_minimum_required(VERSION 3.22) project(my_package)
#
# find_package(cake REQUIRED)
#
# cake_auto_package()
#
# Conventions enforced: - Main library target is named ${PROJECT_NAME} - Library type is SHARED (for rclcpp components)
# - C++ standard is C++20 - Source files are located in nodes/ subdirectory - Node directories use snake_case (e.g.,
# "my_node") - C++ classes use PascalCase (e.g., "MyNode") - Namespace: ${PROJECT_NAME}::${node_name} - Full plugin:
# ${PROJECT_NAME}::${node_name}::${NodeClass}
#
macro(cake_auto_package)
    # Find ament_cmake_auto (required for all subsequent calls)
    find_package(ament_cmake_auto REQUIRED)

    # Find generate_parameter_library (used internally for parameters.yaml files)
    find_package(generate_parameter_library REQUIRED)

    # Find all build dependencies
    ament_auto_find_build_dependencies()

    # Create the main library from all source files in nodes/
    ament_auto_add_library(${PROJECT_NAME} SHARED DIRECTORY nodes)
    target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

    # Auto-register all nodes
    _cake_auto_register_nodes(${PROJECT_NAME})

    # Finalize package with scoped header install directory (best practice). USE_SCOPED_HEADER_INSTALL_DIR is used so
    # that we behave the same way on Jazzy as with Kilted. As far as I can tell, this is a non-breaking change because
    # it also changes which include directory is set with ament_export_include_directories - i.e. it doesn't really
    # matter.
    ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
endmacro()

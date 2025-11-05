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
# This function handles: - Detecting node language (C++ or Python) - Dispatching to appropriate language-specific
# registration function
#
# Args: TARGET_LIBRARY: The main library target to link against (e.g., ${PROJECT_NAME}) NODE_NAME: The name of the node
# directory (e.g., "my_node") NODE_DIR: The full path to the node directory
#
function(_cake_register_single_node TARGET_LIBRARY NODE_NAME NODE_DIR)
    set(INTERFACE_YAML "${NODE_DIR}/interface.yaml")

    # Detect language by checking for source files
    file(GLOB CPP_FILES "${NODE_DIR}/*.cpp" "${NODE_DIR}/*.hpp")
    file(GLOB PY_FILES "${NODE_DIR}/*.py")

    set(IS_CPP_NODE FALSE)
    set(IS_PYTHON_NODE FALSE)

    if(CPP_FILES)
        set(IS_CPP_NODE TRUE)
    endif()

    if(PY_FILES)
        set(IS_PYTHON_NODE TRUE)
    endif()

    # Validate: must be exactly one language
    if(IS_CPP_NODE AND IS_PYTHON_NODE)
        message(
            FATAL_ERROR
                "cake: Node '${NODE_NAME}' has both C++ and Python files. Mixed language nodes are not supported."
        )
    elseif(NOT IS_CPP_NODE AND NOT IS_PYTHON_NODE)
        message(FATAL_ERROR "cake: Node '${NODE_NAME}' has no C++ (.cpp/.hpp) or Python (.py) files.")
    endif()

    # Dispatch to language-specific registration
    if(IS_CPP_NODE)
        _cake_register_cpp_node(${TARGET_LIBRARY} ${NODE_NAME} ${NODE_DIR} ${INTERFACE_YAML})
    else()
        _cake_register_python_node(${TARGET_LIBRARY} ${NODE_NAME} ${NODE_DIR} ${INTERFACE_YAML})
    endif()
endfunction()

# Internal: Registers a C++ node
#
# Args: TARGET_LIBRARY: The main library target to link against NODE_NAME: The name of the node directory NODE_DIR: The
# full path to the node directory INTERFACE_YAML: Path to the interface.yaml file
#
function(_cake_register_cpp_node TARGET_LIBRARY NODE_NAME NODE_DIR INTERFACE_YAML)
    # Generate and link interface library if interface.yaml exists
    if(EXISTS ${INTERFACE_YAML})
        set(INTERFACE_LIB_NAME "${NODE_NAME}_interface")
        cake_generate_node_interface(${INTERFACE_LIB_NAME} "nodes/${NODE_NAME}/interface.yaml" ${NODE_NAME} cpp)
        target_link_libraries(${TARGET_LIBRARY} ${INTERFACE_LIB_NAME})
        message(STATUS "cake: Generated C++ interface library for node '${NODE_NAME}'")
    endif()

    # Always generate and link parameters library Parameters are defined in interface.yaml and auto-generated as
    # .params.yaml
    set(GENERATED_PARAMETERS_YAML_ABS
        "${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/${INTERFACE_LIB_NAME}.params.yaml"
    )

    # generate_parameter_library expects a path relative to CMAKE_CURRENT_SOURCE_DIR Compute the relative path from
    # source to binary dir
    file(RELATIVE_PATH GENERATED_PARAMETERS_YAML_REL ${CMAKE_CURRENT_SOURCE_DIR} ${GENERATED_PARAMETERS_YAML_ABS})

    set(PARAMETERS_LIB_NAME "${NODE_NAME}_parameters")
    generate_parameter_library(${PARAMETERS_LIB_NAME} ${GENERATED_PARAMETERS_YAML_REL})

    # Make parameters library depend on interface library (which generates the .params.yaml)
    add_dependencies(${PARAMETERS_LIB_NAME} ${INTERFACE_LIB_NAME})

    target_link_libraries(${TARGET_LIBRARY} ${PARAMETERS_LIB_NAME})
    message(STATUS "cake: Generated parameters library for node '${NODE_NAME}'")

    # Register the node as an rclcpp component Convention: ${PROJECT_NAME}::${NODE_NAME}::${NodeNamePascal}
    _cake_snake_to_pascal(NODE_CLASS_NAME ${NODE_NAME})
    set(PLUGIN_CLASS "${PROJECT_NAME}::${NODE_NAME}::${NODE_CLASS_NAME}")

    rclcpp_components_register_node(${TARGET_LIBRARY} PLUGIN ${PLUGIN_CLASS} EXECUTABLE ${NODE_NAME})
    message(STATUS "cake: Registered component '${PLUGIN_CLASS}' with executable '${NODE_NAME}'")
endfunction()

# Internal: Registers a Python node
#
# Args: TARGET_LIBRARY: The main library target to link against (unused for Python) NODE_NAME: The name of the node
# directory NODE_DIR: The full path to the node directory INTERFACE_YAML: Path to the interface.yaml file
#
# Note: Uses PYTHON_INSTALL_DIR cache variable set by ament_cmake_python
#
function(_cake_register_python_node TARGET_LIBRARY NODE_NAME NODE_DIR INTERFACE_YAML)
    # PYTHON_INSTALL_DIR is a cache variable set by find_package(ament_cmake_python) It should already be available from
    # the macro scope
    if(NOT DEFINED PYTHON_INSTALL_DIR)
        message(FATAL_ERROR "cake: PYTHON_INSTALL_DIR not set. Ensure find_package(ament_cmake_python) was called.")
    endif()

    # Generate Python interface if interface.yaml exists
    if(EXISTS ${INTERFACE_YAML})
        set(INTERFACE_MODULE_NAME "${NODE_NAME}_interface")
        cake_generate_node_interface(${INTERFACE_MODULE_NAME} "nodes/${NODE_NAME}/interface.yaml" ${NODE_NAME} python)
        message(STATUS "cake: Generated Python interface for node '${NODE_NAME}'")
    endif()

    # Install user Python files to site-packages/${PROJECT_NAME}/${NODE_NAME}/
    file(GLOB USER_PY_FILES "${NODE_DIR}/*.py")
    if(USER_PY_FILES)
        install(FILES ${USER_PY_FILES} DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/${NODE_NAME}")
        message(STATUS "cake: Installed user Python files for node '${NODE_NAME}'")
    endif()

    # Install generated Python interface files to site-packages/${PROJECT_NAME}/${NODE_NAME}/
    set(PYTHON_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/python_generated/${NODE_NAME}")
    install(
        DIRECTORY ${PYTHON_GEN_DIR}/
        DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/${NODE_NAME}"
        FILES_MATCHING
        PATTERN "*.py"
    )

    # Generate wrapper executable using runpy.run_module()
    set(EXECUTABLE_PATH "${CMAKE_CURRENT_BINARY_DIR}/executables/${NODE_NAME}")
    file(
        WRITE ${EXECUTABLE_PATH}
        "#!/usr/bin/env python3
import runpy

# Run the node's main file as __main__
runpy.run_module('${PROJECT_NAME}.${NODE_NAME}.${NODE_NAME}', run_name='__main__')
"
    )

    # Make executable
    file(
        CHMOD
        ${EXECUTABLE_PATH}
        PERMISSIONS
        OWNER_READ
        OWNER_WRITE
        OWNER_EXECUTE
        GROUP_READ
        GROUP_EXECUTE
        WORLD_READ
        WORLD_EXECUTE
    )

    # Install executable to lib/${PROJECT_NAME}/ for ros2 run
    install(PROGRAMS ${EXECUTABLE_PATH} DESTINATION lib/${PROJECT_NAME})
    message(STATUS "cake: Created executable wrapper for Python node '${NODE_NAME}'")
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
# Creates the main library from all source files in nodes/ (for C++ nodes) - Sets C++20 standard requirement -
# Automatically discovers and registers all nodes in the nodes/ directory - Generates interface and parameter libraries
# for each node - Registers nodes as rclcpp components (C++) or creates executables (Python) - Finalizes package with
# ament_auto_package()
#
# Usage: cmake_minimum_required(VERSION 3.22) project(my_package)
#
# find_package(cake REQUIRED)
#
# cake_auto_package()
#
# Conventions enforced: - Main library target is named ${PROJECT_NAME} (C++ only) - Library type is SHARED (for rclcpp
# components) - C++ standard is C++20 - Source files are located in nodes/ subdirectory - Node directories use
# snake_case (e.g., "my_node") - C++ classes use PascalCase (e.g., "MyNode") - Namespace: ${PROJECT_NAME}::${node_name}
# - Full plugin: ${PROJECT_NAME}::${node_name}::${NodeClass}
#
macro(cake_auto_package)
    # Find ament_cmake_auto (required for all subsequent calls)
    find_package(ament_cmake_auto REQUIRED)

    # Find all build dependencies
    ament_auto_find_build_dependencies()

    # Detect if we have C++ or Python nodes
    file(GLOB_RECURSE CPP_SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/nodes/*.cpp")
    file(GLOB_RECURSE PY_SOURCE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/nodes/*.py")

    set(HAS_CPP_NODES FALSE)
    set(HAS_PYTHON_NODES FALSE)

    if(CPP_SOURCE_FILES)
        set(HAS_CPP_NODES TRUE)
    endif()

    if(PY_SOURCE_FILES)
        set(HAS_PYTHON_NODES TRUE)
    endif()

    # Setup C++ library if we have C++ nodes
    if(HAS_CPP_NODES)
        # Find generate_parameter_library (used internally for parameters.yaml files)
        find_package(generate_parameter_library REQUIRED)

        # Create the main library from all source files in nodes/
        ament_auto_add_library(${PROJECT_NAME} SHARED DIRECTORY nodes)
        target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
        message(STATUS "cake: Created C++ library '${PROJECT_NAME}'")
    endif()

    # Setup Python package if we have Python nodes
    if(HAS_PYTHON_NODES)
        # Find ament_cmake_python
        find_package(ament_cmake_python REQUIRED)

        # Explicitly get Python install directory (sets PYTHON_INSTALL_DIR cache variable)
        _ament_cmake_python_get_python_install_dir()

        # Create top-level __init__.py for the package
        set(PACKAGE_INIT_PY "${CMAKE_CURRENT_BINARY_DIR}/python_package_init/__init__.py")
        file(WRITE ${PACKAGE_INIT_PY} "# Auto-generated by cake_auto_package\n")
        install(FILES ${PACKAGE_INIT_PY} DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")
        message(STATUS "cake: Created Python package '${PROJECT_NAME}'")
    endif()

    # Auto-register all nodes (both C++ and Python)
    if(HAS_CPP_NODES OR HAS_PYTHON_NODES)
        _cake_auto_register_nodes(${PROJECT_NAME})
    else()
        message(WARNING "cake: No C++ or Python nodes found in nodes/ directory")
    endif()

    # Finalize package with scoped header install directory (best practice). USE_SCOPED_HEADER_INSTALL_DIR is used so
    # that we behave the same way on Jazzy as with Kilted. As far as I can tell, this is a non-breaking change because
    # it also changes which include directory is set with ament_export_include_directories - i.e. it doesn't really
    # matter.
    ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
endmacro()

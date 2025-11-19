# ============================================================================
# cake_auto_package.cmake
#
# Automated build system for ROS 2 packages with cake nodes. Expects nodes/ directory with subdirectories containing
# interface.yaml and C++ or Python implementation files.
#
# NOTE: Most internal functions are implemented as macros (not functions) for consistency with ament_cmake patterns and
# to preserve variable scope behavior. This allows macros like ament_auto_add_library() and
# rclcpp_components_register_node() to properly set variables in the caller's scope for later use by
# ament_auto_package(). All macro-local variables are prefixed with _cake_ or _cake_node_ to avoid namespace pollution.
# ============================================================================

# Detect which languages (C++ and/or Python) are present in a directory Args: DIR - Directory to scan for source files
# OUT_HAS_CPP - Output variable, set to TRUE if .cpp files found OUT_HAS_PYTHON - Output variable, set to TRUE if .py
# files found
function(_cake_detect_languages DIR OUT_HAS_CPP OUT_HAS_PYTHON)
    file(GLOB_RECURSE CPP_SOURCE_FILES "${DIR}/*.cpp")
    file(GLOB_RECURSE PY_SOURCE_FILES "${DIR}/*.py")

    set(${OUT_HAS_CPP} FALSE PARENT_SCOPE)
    set(${OUT_HAS_PYTHON} FALSE PARENT_SCOPE)

    if(CPP_SOURCE_FILES)
        set(${OUT_HAS_CPP} TRUE PARENT_SCOPE)
    endif()

    if(PY_SOURCE_FILES)
        set(${OUT_HAS_PYTHON} TRUE PARENT_SCOPE)
    endif()
endfunction()

# Convert snake_case to PascalCase (e.g., "my_node" -> "MyNode") Args: OUTPUT_VAR - Variable name to store the result
# INPUT_STRING - snake_case string to convert
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

# Main entry point for cake build system Automates the entire build process for ROS 2 packages with cake nodes. Detects
# languages, generates interfaces, builds libraries, and registers components. Requires: nodes/ directory with at least
# one .cpp or .py file
macro(cake_auto_package)
    # Parse optional arguments
    set(_cake_auto_options "")
    set(_cake_auto_oneValueArgs "")
    set(_cake_auto_multiValueArgs "INSTALL_TO_SHARE")
    cmake_parse_arguments(
        CAKE_AUTO "${_cake_auto_options}" "${_cake_auto_oneValueArgs}" "${_cake_auto_multiValueArgs}" ${ARGN}
    )

    # Auto-detect standard directories to install to share
    set(_cake_auto_detected_dirs "")
    if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/launch")
        list(APPEND _cake_auto_detected_dirs "launch")
    endif()
    if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/config")
        list(APPEND _cake_auto_detected_dirs "config")
    endif()

    # Combine auto-detected directories with user-specified ones
    set(_cake_install_to_share_combined ${_cake_auto_detected_dirs} ${CAKE_AUTO_INSTALL_TO_SHARE})
    if(_cake_install_to_share_combined)
        list(REMOVE_DUPLICATES _cake_install_to_share_combined)
    endif()

    find_package(ament_cmake_auto REQUIRED)
    ament_auto_find_build_dependencies()

    # NOTE: CAKE_NODES_DIR is part of the cake cmake API
    set(CAKE_NODES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/nodes")

    if(NOT IS_DIRECTORY ${CAKE_NODES_DIR})
        message(FATAL_ERROR "cake: nodes/ directory not found at ${CAKE_NODES_DIR}")
    endif()

    _cake_detect_languages(${CAKE_NODES_DIR} _cake_HAS_CPP _cake_HAS_PYTHON)

    if(NOT _cake_HAS_CPP AND NOT _cake_HAS_PYTHON)
        message(FATAL_ERROR "cake: nodes/ directory has no C++ (.cpp) or Python (.py) files.")
    endif()

    if(_cake_HAS_CPP)
        # NOTE: CAKE_CPP_PACKAGE_TARGET is part of the cake cmake API
        set(CAKE_CPP_PACKAGE_TARGET "${PROJECT_NAME}")
        _cake_create_package_shared_cpp_library(${CAKE_CPP_PACKAGE_TARGET})
    endif()

    if(_cake_HAS_PYTHON)
        # set up python package
        find_package(ament_cmake_python REQUIRED)
        _ament_cmake_python_get_python_install_dir()
        _cake_create_top_level_python_package()
    endif()

    _cake_generate_nodes(${CAKE_NODES_DIR})

    # Process and install interface.yaml files with token replacement
    _cake_process_and_install_interfaces(${CAKE_NODES_DIR})

    # Finalize package with scoped header install directory (best practice). USE_SCOPED_HEADER_INSTALL_DIR is used so
    # that we behave the same way on Jazzy as with Kilted. As far as I can tell, this is a non-breaking change because
    # it also changes which include directory is set with ament_export_include_directories - i.e. it doesn't really
    # matter.
    if(_cake_install_to_share_combined)
        ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE ${_cake_install_to_share_combined})
    else()
        ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
    endif()
endmacro()

# Create shared C++ library for all C++ nodes Creates a shared library from all .cpp files in nodes/ directory
macro(_cake_create_package_shared_cpp_library)
    # NOTE: this uses a slightly implicit path "nodes" because ament_auto_add_library uses a relative path we have
    # already checked the nodes directory exists in cake_auto_package before calling this function
    ament_auto_add_library(${CAKE_CPP_PACKAGE_TARGET} SHARED DIRECTORY nodes)
    target_compile_features(${CAKE_CPP_PACKAGE_TARGET} PUBLIC cxx_std_20)
    message(STATUS "cake: Created C++ library '${CAKE_CPP_PACKAGE_TARGET}'")
endmacro()

# Create top-level Python package __init__.py Generates and installs __init__.py to make package importable
macro(_cake_create_top_level_python_package)
    # Create top-level __init__.py for the package
    set(_cake_PACKAGE_INIT_PY "${CMAKE_CURRENT_BINARY_DIR}/python_package_init/__init__.py")
    file(WRITE ${_cake_PACKAGE_INIT_PY} "# Auto-generated by cake_auto_package\n")
    install(FILES ${_cake_PACKAGE_INIT_PY} DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")
    message(STATUS "cake: Created Python package '${PROJECT_NAME}'")
endmacro()

# Discover and process all nodes in the nodes/ directory Args: NODES_DIR - Path to nodes/ directory containing node
# subdirectories
macro(_cake_generate_nodes NODES_DIR)
    file(GLOB _cake_NODE_DIRS RELATIVE ${NODES_DIR} ${NODES_DIR}/*)
    if(NOT _cake_NODE_DIRS)
        message(WARNING "cake: No nodes found in ${NODES_DIR}")
        return()
    endif()

    message(STATUS "cake: Auto-generating nodes from ${NODES_DIR}")

    # find the generator script
    unset(_cake_codegen_script_BIN CACHE)
    find_program(
        _cake_codegen_script_BIN
        NAMES "generate_node_interface.py"
        PATHS "${cake_DIR}/../../../lib/cake"
        NO_DEFAULT_PATH
    )
    if(NOT _cake_codegen_script_BIN)
        message(FATAL_ERROR "Could not find generate_node_interface.py in ${cake_DIR}/../../../lib/cake")
    endif()

    foreach(_cake_NODE_ENTRY ${_cake_NODE_DIRS})
        set(_cake_NODE_PATH "${NODES_DIR}/${_cake_NODE_ENTRY}")
        if(IS_DIRECTORY ${_cake_NODE_PATH})
            _cake_generate_node(${_cake_NODE_ENTRY} ${_cake_NODE_PATH})
        endif()
    endforeach()
endmacro()

# Process a single node and generate language-specific interfaces Validates interface.yaml exists and dispatches to C++
# or Python generation. Mixed language nodes are not supported. Args: NODE_NAME - Name of the node NODE_DIR - Full path
# to the node's directory
macro(_cake_generate_node NODE_NAME NODE_DIR)
    set(_cake_node_INTERFACE_YAML "${NODE_DIR}/interface.yaml")
    if(NOT EXISTS ${_cake_node_INTERFACE_YAML})
        message(FATAL_ERROR "cake: Node '${NODE_NAME}' is missing interface.yaml.")
    endif()

    _cake_detect_languages(${NODE_DIR} _cake_node_HAS_CPP _cake_node_HAS_PYTHON)
    if(_cake_node_HAS_CPP AND _cake_node_HAS_PYTHON)
        message(
            FATAL_ERROR
                "cake: Node '${NODE_NAME}' has both C++ and Python files. Mixed language nodes are not supported."
        )
    elseif(NOT _cake_node_HAS_CPP AND NOT _cake_node_HAS_PYTHON)
        message(FATAL_ERROR "cake: Node '${NODE_NAME}' has no C++ (.cpp) or Python (.py) files.")
    endif()

    if(_cake_node_HAS_CPP)
        _cake_generate_cpp_node(${NODE_NAME} ${_cake_node_INTERFACE_YAML})
    endif()

    if(_cake_node_HAS_PYTHON)
        _cake_generate_python_node(${NODE_NAME} ${NODE_DIR} ${_cake_node_INTERFACE_YAML})
    endif()
endmacro()

# Generate C++ interface, parameters, and component registration Creates interface header, parameter library, and
# registers as rclcpp_component. Plugin class follows pattern: ${PROJECT_NAME}::${NODE_NAME}::${NodeNamePascal} Args:
# NODE_NAME - Name of the node INTERFACE_YAML - Path to interface.yaml file
macro(_cake_generate_cpp_node NODE_NAME INTERFACE_YAML)
    find_package(rclcpp REQUIRED)
    find_package(rclcpp_components REQUIRED)
    find_package(generate_parameter_library REQUIRED)

    set(_cake_node_LIB_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME})
    file(MAKE_DIRECTORY ${_cake_node_LIB_INCLUDE_DIR})

    set(_cake_node_INTERFACE_HEADER_FILE ${_cake_node_LIB_INCLUDE_DIR}/${NODE_NAME}_interface.hpp)
    set(_cake_node_INTERFACE_PARAMS_FILE ${_cake_node_LIB_INCLUDE_DIR}/${NODE_NAME}_interface.params.yaml)
    set(_cake_node_REGISTRATION_CPP_FILE ${_cake_node_LIB_INCLUDE_DIR}/${NODE_NAME}_registration.cpp)
    set(_cake_node_INTERFACE_YAML_FILE ${_cake_node_LIB_INCLUDE_DIR}/${NODE_NAME}.yaml)

    set(
        _cake_node_CODEGEN_CMD
        ${_cake_codegen_script_BIN}
        ${INTERFACE_YAML}
        --language
        cpp
        --package
        ${PROJECT_NAME}
        --node-name
        ${NODE_NAME}
        --output
        ${_cake_node_LIB_INCLUDE_DIR}
    )

    add_custom_command(
        OUTPUT ${_cake_node_INTERFACE_HEADER_FILE} ${_cake_node_INTERFACE_PARAMS_FILE}
               ${_cake_node_REGISTRATION_CPP_FILE} ${_cake_node_INTERFACE_YAML_FILE}
        COMMAND ${_cake_node_CODEGEN_CMD}
        DEPENDS ${INTERFACE_YAML}
        DEPENDS ${_cake_codegen_script_BIN}
        COMMENT "Generating C++ interface for node '${NODE_NAME}'"
        VERBATIM
    )

    set(_cake_node_INTERFACE_LIB_NAME "${NODE_NAME}_interface")
    add_library(${_cake_node_INTERFACE_LIB_NAME} INTERFACE ${_cake_node_INTERFACE_HEADER_FILE})
    target_include_directories(
        ${_cake_node_INTERFACE_LIB_NAME} INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
                                                   $<INSTALL_INTERFACE:include>
    )
    set_target_properties(${_cake_node_INTERFACE_LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)
    target_link_libraries(${_cake_node_INTERFACE_LIB_NAME} INTERFACE rclcpp::rclcpp)

    # Install the generated headers
    install(DIRECTORY ${_cake_node_LIB_INCLUDE_DIR} DESTINATION include)

    # Export dependencies
    ament_export_dependencies(rclcpp)

    target_link_libraries(${CAKE_CPP_PACKAGE_TARGET} ${_cake_node_INTERFACE_LIB_NAME})

    message(STATUS "cake: Generated C++ interface library for node '${NODE_NAME}'")

    # generate_parameter_library expects a path relative to CMAKE_CURRENT_SOURCE_DIR Compute the relative path from
    # source to binary dir
    file(RELATIVE_PATH _cake_node_INTERFACE_PARAMS_FILE_REL ${CMAKE_CURRENT_SOURCE_DIR}
         ${_cake_node_INTERFACE_PARAMS_FILE}
    )

    set(_cake_node_PARAMETERS_LIB_NAME "${NODE_NAME}_parameters")
    generate_parameter_library(${_cake_node_PARAMETERS_LIB_NAME} ${_cake_node_INTERFACE_PARAMS_FILE_REL})

    # Make parameters library depend on interface library (which generates the .params.yaml)
    add_dependencies(${_cake_node_PARAMETERS_LIB_NAME} ${_cake_node_INTERFACE_LIB_NAME})

    target_link_libraries(${CAKE_CPP_PACKAGE_TARGET} ${_cake_node_PARAMETERS_LIB_NAME})
    message(STATUS "cake: Generated parameters library for node '${NODE_NAME}'")

    # Add the generated registration .cpp file to the package target
    target_sources(${CAKE_CPP_PACKAGE_TARGET} PRIVATE ${_cake_node_REGISTRATION_CPP_FILE})

    # Add source directory to include path so registration file can find node headers
    target_include_directories(${CAKE_CPP_PACKAGE_TARGET} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

    # Ensure the registration file is generated before compiling
    add_dependencies(${CAKE_CPP_PACKAGE_TARGET} ${_cake_node_INTERFACE_LIB_NAME})

    # Register the node as an rclcpp component Convention: ${PROJECT_NAME}::${NodeNamePascal}
    _cake_snake_to_pascal(_cake_node_CLASS_NAME ${NODE_NAME})
    set(_cake_node_PLUGIN_CLASS "${PROJECT_NAME}::${_cake_node_CLASS_NAME}")

    rclcpp_components_register_node(
        ${CAKE_CPP_PACKAGE_TARGET} PLUGIN ${_cake_node_PLUGIN_CLASS} EXECUTABLE ${NODE_NAME}
    )
    message(STATUS "cake: Registered component '${_cake_node_PLUGIN_CLASS}' with executable '${NODE_NAME}'")

    # Install the generated interface YAML file
    install(FILES ${_cake_node_INTERFACE_YAML_FILE} DESTINATION share/${PROJECT_NAME}/interfaces)
endmacro()

# Generate Python interface and executable wrapper Creates _interface.py, _parameters.py, and executable using
# runpy.run_module(). Installs to ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/${NODE_NAME}/ Args: NODE_NAME - Name of the node
# NODE_DIR - Full path to node directory INTERFACE_YAML - Path to interface.yaml file
macro(_cake_generate_python_node NODE_NAME NODE_DIR INTERFACE_YAML)
    set(_cake_node_PYTHON_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/python_generated/${NODE_NAME})
    file(MAKE_DIRECTORY ${_cake_node_PYTHON_GEN_DIR})

    # Generated files
    set(_cake_node_INTERFACE_PY ${_cake_node_PYTHON_GEN_DIR}/interface.py)
    set(_cake_node_PARAMETERS_INTERNAL_PY ${_cake_node_PYTHON_GEN_DIR}/_parameters.py)
    set(_cake_node_PARAMETERS_PY ${_cake_node_PYTHON_GEN_DIR}/parameters.py)
    set(_cake_node_INIT_PY ${_cake_node_PYTHON_GEN_DIR}/__init__.py)
    set(_cake_node_INTERFACE_YAML_FILE ${_cake_node_PYTHON_GEN_DIR}/${NODE_NAME}.yaml)

    set(
        _cake_node_CODEGEN_CMD
        ${_cake_codegen_script_BIN}
        ${INTERFACE_YAML}
        --language
        python
        --package
        ${PROJECT_NAME}
        --node-name
        ${NODE_NAME}
        --output
        ${_cake_node_PYTHON_GEN_DIR}
    )

    add_custom_command(
        OUTPUT ${_cake_node_INTERFACE_PY} ${_cake_node_PARAMETERS_INTERNAL_PY} ${_cake_node_PARAMETERS_PY}
               ${_cake_node_INIT_PY} ${_cake_node_INTERFACE_YAML_FILE}
        COMMAND ${_cake_node_CODEGEN_CMD}
        DEPENDS ${INTERFACE_YAML}
        DEPENDS ${_cake_codegen_script_BIN}
        COMMENT "Generating Python interface for node '${NODE_NAME}'"
        VERBATIM
    )

    add_custom_target(
        ${NODE_NAME}_interface ALL
        DEPENDS ${_cake_node_INTERFACE_PY} ${_cake_node_PARAMETERS_INTERNAL_PY} ${_cake_node_PARAMETERS_PY}
                ${_cake_node_INIT_PY} ${_cake_node_INTERFACE_YAML_FILE}
    )

    # Install user Python files to site-packages/${PROJECT_NAME}/${NODE_NAME}/
    file(GLOB _cake_node_USER_PY_FILES "${NODE_DIR}/*.py")
    if(_cake_node_USER_PY_FILES)
        install(FILES ${_cake_node_USER_PY_FILES} DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/${NODE_NAME}")
        message(STATUS "cake: Installed user Python files for node '${NODE_NAME}'")
    endif()

    # Install generated Python interface files to site-packages/${PROJECT_NAME}/${NODE_NAME}/
    install(
        DIRECTORY ${_cake_node_PYTHON_GEN_DIR}/
        DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}/${NODE_NAME}"
        FILES_MATCHING
        PATTERN "*.py"
    )

    # Generate wrapper executable using runpy.run_module()
    set(_cake_node_EXECUTABLE_PATH "${CMAKE_CURRENT_BINARY_DIR}/executables/${NODE_NAME}")
    file(
        WRITE ${_cake_node_EXECUTABLE_PATH}
        "#!/usr/bin/env python3
import runpy

# Run the node's main file as __main__
runpy.run_module('${PROJECT_NAME}.${NODE_NAME}.${NODE_NAME}', run_name='__main__')
"
    )

    # Make executable
    file(
        CHMOD
        ${_cake_node_EXECUTABLE_PATH}
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
    install(PROGRAMS ${_cake_node_EXECUTABLE_PATH} DESTINATION lib/${PROJECT_NAME})

    # Install the generated interface YAML file
    install(FILES ${_cake_node_INTERFACE_YAML_FILE} DESTINATION share/${PROJECT_NAME}/interfaces)
endmacro()

# Process and install top-level interface.yaml files from optional interfaces/ directory. Per-node interface.yaml files
# are now processed and installed by the code generation script. Args: NODES_DIR - Path to nodes/ directory containing
# node subdirectories
macro(_cake_process_and_install_interfaces NODES_DIR)
    set(_cake_interfaces_output_dir "${CMAKE_CURRENT_BINARY_DIR}/interfaces")
    file(MAKE_DIRECTORY ${_cake_interfaces_output_dir})

    # Collect expected generated node YAML names (for conflict checking)
    set(_cake_generated_node_yaml_names "")
    file(GLOB _cake_interface_NODE_DIRS RELATIVE ${NODES_DIR} ${NODES_DIR}/*)
    foreach(_cake_interface_NODE_ENTRY ${_cake_interface_NODE_DIRS})
        set(_cake_interface_NODE_PATH "${NODES_DIR}/${_cake_interface_NODE_ENTRY}")
        if(IS_DIRECTORY ${_cake_interface_NODE_PATH})
            set(_cake_interface_YAML_PATH "${_cake_interface_NODE_PATH}/interface.yaml")
            if(EXISTS ${_cake_interface_YAML_PATH})
                list(APPEND _cake_generated_node_yaml_names "${_cake_interface_NODE_ENTRY}.yaml")
            endif()
        endif()
    endforeach()

    # Process top-level interfaces/ directory if it exists
    set(_cake_toplevel_interfaces_dir "${CMAKE_CURRENT_SOURCE_DIR}/interfaces")
    set(_cake_interfaces_files_to_install "")
    if(IS_DIRECTORY ${_cake_toplevel_interfaces_dir})
        file(GLOB _cake_toplevel_interface_files "${_cake_toplevel_interfaces_dir}/*.yaml")

        foreach(_cake_toplevel_interface_file ${_cake_toplevel_interface_files})
            get_filename_component(_cake_interface_filename ${_cake_toplevel_interface_file} NAME)

            # Check for naming conflicts with generated node YAML files
            list(FIND _cake_generated_node_yaml_names "${_cake_interface_filename}" _cake_conflict_index)
            if(NOT _cake_conflict_index EQUAL -1)
                message(
                    FATAL_ERROR
                        "cake: Naming conflict detected! Package-level interfaces/${_cake_interface_filename} conflicts with auto-generated interface YAML for a node. Please rename the package-level file."
                )
            endif()

            # Copy the file to the build directory
            set(_cake_interface_output_file "${_cake_interfaces_output_dir}/${_cake_interface_filename}")
            configure_file(${_cake_toplevel_interface_file} ${_cake_interface_output_file} COPYONLY)

            list(APPEND _cake_interfaces_files_to_install ${_cake_interface_output_file})

            message(STATUS "cake: Processed package-level interfaces/${_cake_interface_filename}")
        endforeach()
    endif()

    # Install top-level interface files
    if(_cake_interfaces_files_to_install)
        list(LENGTH _cake_interfaces_files_to_install _cake_interfaces_count)
        install(FILES ${_cake_interfaces_files_to_install} DESTINATION share/${PROJECT_NAME}/interfaces)
        message(
            STATUS
                "cake: Installing ${_cake_interfaces_count} package-level interface files to share/${PROJECT_NAME}/interfaces/"
        )
    endif()
endmacro()

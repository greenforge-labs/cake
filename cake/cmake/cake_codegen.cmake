# CMake module for cake code generation

macro(cake_generate_node_interface LIB_NAME YAML_FILE NODE_NAME LANGUAGE)
    # LANGUAGE is now a required fourth argument: "cpp" or "python"
    unset(cake_codegen_script_BIN CACHE) # Unset the cache variable
    find_program(
        cake_codegen_script_BIN
        NAMES "generate_node_interface.py"
        PATHS "${cake_DIR}/../../../lib/cake"
        NO_DEFAULT_PATH
    )
    if(NOT cake_codegen_script_BIN)
        message(FATAL_ERROR "Could not find generate_node_interface.py in ${cake_DIR}/../../../lib/cake")
    endif()

    # Make the include directory
    set(LIB_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME})
    file(MAKE_DIRECTORY ${LIB_INCLUDE_DIR})

    # Set the yaml file parameter to be relative to the current source dir
    set(YAML_FILE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/${YAML_FILE})

    if(NOT EXISTS "${YAML_FILE_PATH}")
        message(FATAL_ERROR "Interface YAML file not found: ${YAML_FILE_PATH}")
    endif()

    # Dispatch based on language Note: Using string() for robust comparison
    string(TOLOWER "${LANGUAGE}" LANGUAGE_LOWER)
    if(LANGUAGE_LOWER STREQUAL "python")
        _cake_generate_python_interface(${LIB_NAME} ${YAML_FILE_PATH} ${NODE_NAME})
    elseif(LANGUAGE_LOWER STREQUAL "cpp")
        _cake_generate_cpp_interface(${LIB_NAME} ${YAML_FILE_PATH} ${NODE_NAME})
    else()
        message(FATAL_ERROR "cake: Invalid LANGUAGE '${LANGUAGE}'. Must be 'cpp' or 'python'")
    endif()
endmacro()

# Internal: Generate C++ interface
macro(_cake_generate_cpp_interface LIB_NAME YAML_FILE_PATH NODE_NAME)
    # Make the include directory
    set(LIB_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME})
    file(MAKE_DIRECTORY ${LIB_INCLUDE_DIR})

    # Set the output file names based on node name
    set(INTERFACE_HEADER_FILE ${LIB_INCLUDE_DIR}/${NODE_NAME}_interface.hpp)
    set(INTERFACE_PARAMS_FILE ${LIB_INCLUDE_DIR}/${NODE_NAME}_interface.params.yaml)

    # Build the command with new unified CLI
    set(
        CODEGEN_CMD
        ${cake_codegen_script_BIN}
        ${YAML_FILE_PATH}
        --language
        cpp
        --package
        ${PROJECT_NAME}
        --node-name
        ${NODE_NAME}
        --output
        ${LIB_INCLUDE_DIR}
    )

    # Generate the header and params files for the library
    add_custom_command(
        OUTPUT ${INTERFACE_HEADER_FILE} ${INTERFACE_PARAMS_FILE}
        COMMAND ${CODEGEN_CMD}
        DEPENDS ${YAML_FILE_PATH}
        COMMENT "Generating C++ interface for node '${NODE_NAME}'"
        VERBATIM
    )

    # Create the INTERFACE library target
    add_library(${LIB_NAME} INTERFACE ${INTERFACE_HEADER_FILE})
    target_include_directories(
        ${LIB_NAME} INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include> $<INSTALL_INTERFACE:include>
    )
    set_target_properties(${LIB_NAME} PROPERTIES LINKER_LANGUAGE CXX)
    target_link_libraries(${LIB_NAME} INTERFACE rclcpp::rclcpp)

    # Install the generated headers
    install(DIRECTORY ${LIB_INCLUDE_DIR} DESTINATION include)

    # Export dependencies
    ament_export_dependencies(rclcpp)
endmacro()

# Internal: Generate Python interface
macro(_cake_generate_python_interface MODULE_NAME YAML_FILE_PATH NODE_NAME)
    # Output directory for generated Python files
    set(PYTHON_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/python_generated/${NODE_NAME})
    file(MAKE_DIRECTORY ${PYTHON_GEN_DIR})

    # Expected generated files
    set(INTERFACE_PY ${PYTHON_GEN_DIR}/_interface.py)
    set(PARAMETERS_PY ${PYTHON_GEN_DIR}/_parameters.py)
    set(INIT_PY ${PYTHON_GEN_DIR}/__init__.py)

    # Build codegen command
    set(
        CODEGEN_CMD
        ${cake_codegen_script_BIN}
        ${YAML_FILE_PATH}
        --language
        python
        --package
        ${PROJECT_NAME}
        --node-name
        ${NODE_NAME}
        --output
        ${PYTHON_GEN_DIR}
    )

    # Generate Python interface files
    add_custom_command(
        OUTPUT ${INTERFACE_PY} ${PARAMETERS_PY} ${INIT_PY}
        COMMAND ${CODEGEN_CMD}
        DEPENDS ${YAML_FILE_PATH}
        COMMENT "Generating Python interface for node '${NODE_NAME}'"
        VERBATIM
    )

    # Create target to trigger generation
    add_custom_target(${MODULE_NAME} ALL DEPENDS ${INTERFACE_PY} ${PARAMETERS_PY} ${INIT_PY})
endmacro()

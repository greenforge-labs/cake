# CMake module for cake code generation

macro(cake_generate_node_interface LIB_NAME YAML_FILE NODE_NAME)
    # NODE_NAME is now a required third argument
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
        COMMENT "Running `${CODEGEN_CMD}`"
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

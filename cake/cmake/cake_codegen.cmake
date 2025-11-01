# CMake module for cake code generation

macro(cake_generate_node_interface LIB_NAME YAML_FILE)
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

    # Set the output header file name
    set(INTERFACE_HEADER_FILE ${LIB_INCLUDE_DIR}/${LIB_NAME}.hpp)

    # Generate the header for the library
    add_custom_command(
        OUTPUT ${INTERFACE_HEADER_FILE}
        COMMAND ${cake_codegen_script_BIN} ${INTERFACE_HEADER_FILE} ${YAML_FILE_PATH}
        DEPENDS ${YAML_FILE_PATH}
        COMMENT "Running `${cake_codegen_script_BIN} ${INTERFACE_HEADER_FILE} ${YAML_FILE_PATH}`"
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

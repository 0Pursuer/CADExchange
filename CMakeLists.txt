cmake_minimum_required(VERSION 3.10)

if(POLICY CMP0091)
    cmake_policy(SET CMP0091 NEW)
endif()

project(CADExchange LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fix for "too many sections" error with MinGW/GCC
if(MINGW OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    add_compile_options(-Wa,-mbig-obj)
endif()

if(MSVC)
    add_compile_options(/bigobj)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

option(CADEXCHANGE_BUILD_EXAMPLES "Build CADExchange example executables" ON)
option(CADEXCHANGE_BUILD_PYTHON_BINDINGS "Build CADExchange Python bindings" OFF)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_CURRENT_SOURCE_DIR}/service/builders
    ${CMAKE_CURRENT_SOURCE_DIR}/service/serialization
    ${CMAKE_CURRENT_SOURCE_DIR}/service/accessors
    ${CMAKE_CURRENT_SOURCE_DIR}/service/api/py
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdParty
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Library target (Internal Service Layer)
add_library(cadexchange STATIC
    core/UnitConverter.cpp
    service/serialization/SerializationRegistry.cpp
    service/serialization/TinyXMLSerializer.cpp
    service/validation/ModelValidator.cpp
    thirdParty/tinyxml2/tinyxml2.cpp
)

target_include_directories(cadexchange PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_CURRENT_SOURCE_DIR}/service/builders
    ${CMAKE_CURRENT_SOURCE_DIR}/service/serialization
    ${CMAKE_CURRENT_SOURCE_DIR}/service/accessors
    ${CMAKE_CURRENT_SOURCE_DIR}/service/validation
    ${CMAKE_CURRENT_SOURCE_DIR}/service/api/py
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdParty
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set_target_properties(cadexchange PROPERTIES
    MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/$<CONFIG>"
)

if(CADEXCHANGE_BUILD_EXAMPLES)
    # Demo Application
    add_executable(PartReconstructionDemo examples/PartReconstructionDemo.cpp)
    target_include_directories(PartReconstructionDemo PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core
        ${CMAKE_CURRENT_SOURCE_DIR}/service/builders
        ${CMAKE_CURRENT_SOURCE_DIR}/service/serialization
        ${CMAKE_CURRENT_SOURCE_DIR}/service/accessors
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdParty
    )
    target_link_libraries(PartReconstructionDemo PRIVATE cadexchange)

    # Recommended Approach Demo (Individual Builders)
    add_executable(RecommendedApproach examples/RecommendedApproach.cpp)
    target_link_libraries(RecommendedApproach PRIVATE cadexchange)

    add_executable(MigrationRegressionTest examples/MigrationRegressionTest.cpp)
    target_link_libraries(MigrationRegressionTest PRIVATE cadexchange)

    add_executable(PythonBindingSmokeTest examples/PythonBindingSmokeTest.cpp)
    target_link_libraries(PythonBindingSmokeTest PRIVATE cadexchange)

    add_executable(test_geom examples/test_geom.cpp)
    target_link_libraries(test_geom PRIVATE cadexchange)

    # add_executable(BuilderDemoAdvanced examples/BuilderDemoAdvanced.cpp)
    # target_link_libraries(BuilderDemoAdvanced PRIVATE cadexchange)

    # add_executable(DirectModelDemo examples/DirectModelDemo.cpp)
    # target_link_libraries(DirectModelDemo PRIVATE cadexchange)
endif()

if(CADEXCHANGE_BUILD_PYTHON_BINDINGS)
    find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION OFF)
    set(PYBIND11_LTO_CXX_FLAGS "" CACHE STRING "" FORCE)
    set(PYBIND11_LTO_LINKER_FLAGS "" CACHE STRING "" FORCE)
    find_package(pybind11 CONFIG REQUIRED)

    pybind11_add_module(cadexchange_py
        service/api/py/module.cpp
    )

    target_include_directories(cadexchange_py PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/core
        ${CMAKE_CURRENT_SOURCE_DIR}/service/accessors
        ${CMAKE_CURRENT_SOURCE_DIR}/service/serialization
        ${CMAKE_CURRENT_SOURCE_DIR}/service/api/py
        ${CMAKE_CURRENT_SOURCE_DIR}/service/validation
        ${CMAKE_CURRENT_SOURCE_DIR}/thirdParty
    )

    target_link_libraries(cadexchange_py PRIVATE cadexchange)

    set_target_properties(cadexchange_py PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python/$<CONFIG>"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python/$<CONFIG>"
    )
endif()

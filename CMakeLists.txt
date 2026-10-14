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
    add_compile_options(/bigobj /utf-8 /EHsc)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

option(CADEXCHANGE_BUILD_EXAMPLES "Build CADExchange example executables" ON)
option(CADEXCHANGE_BUILD_PYTHON_BINDINGS "Build CADExchange Python bindings" OFF)
option(CADEXCHANGE_BUILD_STEP_COMPARE
       "Build the independent OCCT STEP final-solid comparator" OFF)

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
    service/geometry/GeometryCompareHelpers.cpp
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

include(CTest)
add_executable(cadexchange_validate tools/model_validate/main.cpp)
target_link_libraries(cadexchange_validate PRIVATE cadexchange)
if(BUILD_TESTING)
    add_executable(hole_model_tests tests/hole_model_tests.cpp)
    target_compile_definitions(hole_model_tests PRIVATE ENABLE_CEREAL_SERIALIZATION)
    target_link_libraries(hole_model_tests PRIVATE cadexchange)
    add_test(NAME hole_model_contract COMMAND hole_model_tests "${CMAKE_CURRENT_SOURCE_DIR}/../tmp/hole-model-tests")
endif()

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

    add_executable(serialization_precision_tests tests/serialization_precision_tests.cpp)
    target_link_libraries(serialization_precision_tests PRIVATE cadexchange)

    add_executable(PythonBindingSmokeTest examples/PythonBindingSmokeTest.cpp)
    target_link_libraries(PythonBindingSmokeTest PRIVATE cadexchange)

    add_executable(test_geom examples/test_geom.cpp)
    target_link_libraries(test_geom PRIVATE cadexchange)

    # add_executable(BuilderDemoAdvanced examples/BuilderDemoAdvanced.cpp)
    # target_link_libraries(BuilderDemoAdvanced PRIVATE cadexchange)

    # add_executable(DirectModelDemo examples/DirectModelDemo.cpp)
    # target_link_libraries(DirectModelDemo PRIVATE cadexchange)
endif()

if(CADEXCHANGE_BUILD_STEP_COMPARE)
    find_package(OpenCASCADE CONFIG REQUIRED)

    add_library(cad_step_compare_core STATIC
        tools/step_compare/StepCompare.cpp
        tools/step_compare/domain/DecisionCode.cpp
        tools/step_compare/domain/DescriptorMatching.cpp
        tools/step_compare/domain/TolerancePolicy.cpp
        tools/step_compare/geometry/ArtifactWriter.cpp
        tools/step_compare/geometry/BooleanVerifier.cpp
        tools/step_compare/geometry/ShapeAudit.cpp
        tools/step_compare/geometry/ShapeNormalizer.cpp
        tools/step_compare/geometry/StepLoader.cpp
        tools/step_compare/pipeline/MultiSolidPipeline.cpp
        tools/step_compare/pipeline/SolidPairPipeline.cpp
        tools/step_compare/report/HumanReport.cpp
        tools/step_compare/report/JsonReport.cpp
        tools/step_compare/report/ResultWriter.cpp
    )
    target_compile_definitions(cad_step_compare_core PUBLIC CADEXCHANGE_HAS_OCCT)
    target_include_directories(cad_step_compare_core
        PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}/tools/step_compare
        PRIVATE
            ${OpenCASCADE_INCLUDE_DIR}
            ${OpenCASCADE_INCLUDE_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}/thirdParty
    )
    target_link_libraries(cad_step_compare_core PRIVATE ${OpenCASCADE_LIBRARIES})

    set_target_properties(cad_step_compare_core PROPERTIES
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
    )

    add_executable(cad_step_compare tools/step_compare/main.cpp)
    target_link_libraries(cad_step_compare PRIVATE cad_step_compare_core)
    set_target_properties(cad_step_compare PROPERTIES
        MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>"
    )

    include(CTest)
    if(BUILD_TESTING)
        add_executable(cad_step_compare_tests
            tests/step_compare_tests.cpp
        )
        target_include_directories(cad_step_compare_tests PRIVATE
            ${OpenCASCADE_INCLUDE_DIR}
            ${OpenCASCADE_INCLUDE_DIRS}
        )
        target_link_libraries(cad_step_compare_tests
            PRIVATE
                cad_step_compare_core
                ${OpenCASCADE_LIBRARIES}
        )
        set_target_properties(cad_step_compare_tests PROPERTIES
            MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>"
        )
        add_test(
            NAME cad_step_compare_tests
            COMMAND cad_step_compare_tests
                    "${CMAKE_CURRENT_SOURCE_DIR}/../tmp/geometry_compare/unit"
        )
    endif()
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

if(BUILD_TESTING)
    add_executable(body_mirror_model_tests tests/body_mirror_model_tests.cpp)
    target_compile_definitions(body_mirror_model_tests PRIVATE ENABLE_CEREAL_SERIALIZATION)
    target_link_libraries(body_mirror_model_tests PRIVATE cadexchange)
    add_test(NAME body_mirror_model_contract COMMAND body_mirror_model_tests "${CMAKE_CURRENT_SOURCE_DIR}/../tmp/body-mirror-model-tests")
endif()

if(BUILD_TESTING AND CADEXCHANGE_BUILD_PYTHON_BINDINGS)
    set_tests_properties(body_mirror_model_contract PROPERTIES FIXTURES_SETUP body_mirror_fixtures)
    add_test(NAME body_mirror_python_contract
             COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/tests/body_mirror_python_tests.py"
                     "$<TARGET_FILE_DIR:cadexchange_py>" "${CMAKE_CURRENT_SOURCE_DIR}/../tmp/body-mirror-model-tests")
    set_tests_properties(body_mirror_python_contract PROPERTIES FIXTURES_REQUIRED body_mirror_fixtures)
endif()

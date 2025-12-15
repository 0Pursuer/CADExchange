cmake_minimum_required(VERSION 3.10)
project(CADExchange LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fix for "too many sections" error with MinGW/GCC
if(MINGW OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    add_compile_options(-Wa,-mbig-obj)
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_CURRENT_SOURCE_DIR}/service/builders
    ${CMAKE_CURRENT_SOURCE_DIR}/service/serialization
    ${CMAKE_CURRENT_SOURCE_DIR}/service/accessors
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdParty
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Library target (Internal Service Layer)
add_library(cadexchange STATIC
    service/serialization/SerializationRegistry.cpp
    service/serialization/TinyXMLSerializer.cpp
    thirdParty/tinyxml2/tinyxml2.cpp
)

target_include_directories(cadexchange PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_CURRENT_SOURCE_DIR}/service/builders
    ${CMAKE_CURRENT_SOURCE_DIR}/service/serialization
    ${CMAKE_CURRENT_SOURCE_DIR}/service/accessors
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdParty
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# DLL Target (API Export)
add_library(CADExchangeDLL SHARED
    src/api/CAD_API.cpp
)
target_include_directories(CADExchangeDLL PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(CADExchangeDLL PRIVATE cadexchange)
target_compile_definitions(CADExchangeDLL PRIVATE CADEXCHANGE_EXPORTS=1)

# Tests (Internal - using core/builders directly)
add_executable(ReadPartTest test/SolidWorks/ReadPartTest.cpp)
target_include_directories(ReadPartTest PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/core 
    ${CMAKE_CURRENT_SOURCE_DIR}/service/builders 
    ${CMAKE_CURRENT_SOURCE_DIR}/service/serialization 
    ${CMAKE_CURRENT_SOURCE_DIR}/service/accessors
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdParty
)
target_link_libraries(ReadPartTest PRIVATE cadexchange)

add_executable(WritePartTest test/SolidWorks/WritePartTest.cpp)
target_include_directories(WritePartTest PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/core 
    ${CMAKE_CURRENT_SOURCE_DIR}/service/builders 
    ${CMAKE_CURRENT_SOURCE_DIR}/service/serialization 
    ${CMAKE_CURRENT_SOURCE_DIR}/service/accessors
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdParty
)
target_link_libraries(WritePartTest PRIVATE cadexchange)

add_executable(UsageExampleTest test/UsageExampleTest.cpp)
target_include_directories(UsageExampleTest PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/core 
    ${CMAKE_CURRENT_SOURCE_DIR}/service/builders 
    ${CMAKE_CURRENT_SOURCE_DIR}/service/serialization 
    ${CMAKE_CURRENT_SOURCE_DIR}/service/accessors
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdParty
)
target_link_libraries(UsageExampleTest PRIVATE cadexchange)

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

# API Tests (Static Linking - using SDK interface)
add_executable(APIReadTest test/APITest/ReadTest.cpp)
target_include_directories(APIReadTest PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(APIReadTest PRIVATE CADExchangeDLL)

add_executable(APIWriteTest test/APITest/WriteTest.cpp)
target_include_directories(APIWriteTest PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(APIWriteTest PRIVATE CADExchangeDLL)

# CTest Integration
option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
    enable_testing()
    
    # Internal tests
    add_test(NAME ReadPartTest COMMAND ReadPartTest)
    add_test(NAME WritePartTest COMMAND WritePartTest)
    add_test(NAME UsageExampleTest COMMAND UsageExampleTest)
    
    # API tests
    add_test(NAME APIReadTest COMMAND APIReadTest)
    add_test(NAME APIWriteTest COMMAND APIWriteTest)
endif()

## use : cd ..; cmake --build . --config Release; cd Release; .\ReadPartTest.exe

cmake_minimum_required(VERSION 3.10)
project(CADExchange LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_CURRENT_SOURCE_DIR}/builders
    ${CMAKE_CURRENT_SOURCE_DIR}/serialization
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdParty
)

# Library target (currently only contains registration/serialization helpers)
add_library(cadexchange STATIC
    serialization/SerializationRegistry.cpp
    serialization/TinyXMLSerializer.cpp
    builders/ReferenceFactory.cpp
    thirdParty/tinyxml2/tinyxml2.cpp
)

#include directories for cadexchange target
target_include_directories(cadexchange PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/core
    ${CMAKE_CURRENT_SOURCE_DIR}/builders
    ${CMAKE_CURRENT_SOURCE_DIR}/serialization
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdParty
)
# Tests
add_executable(ReadPartTest test/SolidWorks/ReadPartTest.cpp)
target_include_directories(ReadPartTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/core ${CMAKE_CURRENT_SOURCE_DIR}/builders ${CMAKE_CURRENT_SOURCE_DIR}/serialization ${CMAKE_CURRENT_SOURCE_DIR}/thirdParty)
target_link_libraries(ReadPartTest PRIVATE cadexchange)

add_executable(WritePartTest test/SolidWorks/WritePartTest.cpp)
target_include_directories(WritePartTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/core ${CMAKE_CURRENT_SOURCE_DIR}/builders ${CMAKE_CURRENT_SOURCE_DIR}/serialization ${CMAKE_CURRENT_SOURCE_DIR}/thirdParty)
target_link_libraries(WritePartTest PRIVATE cadexchange)

add_executable(UsageExampleTest test/UsageExampleTest.cpp)
target_include_directories(UsageExampleTest PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/core ${CMAKE_CURRENT_SOURCE_DIR}/builders ${CMAKE_CURRENT_SOURCE_DIR}/serialization ${CMAKE_CURRENT_SOURCE_DIR}/thirdParty)
target_link_libraries(UsageExampleTest PRIVATE cadexchange)

# Enable CTest integration
option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
    enable_testing()
    add_test(NAME ReadPartTest COMMAND ReadPartTest)
    add_test(NAME WritePartTest COMMAND WritePartTest)
    add_test(NAME UsageExampleTest COMMAND UsageExampleTest)
endif()

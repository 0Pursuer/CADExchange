## use : cd ..; cmake --build . --config Release; cd Release; .\ReadPartTest.exe

cmake_minimum_required(VERSION 3.10)
project(CADExchange LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/thirdParty)

# Library target (currently only contains registration/serialization helpers)
add_library(cadexchange STATIC
    src/SerializationRegistry.cpp
    src/TinyXMLSerializer.cpp
    thirdParty/tinyxml2/tinyxml2.cpp
)

#include directories for cadexchange target
target_include_directories(cadexchange PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/thirdParty
)

# Tests
add_executable(ReadPartTest test/SolidWorks/ReadPartTest.cpp)
target_include_directories(ReadPartTest PRIVATE ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/thirdParty)
target_link_libraries(ReadPartTest PRIVATE cadexchange)

add_executable(WritePartTest test/SolidWorks/WritePartTest.cpp)
target_include_directories(WritePartTest PRIVATE ${CMAKE_SOURCE_DIR}/include ${CMAKE_SOURCE_DIR}/thirdParty)
target_link_libraries(WritePartTest PRIVATE cadexchange)

# Enable CTest integration
option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
    enable_testing()
    add_test(NAME ReadPartTest COMMAND ReadPartTest)
    add_test(NAME WritePartTest COMMAND WritePartTest)
endif()

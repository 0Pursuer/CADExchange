cmake_minimum_required(VERSION 3.10)
project(CADExchange LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fix for "too many sections" error with MinGW/GCC
if(MINGW OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    add_compile_options(-Wa,-mbig-obj)
endif()

if(MSVC)
    add_compile_options(/bigobj)
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

# add_executable(BuilderDemoAdvanced examples/BuilderDemoAdvanced.cpp)
# target_link_libraries(BuilderDemoAdvanced PRIVATE cadexchange)

# add_executable(DirectModelDemo examples/DirectModelDemo.cpp)
# target_link_libraries(DirectModelDemo PRIVATE cadexchange)

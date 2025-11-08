cmake_minimum_required(VERSION 3.16)
project(OrthophotoGUI VERSION 2.0 LANGUAGES CXX)

# C++ standard configuration
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Qt configuration
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Build type optimization
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Compiler optimizations for Release builds
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    if(MSVC)
        add_compile_options(/O2 /GL)
        add_link_options(/LTCG)
    else()
        add_compile_options(-O3 -march=native)
    endif()
endif()

# Find dependencies
find_package(Qt6 REQUIRED COMPONENTS Widgets)
find_package(VTK REQUIRED COMPONENTS
    CommonCore
    CommonDataModel
    RenderingCore
    RenderingOpenGL2
)
find_package(PCL REQUIRED COMPONENTS
    common
    io
    surface
    visualization
)
find_package(PDAL REQUIRED)

# Create executable
add_executable(orthophoto_gui orthophoto_gui.cpp)

# Set target properties
set_target_properties(orthophoto_gui PROPERTIES
    WIN32_EXECUTABLE ON
    OUTPUT_NAME "orthophoto_gui"
)

# Link libraries
target_link_libraries(orthophoto_gui PRIVATE
    Qt6::Widgets
    ${VTK_LIBRARIES}
    ${PCL_LIBRARIES}
    PDAL::PDAL
)

# Include directories
target_include_directories(orthophoto_gui PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
)

# Compile definitions
target_compile_definitions(orthophoto_gui PRIVATE
    ${PCL_DEFINITIONS}
)

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
    features
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
    ${PDAL_LIBRARIES}
)

# Include directories
target_include_directories(orthophoto_gui PRIVATE
    ${PCL_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
    ${PDAL_INCLUDE_DIRS}
)

# Compile definitions
target_compile_definitions(orthophoto_gui PRIVATE
    ${PCL_DEFINITIONS}
)

# Windows-specific compile definitions to fix max/min macro conflicts
if(WIN32)
    target_compile_definitions(orthophoto_gui PRIVATE
        NOMINMAX
        WIN32_LEAN_AND_MEAN
    )
endif()

# Install target
install(TARGETS orthophoto_gui
    RUNTIME DESTINATION bin
)

# Copy runtime dependencies (DLLs) on Windows
if(WIN32)
    # Determine vcpkg bin directory location
    set(_vcpkg_bin_dir "")
    if(DEFINED VCPKG_INSTALLED_DIR)
        set(_vcpkg_bin_dir "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/bin")
    elseif(DEFINED _VCPKG_INSTALLED_DIR)
        set(_vcpkg_bin_dir "${_VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/bin")
    elseif(DEFINED CMAKE_TOOLCHAIN_FILE)
        # Extract vcpkg root from toolchain file path
        get_filename_component(_vcpkg_root "${CMAKE_TOOLCHAIN_FILE}" DIRECTORY)
        get_filename_component(_vcpkg_root "${_vcpkg_root}" DIRECTORY)
        get_filename_component(_vcpkg_root "${_vcpkg_root}" DIRECTORY)
        set(_vcpkg_bin_dir "${_vcpkg_root}/installed/${VCPKG_TARGET_TRIPLET}/bin")
    endif()
    
    # Get Qt paths
    set(_qt_bin_dir "")
    set(_qt_root_dir "")
    if(TARGET Qt6::Widgets AND TARGET Qt6::qmake)
        get_target_property(_qmake_executable Qt6::qmake IMPORTED_LOCATION)
        get_filename_component(_qt_bin_dir "${_qmake_executable}" DIRECTORY)
        get_filename_component(_qt_root_dir "${_qt_bin_dir}" DIRECTORY)
    endif()
    
    # Install commands for deployment
    if(_vcpkg_bin_dir AND EXISTS "${_vcpkg_bin_dir}")
        message(STATUS "Installing DLLs from: ${_vcpkg_bin_dir}")
        install(DIRECTORY "${_vcpkg_bin_dir}/"
            DESTINATION bin
            FILES_MATCHING PATTERN "*.dll"
        )
    elseif(_vcpkg_bin_dir)
        message(WARNING "vcpkg bin directory not found: ${_vcpkg_bin_dir}")
    endif()
    
    if(_qt_bin_dir AND EXISTS "${_qt_bin_dir}")
        install(DIRECTORY "${_qt_bin_dir}/"
            DESTINATION bin
            FILES_MATCHING PATTERN "*.dll"
        )
        
        if(EXISTS "${_qt_root_dir}/plugins/platforms")
            install(DIRECTORY "${_qt_root_dir}/plugins/platforms"
                DESTINATION bin/plugins
            )
        endif()
        
        if(EXISTS "${_qt_root_dir}/plugins/styles")
            install(DIRECTORY "${_qt_root_dir}/plugins/styles"
                DESTINATION bin/plugins
            )
        endif()
    endif()
    
    # POST_BUILD commands to copy DLLs to build directory
    # This allows running the executable directly from the build directory without installation
    
    # Copy all vcpkg DLLs (including pdalcpp.dll, VTK, PCL, PDAL dependencies)
    if(_vcpkg_bin_dir AND EXISTS "${_vcpkg_bin_dir}")
        add_custom_command(TARGET orthophoto_gui POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${_vcpkg_bin_dir}"
            "$<TARGET_FILE_DIR:orthophoto_gui>"
            COMMENT "Copying runtime DLLs to build directory"
        )
    endif()
    
    # Copy Qt DLLs and plugins to build directory
    if(_qt_bin_dir AND EXISTS "${_qt_bin_dir}")
        # Create plugins directory
        add_custom_command(TARGET orthophoto_gui POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E make_directory
            "$<TARGET_FILE_DIR:orthophoto_gui>/plugins"
            COMMENT "Creating plugins directory"
        )
        
        # Copy Qt DLLs
        add_custom_command(TARGET orthophoto_gui POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_qt_bin_dir}/Qt6Core.dll"
            "${_qt_bin_dir}/Qt6Gui.dll"
            "${_qt_bin_dir}/Qt6Widgets.dll"
            "$<TARGET_FILE_DIR:orthophoto_gui>"
            COMMENT "Copying Qt DLLs to build directory"
        )
        
        # Copy platform plugins
        if(EXISTS "${_qt_root_dir}/plugins/platforms")
            add_custom_command(TARGET orthophoto_gui POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${_qt_root_dir}/plugins/platforms"
                "$<TARGET_FILE_DIR:orthophoto_gui>/plugins/platforms"
                COMMENT "Copying Qt platform plugins to build directory"
            )
        endif()
        
        # Copy style plugins
        if(EXISTS "${_qt_root_dir}/plugins/styles")
            add_custom_command(TARGET orthophoto_gui POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${_qt_root_dir}/plugins/styles"
                "$<TARGET_FILE_DIR:orthophoto_gui>/plugins/styles"
                COMMENT "Copying Qt style plugins to build directory"
            )
        endif()
    endif()
endif()

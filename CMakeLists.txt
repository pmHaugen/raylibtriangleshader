cmake_minimum_required(VERSION 3.10)

if(WIN32)
    # Assuming you're using MSYS2 with Mingw-w64 GCC
    set(CMAKE_CXX_COMPILER "C:/msys64/mingw64/bin/g++.exe")
    message(STATUS "Setting compiler to MSYS2 Mingw-w64 GCC on Windows")
    include_directories(C:/msys64/usr/local/include)
    include_directories(
        "C:/msys64/usr/local/include"
    )
    link_directories("C:/msys64/usr/local/lib")
    


elseif(UNIX)
    message(STATUS "UNIX")
    # Assuming you want to use GCC 14 on Linux
    set(CMAKE_CXX_COMPILER "/usr/bin/g++")
    #set(CMAKE_CXX_COMPILER "/usr/local/gcc-14.1.0/bin/g++-14.1.0")
    # message(STATUS "Setting compiler to GCC 14 on Unix")

endif()

# Project name
project(TriangleSimulation)

# Set C++ standard (if needed for any custom code)
set(CMAKE_CXX_STANDARD 11)

# Find Raylib package (or add your custom path to raylib)
find_package(raylib REQUIRED)

# Add your source files here
add_executable(${PROJECT_NAME} main.cpp)

# Link Raylib to your project
target_link_libraries(${PROJECT_NAME} raylib)

# Optional: Add flags for GCC optimization
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(${PROJECT_NAME} PRIVATE -Ofast -Wall -Wextra)
endif()

# Required for Linux: Link additional libraries
if(UNIX AND NOT APPLE)
    target_link_libraries(${PROJECT_NAME} m pthread dl)
endif()

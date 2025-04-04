cmake_minimum_required(VERSION 3.14) # Updated to a more recent version of CMake
project(TEST)

# Set the build type to Debug if not specified
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

message("Build type: " ${CMAKE_BUILD_TYPE})

# Set C++ standard to C++17 (or C++20 if desired)
set(CMAKE_CXX_STANDARD 17) # Change to 20 for C++20
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

find_package(nlohmann_json 3.10.5 REQUIRED)
# Add OpenCV
set(OpenCV_DIR "/home/jaysszhou/Documents/ThirdParty/opencv-3.4.17/build")
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV INCLUDE: " ${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV LIBS: " ${OpenCV_LIBS})

# Add Eigen3
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 INCLUDE: " ${EIGEN3_INCLUDE_DIR})
message(STATUS "Eigen3 LIBS: " ${EIGEN3_LIBS})

# Add Pangolin
find_package(Pangolin REQUIRED)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
)

# Set output directories
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/out)

# Add the shared library
add_library(${PROJECT_NAME} SHARED
    src/main.cc
    src/solution.cc
    src/map.cc
    src/COC.cc
)

# Link libraries to the shared library
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
)

# Build examples
add_executable(test example/test.cc)
target_link_libraries(test ${PROJECT_NAME})

add_executable(run src/main.cc)
target_link_libraries(run ${PROJECT_NAME})
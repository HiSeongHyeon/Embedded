# =========================
# File: glare_detection/CMakeLists.txt
# =========================
# This CMake file sets up the build environment for the GlareDetection project.
# It uses OpenCV and organizes files into include/ and src/ directories.
#
# ✅ Directory Structure:
# Embedded/
# └── glare_detection/
#     ├── include/
#     │   ├── glare_detector.hpp
#     │   ├── glare_utils.hpp
#     │   └── gps_utils.hpp
#     ├── src/
#     │   ├── main.cpp
#     │   ├── glare_detector.cpp
#     │   ├── glare_utils.cpp
#     │   └── gps_utils.cpp
#     └── CMakeLists.txt
#
# ✅ How to build and run:
# $ cd Embedded/glare_detection
# $ mkdir build && cd build
# $ cmake ..
# $ make
# $ ./GlareDetection

cmake_minimum_required(VERSION 3.10)
project(GlareDetection)

# Use C++14 standard
set(CMAKE_CXX_STANDARD 14)

# Find OpenCV library
find_package(OpenCV REQUIRED)

# Add include directory so headers in include/ can be found
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)

# Define executable target and source files
add_executable(GlareDetection
    src/main.cpp
    src/glare_detector.cpp
    src/glare_utils.cpp
    src/gps_utils.cpp
)

# Link OpenCV libraries to the executable
target_link_libraries(GlareDetection ${OpenCV_LIBS})

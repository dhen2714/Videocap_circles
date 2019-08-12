cmake_minimum_required(VERSION 3.5)
set (CMAKE_CXX_STANDARD 11)
project(VideoCap_circles)

find_package(PkgConfig)
pkg_check_modules(GTKMM gtkmm-3.0)

# Tell cmake we will need dlib.  This command will pull in dlib and compile it
# into your project.  Note that you don't need to compile or install dlib.  All
# cmake needs is the dlib source code folder and it will take care of everything.
add_subdirectory(../dlib-master/dlib dlib_build)

# Load Boost
find_package(Boost 1.58)

# Bring the headers into the project.
include_directories(include ${GTKMM_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
link_directories(${GTKMM_LIBRARY_DIRS})

# Add sources with SET command.
set(SOURCES source/main.cpp source/VideoCap.cpp source/GaussianFit.cpp source/CapApp.cpp)

find_package(OpenCV REQUIRED)
add_executable(VideoCap_circles ${SOURCES})
target_link_libraries(VideoCap_circles dlib::dlib ${OpenCV_LIBS} ${Boost_LIBRARIES} ${GTKMM_LIBRARIES} -lpthread -lboost_system -lboost_thread)

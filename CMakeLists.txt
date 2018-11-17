cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(raw_serial)

set (CMAKE_CXX_STANDARD 11)
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

add_library(RawSerial raw_serial.cpp)
add_library(BusMotor bus_motor.cpp)

add_executable (raw_serial_test main.cpp)

add_dependencies(BusMotor RawSerial)
add_dependencies(raw_serial_test RawSerial BusMotor)

target_link_libraries (BusMotor RawSerial)
target_link_libraries (raw_serial_test RawSerial BusMotor
)


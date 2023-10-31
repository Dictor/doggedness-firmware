cmake_minimum_required(VERSION 3.20.0)
set(BOARD nucleo_f446re)

find_package(Zephyr)
project(doggedness)

file(GLOB DYNAMIXEL src/dynamixel_sdk/*.cpp)
file(GLOB SRC src/*.cpp)
target_sources(app PRIVATE src/main.c  ${DYNAMIXEL} ${SRC})
set(gen_dir ${ZEPHYR_BINARY_DIR}/include/generated/)
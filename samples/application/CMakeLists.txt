# SPDX-License-Identifier: Apache-2.0

# Add the driver directory as a zephyr module by hand
list(APPEND ZEPHYR_EXTRA_MODULES
	${CMAKE_CURRENT_SOURCE_DIR}/../drivers/sensors/ad5423
)

list(APPEND OVERLAY_CONFIG
	${CMAKE_CURRENT_SOURCE_DIR}/../drivers/sensors/ad5423/ad5423.conf
)

# Re-direct the directory where the 'boards' directory is found from
# $ZEPHYR_BASE to this directory.
#set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
# This sample is only compatible with our board because it has
# re-directed the 'boards' directory to a directory that only contains
# this board.
set(BOARD letitbeat_f446ze)


cmake_minimum_required(VERSION 3.20.0)

# required for apps
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(application_letitbeat)

zephyr_include_directories(src)


# Note that here, we're adding CMAKE_SOURCE_DIR to the include path for nanopb.
# This is needed because the below call to nanopb_generate_cpp() is using
# 'RELPATH .'
set(NANOPB_OPTIONS "-I${CMAKE_SOURCE_DIR}")
nanopb_generate_cpp(proto_sources proto_headers RELPATH .
    src/simple.proto
)
# we need to be able to include generated header files
zephyr_library_include_directories(${CMAKE_CURRENT_BINARY_DIR})

#FILE(GLOB app_sources src/*.c)
#target_sources(app PRIVATE ${proto_sources} ${app_sources})
target_sources(app PRIVATE
	src/main.c
	src/led.c
	src/com.c
	#src/sens.c
	src/shell.c
)
# SPDX-License-Identifier: Apache-2.0

# Add the driver directory as a zephyr module by hand
list(APPEND ZEPHYR_EXTRA_MODULES
	${CMAKE_CURRENT_SOURCE_DIR}/../drivers/tmc5160
)

list(APPEND OVERLAY_CONFIG
	${CMAKE_CURRENT_SOURCE_DIR}/../drivers/tmc5160/tmc5160.conf
)

cmake_minimum_required(VERSION 3.20.0)

# required for apps
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(application_tmc)

zephyr_include_directories(src)

target_sources(app PRIVATE
	src/main.c
)

# Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
#
# SPDX-License-Identifier: Apache-2.0

list(APPEND ZEPHYR_EXTRA_MODULES
	${CMAKE_CURRENT_SOURCE_DIR}/../../drivers
)

cmake_minimum_required(VERSION 3.20.0)

# required for apps
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(tmc-sd)

zephyr_include_directories(src)

target_sources(app PRIVATE
	src/main.c
)

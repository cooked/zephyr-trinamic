# Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
#
# SPDX-License-Identifier: Apache-2.0

# Add *.h to global include paths
zephyr_include_directories(src)

zephyr_library()

zephyr_library_sources(src/tmc2209.c)
zephyr_library_sources(src/tmc2209_map.c)
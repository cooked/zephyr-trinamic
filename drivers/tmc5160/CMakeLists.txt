# Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
#
# SPDX-License-Identifier: Apache-2.0

# Add *.h to global include paths
zephyr_include_directories(src)

zephyr_library()

zephyr_library_sources(src/tmc5160.c)
zephyr_library_sources_ifdef(CONFIG_SHELL src/tmc5160_shell.c)

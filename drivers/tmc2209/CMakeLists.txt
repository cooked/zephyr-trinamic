# Copyright (c) 2022, Stefano Cottafavi <stefano.cottafavi@gmail.com>
#
# SPDX-License-Identifier: Apache-2.0

# Add *.h to global include paths
zephyr_include_directories(src)
zephyr_include_directories(../tmc)

zephyr_library()

zephyr_library_sources(../tmc/tmc.c)
zephyr_library_sources_ifdef(CONFIG_TMC_SPI 	../tmc/tmc_spi.c)
zephyr_library_sources_ifdef(CONFIG_TMC_UART 	../tmc/tmc_uart.c)

zephyr_library_sources(src/tmc2209.c)
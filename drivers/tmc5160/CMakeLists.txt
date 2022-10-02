# SPDX-License-Identifier: Apache-2.0

if(CONFIG_TMC5160)
	# Add *.h to global include paths
	zephyr_include_directories(src)

	zephyr_library()
	zephyr_library_sources(src/tmc5160.c)
	zephyr_library_sources(src/tmc5160_cmd.c)
	zephyr_library_sources_ifdef(CONFIG_TMC5160_SPI 	src/tmc5160_spi.c)
	zephyr_library_sources_ifdef(CONFIG_TMC5160_UART 	src/tmc5160_uart.c)

endif()

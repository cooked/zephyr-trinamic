/*
 * Copyright (c) 2022 Anything Connected
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main_firmware, LOG_LEVEL_DBG);

#include <stdio.h>
#include <stdlib.h>

#include <zephyr/kernel.h>

// local headers
#include "common.h"
#include "led.h"
#include "com.h"
//#include "sens.h"

enum conn_state connected_global = STATE_NOT_CONNECTED;

#define STACKSIZE 		1024
#define STACKSIZE_HI 	4096

#define SAMPLE_T 100

#define SLEEP_TIME_MS   1000

// threads
K_THREAD_STACK_DEFINE(led_thread_stack_area, 	STACKSIZE);
K_THREAD_STACK_DEFINE(sens_thread_stack_area, 	STACKSIZE);
K_THREAD_STACK_DEFINE(com_thread_stack_area, 	STACKSIZE_HI);

static struct k_thread led_thread_data;
static struct k_thread sens_thread_data;
static struct k_thread com_thread_data;

#define THREAD_PRIORITY_HI 	5
#define THREAD_PRIORITY 	7
#define THREAD_PRIORITY_LOW 9

K_MUTEX_DEFINE(mtx);
struct k_fifo mdata_fifo;

void main(void)
{

	k_fifo_init(&mdata_fifo);

	// LEDs
	k_thread_create(&led_thread_data, led_thread_stack_area, K_THREAD_STACK_SIZEOF(led_thread_stack_area),
		(k_thread_entry_t) thread_led,
		&connected_global, NULL, NULL,
		THREAD_PRIORITY_LOW, 0, K_FOREVER);
	k_thread_name_set(&led_thread_data, "LED thread");
	k_thread_start(&led_thread_data);

	// sensors
	/*k_thread_create(&sens_thread_data, sens_thread_stack_area, K_THREAD_STACK_SIZEOF(sens_thread_stack_area),
		(k_thread_entry_t) thread_sens,
		NULL, &connected_global, &mtx,
		THREAD_PRIORITY, 0, K_FOREVER);
	k_thread_name_set(&sens_thread_data, "SEN thread");
	k_thread_start(&sens_thread_data);*/

	// com
	k_thread_create(&com_thread_data, com_thread_stack_area, K_THREAD_STACK_SIZEOF(com_thread_stack_area),
		(k_thread_entry_t) thread_com,
		NULL, &connected_global, &mtx,
		THREAD_PRIORITY_HI, 0, K_FOREVER);
	k_thread_name_set(&com_thread_data, "COM thread");
	k_thread_start(&com_thread_data);

}

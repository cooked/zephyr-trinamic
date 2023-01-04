
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(main_firmware, LOG_LEVEL_DBG);

#include <stdio.h>		// sprintf()
#include <string.h>		// strlen()
#include <time.h>		// gmtime()

#include <zephyr/sys/reboot.h>
//#include <date_time.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

#include "common.h"
#include "com.h"

// USB
// https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/zephyr/samples/subsys/usb/console/README.html

// protobuf
//#include <pb_encode.h>
//#include <pb_decode.h>
//#include "src/simple.pb.h"

extern struct k_fifo mdata_fifo;

//BUILD_ASSERT(DT_NODE_HAS_COMPAT(DT_CHOSEN(zephyr_console), zephyr_cdc_acm_uart),
//	     "Console device is not ACM CDC UART device");

void thread_com(struct k_lifo *data, enum conn_state *conn_state, struct k_mutex *mtx) {

	// TODO
	*conn_state = STATE_NOT_CONNECTED;

	// store date and time
	int64_t 	datetime_ms;
	time_t 		datetime_s;
	struct tm 	*datetime_tm;
	char		datetime_str[DATE_TIME_LEN];


	const struct device *const dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	uint32_t dtr = 0;

	/*if (usb_enable(NULL)) {
		return;
	}*/

	/* Poll if the DTR flag was set */
	/*while (!dtr) {
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
		// Give CPU resources to low priority threads.
		k_sleep(K_MSEC(100));
	}*/


	while(1) {

		printk("Hello World! %s\n", CONFIG_ARCH);
		k_sleep(K_SECONDS(1));

		// if LIFO empty wait here
		//struct mdata_t *md = k_fifo_get(&mdata_fifo,  K_FOREVER);


		// TODO: move this to the sens thread
		//date_time_now(&datetime_ms);
		//datetime_s 	= datetime_ms/1000;
		//datetime_tm = gmtime(&datetime_s);

		//strftime(datetime_str,DATE_TIME_LEN, "%Y-%m-%d %H:%M:%S.000000", datetime_tm);
		/*char payload[1024]= {"\0"};
		sprintf(payload,"{\"time_measured\":\"%s\"," \
						"\"data\": {" \
						"\"ax\":\"%.3f\",\"ay\":\"%.3f\",\"az\":\"%.3f\"," \
						"\"gx\":\"%.3f\",\"gy\":\"%.3f\",\"gz\":\"%.3f\"," \
						"\"mx\":\"%.3f\",\"my\":\"%.3f\",\"mz\":\"%.3f\"," \
						"\"temp\":\"%.3f\"}}",
						datetime_str,
						sensor_value_to_double(&md->ax),
						sensor_value_to_double(&md->ay),
						sensor_value_to_double(&md->az),
						sensor_value_to_double(&md->gx),
						sensor_value_to_double(&md->gy),
						sensor_value_to_double(&md->gz),
						sensor_value_to_double(&md->mx),
						sensor_value_to_double(&md->my),
						sensor_value_to_double(&md->mz),
						sensor_value_to_double(&md->temp)
		);*/

		// TODO: TX

	}


}

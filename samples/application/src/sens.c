
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(main_firmware, LOG_LEVEL_DBG);

#include <zephyr/drivers/sensor.h>

#include "ad5423.h"

#include "common.h"
#include "sens.h"

extern struct k_fifo mdata_fifo;

const struct device *dac;

void thread_sens(struct k_lifo *data, enum conn_state *conn_state, struct k_mutex *mtx)
{

	dac = DEVICE_DT_GET_ANY(adi_ad5423);

	//
	//const struct device *ad5423  = device_get_binding("AD5423");

	if (!dac) {
		LOG_ERR("Failed to get device binding for AD5423");
		return;
	}

	struct sensor_value accel[3], gyro[3], mag[4];

	while(1) {

		get_chip_id(dac);
		//printk("SENS \n");

		/*if(*conn_state == STATE_BACKEND_CONNECTED) {

			// get 6DOF
			//sensor_sample_fetch(iim42652);
			//sensor_channel_get(iim42652, SENSOR_CHAN_ACCEL_XYZ, accel);
			//sensor_channel_get(iim42652, SENSOR_CHAN_GYRO_XYZ, gyro);

			// get MAG
			//sensor_sample_fetch(mlx90393);
			//sensor_channel_get(mlx90393, SENSOR_CHAN_ALL, mag);

			struct mdata_t md = {
				.ax = accel[0],
				.ay = accel[1],
				.az = accel[2],
				.gx = gyro[0],
				.gy = gyro[1],
				.gz = gyro[2],
				.mx = mag[0],
				.my = mag[1],
				.mz = mag[2],
				.temp = mag[3]
			};

			k_fifo_put(&mdata_fifo, &md);

		}*/

		// slow down
		k_msleep( 1000 );

	}

}
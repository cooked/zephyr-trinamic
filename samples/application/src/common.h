
#include <zephyr/drivers/sensor.h>

enum conn_state {
	STATE_NOT_CONNECTED,
	STATE_LTE_CONNECTED,
	STATE_BACKEND_CONNECTED,
	STATE_ERROR
};

// data structure
struct mdata_t {
	void  *reserved; /* 1st word reserved for use by fifo */
	struct sensor_value ax;
	struct sensor_value ay;
	struct sensor_value az;
	struct sensor_value gx;
	struct sensor_value gy;
	struct sensor_value gz;
	struct sensor_value mx;
	struct sensor_value my;
	struct sensor_value mz;
	struct sensor_value temp;
};
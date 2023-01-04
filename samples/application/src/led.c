
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(main_firmware, LOG_LEVEL_DBG);

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>

#include <zephyr/drivers/gpio.h>

#include "common.h"
#include "led.h"


static const struct gpio_dt_spec led1_r = GPIO_DT_SPEC_GET(LED1_R_NODE, gpios);
static const struct gpio_dt_spec led1_g = GPIO_DT_SPEC_GET(LED1_G_NODE, gpios);
static const struct gpio_dt_spec led1_b = GPIO_DT_SPEC_GET(LED1_B_NODE, gpios);

void timer_expired() {
	//gpio_pin_set_dt(&led1_g, 0);
	gpio_pin_set_dt(&led1_g, 0);
}

K_TIMER_DEFINE(timer, timer_expired, NULL);

int led_off() {
	int ret = 0;
	ret |= gpio_pin_set_dt(&led1_r, 0);
	ret |= gpio_pin_set_dt(&led1_g, 0);
	ret |= gpio_pin_set_dt(&led1_b, 0);
	return ret;
}

void thread_led(enum conn_state *conn_state, void *arg2, void *arg3)
{
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	printk("LED control starting...\n");

	if(	gpio_pin_configure_dt(&led1_r, GPIO_OUTPUT_ACTIVE) < 0 ||
		gpio_pin_configure_dt(&led1_g, GPIO_OUTPUT_ACTIVE) < 0 ||
		gpio_pin_configure_dt(&led1_b, GPIO_OUTPUT_ACTIVE) < 0 ) {
		return;
	}

	led_off();

	LOG_DBG("Succesfully initialized LED");

	int ret;
	uint8_t prev_state = STATE_NOT_CONNECTED;

	while(1) {

		switch(*conn_state) {
			case STATE_NOT_CONNECTED:
				prev_state = STATE_NOT_CONNECTED;

				ret = gpio_pin_toggle_dt(&led1_b);
				ret = gpio_pin_toggle_dt(&led1_r);
				ret = gpio_pin_toggle_dt(&led1_g);
				k_sleep(K_MSEC(500));
				break;

			case STATE_LTE_CONNECTED:
				prev_state = STATE_LTE_CONNECTED;

				ret = gpio_pin_set_dt(&led1_b, 1);

				k_sleep(K_MSEC(500));
				break;

			case STATE_BACKEND_CONNECTED:
				if( prev_state != STATE_BACKEND_CONNECTED) {

					ret = gpio_pin_set_dt(&led1_b, 0);

					// turn green ON (turned OFF by timer callback)
					k_timer_start(&timer, K_MSEC(5000), K_NO_WAIT);
					//ret = gpio_pin_set_dt(&led1_g, 1);
					ret |= gpio_pin_set_dt(&led1_g, 1);

				}

				prev_state = STATE_BACKEND_CONNECTED;
				break;

			case STATE_ERROR:
				prev_state = STATE_ERROR;

				//ret = gpio_pin_set_dt(&led1_b, 0);
				//ret |= gpio_pin_set_dt(&led2_b, 0);
				//ret |= gpio_pin_toggle_dt(&led1_r);
				ret = gpio_pin_set_dt(&led1_b, 0);
				ret |= gpio_pin_toggle_dt(&led1_r);

				k_sleep(K_MSEC(1000));
				break;
		}

	}

}

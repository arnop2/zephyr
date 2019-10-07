/*
 * Copyright (c) 2019, STMICROLECTRONICS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <zephyr.h>
#include <device.h>
#include <string.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <drivers/sensor.h>
#include <drivers/i2c.h>


/* Display / Framebuffer */
#include <display/cfb.h>
#if defined(CONFIG_SSD1306)
#define DISPLAY_DRIVER		"SSD1306"
#endif

#include <logging/log.h>
LOG_MODULE_REGISTER(openamp_uart, LOG_LEVEL_DBG);

#define I2C_DEV "I2C_5"

#define APP_TASK_STACK_SIZE (4096)
K_THREAD_STACK_DEFINE(thread_stack, APP_TASK_STACK_SIZE);
static struct k_thread thread_data;


static struct device *dev_display;
static struct device *dev_tof;

static u16_t display_rows;
static u8_t display_ppt;
static u8_t display_font_width;
static u8_t display_font_height;
static bool display_available;

#define TEST_DATA_SIZE	20

void stop_display(void)
{
	display_blanking_on(dev_display);
}

void init_sensors(void)
{
	dev_tof = device_get_binding(DT_INST_0_ST_VL53L0X_LABEL);
	if (dev_tof == NULL) {
		printk("Could not get VL53L0X device\n");
}

}
void init_display(void)
{
	/* init display */
	dev_display = device_get_binding(DISPLAY_DRIVER);
	if (!dev_display) {
		LOG_ERR("Display Device not found\n");
	} else {
		display_available = true;

		if (display_set_pixel_format(dev_display,
					     PIXEL_FORMAT_MONO10) != 0) {
			LOG_ERR("Failed to set format PIXEL_FORMAT_MONO10\n");
			return;
		}
		if (cfb_framebuffer_init(dev_display)) {
			LOG_ERR("Framebuffer initialization failed!\n");
			return;
		}
		cfb_framebuffer_clear(dev_display, true);

		display_blanking_off(dev_display);

		display_rows = cfb_get_display_parameter(dev_display,
							 CFB_DISPLAY_ROWS);
		display_ppt = cfb_get_display_parameter(dev_display,
							CFB_DISPLAY_PPT);
		for (int idx = 0; idx < 42; idx++) {
			if (cfb_get_font_size(dev_display, idx,
					      &display_font_width,
					      &display_font_height)) {
				break;
			}
			cfb_framebuffer_set_font(dev_display, idx);
			LOG_DBG("idx: %d font width %d, font height %d\n",
				idx, display_font_width, display_font_height);
		}
		/* for idx: 0 font width 10, font height 16*/
		cfb_framebuffer_set_font(dev_display, 0);
		cfb_get_font_size(dev_display, 0, &display_font_width,
				  &display_font_height);

		LOG_DBG("x_res %d, y_res %d, ppt %d, rows %d, cols %d\n",
			cfb_get_display_parameter(dev_display, CFB_DISPLAY_WIDTH),
			cfb_get_display_parameter(dev_display, CFB_DISPLAY_HEIGH),
			display_ppt,
			display_rows,
			cfb_get_display_parameter(dev_display, CFB_DISPLAY_COLS));
	}
}


void app_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	char display_buf[16];
	u8_t line = 0;
	struct sensor_value value;
	int ret;

	LOG_INF("\r\ndemo started\n");

	init_display();
	init_sensors();

	if (display_available) {
		line = 0;
		printk("--init display screen --\n");

		cfb_framebuffer_clear(dev_display, false);
		cfb_print(dev_display, "Zephyr", 0,
			  line++ * display_font_height);
		cfb_print(dev_display, "   Waiting ", 0,
			  line++ * display_font_height);
		cfb_print(dev_display, "     TTY   ", 0,
			  line++ * display_font_height);
		cfb_print(dev_display, "    data   ", 0,
			  line++ * display_font_height);
		cfb_framebuffer_finalize(dev_display);
	}

	while (1) {
		line = 0;
		
		if (display_available)
			cfb_framebuffer_clear(dev_display, false);

		if (display_available)
			cfb_print(dev_display, "   Zephyr", 0,
				  line++ * display_font_height);
		ret = sensor_sample_fetch(dev_tof);
		if (ret) {
			printk("sensor_sample_fetch failed ret %d\n", ret);
			return;
		}

		/* proxy */
		ret = sensor_channel_get(dev_tof, SENSOR_CHAN_PROX, &value);
		printk("prox is %d\n", value.val1);
		snprintf(display_buf, sizeof(display_buf), "proxy: %d",
			 value.val1);
		if (display_available)
			cfb_print(dev_display, display_buf, 0,
				  line++ * display_font_height);

		/* distance */
		ret = sensor_channel_get(dev_tof,
					 SENSOR_CHAN_DISTANCE,
					 &value);
		printf("distance is %.3fm\n", sensor_value_to_double(&value));
		snprintf(display_buf, sizeof(display_buf), "dist: %.3fm",
			sensor_value_to_double(&value));
		if (display_available)
			cfb_print(dev_display, display_buf, 0,
				  line++ * display_font_height);

		if (display_available)
			cfb_framebuffer_finalize(dev_display);
		k_sleep(1000);
	}


	printk("app task ended\n");
}


void main(void)
{

	k_thread_create(&thread_data, thread_stack, APP_TASK_STACK_SIZE,
			(k_thread_entry_t)app_task,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, 0);

}

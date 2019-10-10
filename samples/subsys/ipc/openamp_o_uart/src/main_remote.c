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
#include <stdio.h>
#include <drivers/sensor.h>
#include <drivers/i2c.h>
#include <drivers/uart.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(openamp_uart, LOG_LEVEL_DBG);

/* Display / Framebuffer */
#include <display/cfb.h>
#if defined(CONFIG_SSD1306)
#define DISPLAY_DRIVER		"SSD1306"
#endif

#define RPMSG_NAME_SIZE	(32)
#define	RPMSG_MAGIC_NUMBER	(0xbe57)

#define RPMSG_MAX_BUF_SIZE	(512)
#define RPMSG_ADDR_ANY		(0xFFFFFFFF)
#define RPMSG_NS_ADDR		(0x35)
#define RPMSG_SERVICE_NAME	"rpmsg-button"

#define STOP_REQ	1
#define RESTART_REQ	2

#define APP_TASK_STACK_SIZE (4096)
K_THREAD_STACK_DEFINE(thread_stack, APP_TASK_STACK_SIZE);
static struct k_thread thread_data;

/* RPMsg structures */
struct rpmsg_hdr {
	uint32_t src;
	uint32_t dst;
	uint32_t reserved;
	uint16_t len;
	uint16_t flags;
	u8_t data[0];
} __attribute__((__packed__));

typedef int (*rpmsg_rx_cb_t)(void *data, uint32_t len);

struct rpmsg_endpoint {
	char name[RPMSG_NAME_SIZE];
	rpmsg_rx_cb_t cb;
	uint32_t src;
	uint32_t dst;
};

struct rpmsg_ns_msg {
	char name[RPMSG_NAME_SIZE];
	uint32_t addr;
	uint32_t flags;
} __attribute__((__packed__));

enum rpmsg_ns_flags {
	RPMSG_NS_CREATE		= 0,
	RPMSG_NS_DESTROY	= 1,
};

static int rpmsg_proxy_cb(void *data, uint32_t len);
static int rpmsg_dist_cb(void *data, uint32_t len);
static int rpmsg_shutdown_cb(void *data, uint32_t len);

static struct rpmsg_endpoint proxy_ept = {
	"rpmsg-button",
	rpmsg_proxy_cb,
	0,
	-1
};

static struct rpmsg_endpoint dist_ept = {
	"rpmsg-vl5310x",
	rpmsg_dist_cb,
	1,
	-1
};

static struct rpmsg_endpoint shutdown_ept = {
	"rpmsg-tty-channel",
	rpmsg_shutdown_cb,
	2,
	-1
};

static struct rpmsg_endpoint *ept_tab[] = {
	&proxy_ept, &dist_ept, &shutdown_ept, NULL};
static u8_t recv_rpmsg[RPMSG_MAX_BUF_SIZE];
static u8_t recvData[RPMSG_MAX_BUF_SIZE];
static int rx_data_idx;
bool magic_number; /* use to synchronize RPMSG message start */

/* devices */
static struct device *dev_display;
static struct device *dev_tof;
static struct device *dev_uart;

/* display */
static u16_t display_rows;
static u8_t display_ppt;
static u8_t display_font_width;
static u8_t display_font_height;
static bool display_available;

static K_SEM_DEFINE(data_sem, 0, 1);
static u8_t proxy_val;
static u8_t stop_mode;

static inline int uart_rpmsg_send(struct device *dev_uart, uint32_t src,
				  uint32_t dst, const void *data, int len)
{
	int msg_size = len + sizeof(struct rpmsg_hdr);
	u16_t mag_num = RPMSG_MAGIC_NUMBER;
	u8_t msg_buff[RPMSG_MAX_BUF_SIZE];
	struct rpmsg_hdr *msg_container = (struct rpmsg_hdr *)msg_buff;
	u8_t *p_msg_buff;
	int i;

	msg_container->src = src;
	msg_container->dst = dst;
	msg_container->flags = 0;
	msg_container->len = len;
	msg_container->reserved = 0;

	memcpy(msg_container->data, data, len);

	p_msg_buff = (u8_t *)&mag_num;
	for (i = 0; i < 2; i++) {
		uart_poll_out(dev_uart, p_msg_buff[i]);
	}

	for (i = 0; i < msg_size; i++) {
		uart_poll_out(dev_uart, msg_buff[i]);
	}

	return 0;
}

static int rpmsg_proxy_cb(void *data, uint32_t len)
{
	LOG_ERR("%s:\n", __func__);

	return 0;
}

static int rpmsg_shutdown_cb(void *data, uint32_t len)
{
	LOG_ERR("%s: %d\n", __func__, len);

	if (!strncmp(data, "stop", 4)) {
		LOG_DBG("%s: shutdown requested\n", __func__);
		stop_mode = STOP_REQ;
	} else if (!strncmp(data, "restart", 7)) {
		LOG_DBG("%s: restart requested\n", __func__);
		stop_mode = RESTART_REQ;
	}
	return 0;
}

static int rpmsg_dist_cb(void *data, uint32_t len)
{

	int ret;
	struct sensor_value value;
	unsigned int val_mm;

	LOG_DBG("%s:\n", __func__);
	ret = sensor_sample_fetch(dev_tof);
	if (ret) {
		printk("sensor_sample_fetch failed ret %d\n", ret);
		return ret;
	}

	ret = sensor_channel_get(dev_tof, SENSOR_CHAN_DISTANCE, &value);
	if (ret) {
		LOG_ERR("%s: %d\n", __func__, __LINE__);
		return ret;
	}

	val_mm = (value.val1 * 1000) + (value.val2 / 1000);
	LOG_DBG("%s: send dist %d\n", __func__, val_mm);
	uart_rpmsg_send(dev_uart, dist_ept.src, dist_ept.dst, &val_mm,
			sizeof(val_mm));

	return 0;
}

static int uart_rpmsg_send_ns_message(struct device *dev_uart,
				      char *name, uint32_t addr,
				      unsigned long flags)

{
	struct rpmsg_ns_msg ns_msg;

	ns_msg.flags = flags;
	ns_msg.addr = addr;
	strncpy(ns_msg.name, name, sizeof(ns_msg.name));
	return uart_rpmsg_send(dev_uart, addr, RPMSG_NS_ADDR,
			       &ns_msg, sizeof(ns_msg));
}

static int uart_rpmsg_create_ept(struct device *dev_uart,
				 struct rpmsg_endpoint *ept)
{
	ept->dst  = -1;
	return uart_rpmsg_send_ns_message(dev_uart, ept->name, ept->src,
					  RPMSG_NS_CREATE);
}

static int uart_rpmsg_destroy_ept(struct device *dev_uart,
				  struct rpmsg_endpoint *ept)
{
	return uart_rpmsg_send_ns_message(dev_uart, ept->name, ept->src,
					  RPMSG_NS_DESTROY);
}

static int receive_message(void)
{
	int status;
	struct rpmsg_hdr *p_msg = (struct rpmsg_hdr *)recv_rpmsg;
	struct rpmsg_endpoint *ept = ept_tab[0];
	unsigned int i = 0;

	status = k_sem_take(&data_sem, K_MSEC(200));
	if (status == 0) {
		while (ept != NULL) {
			if (p_msg->dst == ept->src) {
				if (ept->dst == -1)
					ept->dst = p_msg->src;
				return ept->cb(p_msg->data, p_msg->len);
			}
			ept = ept_tab[i];
			i++;
		}
	}

	return status;
}

void stop_display(void)
{
	display_blanking_on(dev_display);
}

static void init_sensors(void)
{
	dev_tof = device_get_binding(DT_INST_0_ST_VL53L0X_LABEL);
	if (dev_tof == NULL) {
		printk("Could not get VL53L0X device\n");
	}
}

static void init_display(void)
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

static void uart_callback(struct device *dev)
{
	uint32_t recv_sync;
	int num;
	static unsigned int msg_size;
	struct rpmsg_hdr *p_msg = (struct rpmsg_hdr *)recvData;

	/* Verify uart_irq_update() */
	if (!uart_irq_update(dev)) {
		LOG_ERR("retval should always be 1\n");
		return;
	}

	/* Verify uart_irq_rx_ready() */
	if (uart_irq_rx_ready(dev)) {
		/* Verify uart_fifo_read() */
		if (!magic_number) {
			num = uart_fifo_read(dev,
					     (u8_t *)&recvData[rx_data_idx], 1);
			rx_data_idx += num;
			if (rx_data_idx == 2) {
				recv_sync = recvData[0] + (recvData[1] << 8);
				if (recvData[0] == 0x57  &&
				    recvData[1] == 0xbe) {
					magic_number = true;
					msg_size = 0;
					rx_data_idx = 0;
				} else {
					recvData[0] = recvData[1];
					rx_data_idx--;
				}
			}
		} else {
			num = uart_fifo_read(dev,
					     (u8_t *)&recvData[rx_data_idx], 1);
			rx_data_idx += num;
			if (rx_data_idx == sizeof(struct rpmsg_hdr)) {
				/* header can be parsed */
				msg_size = p_msg->len;
			} else if (msg_size != 0 &&
				   (rx_data_idx ==
					sizeof(struct rpmsg_hdr) + msg_size)) {
				memcpy(recv_rpmsg, recvData,
				       sizeof(struct rpmsg_hdr) + msg_size);
				magic_number = false;
				rx_data_idx = 0;
				LOG_DBG("%s: msg received\n", __func__);
				k_sem_give(&data_sem);
			}
		}

	}
}

static void app_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	char display_buf[16];
	u8_t line = 0;
	struct sensor_value value;
	int ret;

	LOG_INF("\r\nrpmsg over uart demo started\n");

	init_sensors();

	dev_uart = device_get_binding(DT_UART_STM32_UART_4_NAME);
	if (!dev_uart) {
		LOG_ERR("dev_uart Device not found\n");
		return;
	}
	magic_number = 0;

	uart_irq_callback_set(dev_uart, uart_callback);

	uart_irq_rx_enable(dev_uart);

start_task:
	init_display();

	if (display_available) {
		line = 0;
		cfb_framebuffer_clear(dev_display, false);
		cfb_print(dev_display, "Zephyr", 0,
			  line++ * display_font_height);
		cfb_print(dev_display, "   Waiting ", 0,
			  line++ * display_font_height);
		cfb_print(dev_display, "    ToF    ", 0,
			  line++ * display_font_height);
		cfb_print(dev_display, "    data   ", 0,
			  line++ * display_font_height);
		cfb_framebuffer_finalize(dev_display);
	}

	/*ept creation*/
	if (uart_rpmsg_create_ept(dev_uart, &proxy_ept) != 0) {
		LOG_ERR("rpmsg ns service failed\n");
		return;
	}

	receive_message();

	/*ept creation*/
	if (uart_rpmsg_create_ept(dev_uart, &dist_ept) != 0) {
		LOG_ERR("rpmsg ns service failed\n");
		return;
	}

	k_sleep(200);
	/*ept creation*/
	if (uart_rpmsg_create_ept(dev_uart, &shutdown_ept) != 0) {
		LOG_ERR("rpmsg ns service failed\n");
		return;
	}

	while (1) {
		unsigned int line = 0;

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
		snprintf(display_buf, sizeof(display_buf), "proximity: %d",
			 value.val1);
		if (display_available)
			cfb_print(dev_display, display_buf, 0,
				  line++ * display_font_height);

		if (value.val1 != proxy_val) {
			proxy_val = value.val1;
			uart_rpmsg_send(dev_uart, proxy_ept.src,
					proxy_ept.dst, &proxy_val,
					sizeof(proxy_val));
		}

		/* distance */
		ret = sensor_channel_get(dev_tof,
					 SENSOR_CHAN_DISTANCE,
					 &value);
		snprintf(display_buf, sizeof(display_buf), "dist: %.3fm",
			 sensor_value_to_double(&value));
		if (display_available)
			cfb_print(dev_display, display_buf, 0,
				  line++ * display_font_height);
		if (display_available)
			cfb_framebuffer_finalize(dev_display);

		receive_message();
		if (stop_mode)
			goto end_task;
	}

end_task:
	k_sleep(200);

	stop_display();

	if (uart_rpmsg_destroy_ept(dev_uart, &proxy_ept) != 0) {
		LOG_ERR("rpmsg ns service destroy failed\n");
		return;
	}

	k_sleep(100);
	if (uart_rpmsg_destroy_ept(dev_uart, &dist_ept) != 0) {
		LOG_ERR("rpmsg ns service destroy failed\n");
		return;
	}
	k_sleep(100);

	if (uart_rpmsg_destroy_ept(dev_uart, &shutdown_ept) != 0) {
		LOG_ERR("rpmsg ns service destroy failed\n");
		return;
	}

	k_sleep(100);
	if (stop_mode == RESTART_REQ) {
		stop_mode = 0;
		goto start_task;
	}

	printk("sensor task ended\n");
}

void main(void)
{
	k_thread_create(&thread_data, thread_stack, APP_TASK_STACK_SIZE,
			(k_thread_entry_t)app_task,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, 0);

}

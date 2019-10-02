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

#include <ipm.h>

#include <drivers/i2c.h>

#include <openamp/open_amp.h>
#include <metal/device.h>
#include <resource_table.h>

/* Display / Framebuffer */
#include <display/cfb.h>
#if defined(CONFIG_SSD1306)
#define DISPLAY_DRIVER		"SSD1306"
#endif

#include <logging/log.h>
LOG_MODULE_REGISTER(openamp_rsc_table, LOG_LEVEL_DBG);

#define SHM_DEVICE_NAME	"shm"
#define I2C_RPMSG_DEV "RPMSG_I2C"
#define I2C_DEV "I2C_5"
#define I2C_REMOTE_DISP_ADDR 0x3D

/* constant derivated from linker symbols */
#define SHM_START_ADDR	DT_IPC_SHM_BASE_ADDRESS
#define SHM_SIZE	(DT_IPC_SHM_SIZE * 1024)

#define APP_TASK_STACK_SIZE (4096)
K_THREAD_STACK_DEFINE(thread_stack, APP_TASK_STACK_SIZE);
static struct k_thread thread_data;

static struct device *ipm_handle;
struct rpmsg_device *rpdev;

static struct device *dev_display;
static u16_t display_rows;
static u8_t display_ppt;
static u8_t display_font_width;
static u8_t display_font_height;
static bool display_available;
static struct device *i2c_dev;
static bool first_write;
static u8_t reg_addr;

static struct sensor_value temp1, temp2, hum, press;
static struct device *hts221;
static struct device *lps22hb;

static bool hts221_available = true;
static bool lps22hb_available = true;

static metal_phys_addr_t shm_physmap = SHM_START_ADDR;

struct metal_device shm_device = {
	.name = SHM_DEVICE_NAME,
	.num_regions = 2,
	.regions = {
		{.virt = NULL}, /* shared memory */
		{.virt = NULL}, /* rsc_table memory */
	},
	.node = { NULL },
	.irq_num = 0,
	.irq_info = NULL
};

static struct metal_io_region *shm_io;
static struct rpmsg_virtio_shm_pool shpool;

static struct metal_io_region *rsc_io;
static struct rpmsg_virtio_device rvdev;

static void *rsc_table;

static K_SEM_DEFINE(data_sem, 0, 1);

#define TEST_DATA_SIZE	20

void stop_display(void)
{
	display_blanking_on(dev_display);
}

static void platform_ipm_callback(void *context, u32_t id, volatile void *data)
{
	if (id == 2) {
		/* receive shutdown */
		stop_display();
		return;
	}

	LOG_ERR("IPM IRQ\n");
	k_sem_give(&data_sem);
}

static void receive_message(unsigned char **msg, unsigned int *len)
{
	unsigned int i = 1000;

	while (i) {
		if (k_sem_take(&data_sem, K_NO_WAIT) == 0) {
			rproc_virtio_notified(rvdev.vdev, VRING1_ID);
		} else {
			k_sleep(1);
			i--;
		}
	}
}

static void new_service_cb(struct rpmsg_device *rdev, const char *name,
			   uint32_t src)
{
	LOG_ERR("%s: unexpected ns service receive for name %s\n",
		__func__, name);
}

int mailbox_notify(void *priv, uint32_t id)
{
	ARG_UNUSED(priv);

	ipm_send(ipm_handle, 0, id, NULL, 0);
	LOG_DBG("%s: msg sent\n", __func__);

	return 0;
}

void init_sensors(void)
{
	/* search in devicetree if sensors are referenced */

#if CONFIG_HTS221
	hts221 = device_get_binding(DT_INST_0_ST_HTS221_LABEL);
	if (!hts221) {
		LOG_ERR("Could not get HTS221 device\n");
		hts221_available = false;
	}
#endif
#if CONFIG_LPS22HB
	lps22hb = device_get_binding(DT_INST_0_ST_LPS22HB_PRESS_LABEL);
	if (!lps22hb) {
		LOG_ERR("Could not get LPS22HB device\n");
		lps22hb_available = false;
	}
#endif

	/* set LSM6DSL accel/gyro sampling frequency to 104 Hz */
	struct sensor_value odr_attr;

	odr_attr.val1 = 104;
	odr_attr.val2 = 0;

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

void init_remote_display(void)
{

	i2c_dev = device_get_binding(I2C_DEV);
}

int platform_init(void)
{
	void *rsc_tab_addr;
	int rsc_size;
	struct metal_device *device;
	struct metal_init_params metal_params = METAL_INIT_DEFAULTS;
	int status;

	status = metal_init(&metal_params);
	if (status) {
		LOG_ERR("metal_init: failed: %d\n", status);
		return -1;
	}

	status = metal_register_generic_device(&shm_device);
	if (status) {
		LOG_ERR("Couldn't register shared memory: %d\n", status);
		return -1;
	}

	status = metal_device_open("generic", SHM_DEVICE_NAME, &device);
	if (status) {
		LOG_ERR("metal_device_open failed: %d\n", status);
		return -1;
	}

	/* declare shared memory region */
	metal_io_init(&device->regions[0], (void *)SHM_START_ADDR, &shm_physmap,
		      SHM_SIZE, -1, 0, NULL);

	shm_io = metal_device_io_region(device, 0);
	if (!shm_io) {
		LOG_ERR("Failed to get shm_io region\n");
		return -1;
	}

	/* declare resource table region */
	rsc_table_get(&rsc_tab_addr, &rsc_size);
	rsc_table = (struct st_resource_table *)rsc_tab_addr;

	metal_io_init(&device->regions[1], rsc_table,
		      (metal_phys_addr_t *)rsc_table, rsc_size, -1, 0, NULL);

	rsc_io = metal_device_io_region(device, 1);
	if (!rsc_io) {
		LOG_ERR("Failed to get rsc_io region\n");
		return -1;
	}

	/* setup IPM */
	ipm_handle = device_get_binding("MAILBOX_0");
	if (!ipm_handle) {
		LOG_ERR("Failed to find ipm device\n");
		return -1;
	}

	ipm_register_callback(ipm_handle, platform_ipm_callback, NULL);

	status = ipm_set_enabled(ipm_handle, 1);
	if (status) {
		LOG_ERR("ipm_set_enabled failed\n");
		return -1;
	}

	return 0;
}

static void cleanup_system(void)
{
	ipm_set_enabled(ipm_handle, 0);
	rpmsg_deinit_vdev(&rvdev);
	metal_finish();
}

struct  rpmsg_device *
platform_create_rpmsg_vdev(unsigned int vdev_index,
			   unsigned int role,
			   void (*rst_cb)(struct virtio_device *vdev),
			   rpmsg_ns_bind_cb ns_cb)
{
	struct fw_rsc_vdev_vring *vring_rsc;
	struct virtio_device *vdev;
	int ret;

	vdev = rproc_virtio_create_vdev(VIRTIO_DEV_SLAVE, VDEV_ID,
					rsc_table_to_vdev(rsc_table),
					rsc_io, NULL, mailbox_notify, NULL);

	if (!vdev) {
		LOG_ERR("failed to create vdev\r\n");
		return NULL;
	}

	/* wait master rpmsg init completion */
	rproc_virtio_wait_remote_ready(vdev);

	vring_rsc = rsc_table_get_vring0(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 0, vring_rsc->notifyid,
				      (void *)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		LOG_ERR("failed to init vring 0\r\n");
		goto failed;
	}

	vring_rsc = rsc_table_get_vring1(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 1, vring_rsc->notifyid,
				      (void *)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		LOG_ERR("failed to init vring 1\r\n");
		goto failed;
	}

	rpmsg_virtio_init_shm_pool(&shpool, NULL, SHM_SIZE);
	ret =  rpmsg_init_vdev(&rvdev, vdev, ns_cb, shm_io, &shpool);

	if (ret) {
		LOG_ERR("failed rpmsg_init_vdev\r\n");
		goto failed;
	}

	return rpmsg_virtio_get_rpmsg_device(&rvdev);

failed:
	rproc_virtio_remove_vdev(vdev);

	return NULL;
}

static int rpmsg_slave_write_requested(struct i2c_slave_config *config)
{

	LOG_DBG("%s\n", __func__);
	first_write = true;

	return 0;
}

static int rpmsg_slave_read_requested(struct i2c_slave_config *config,
				      u8_t *val)
{
	LOG_DBG("%s: val=0x%x\n", __func__, *val);
	reg_addr = *val;
	/* Increment will be done in the read_processed callback */

	return 0;
}

static int rpmsg_slave_write_received(struct i2c_slave_config *config,
				      u8_t val)
{
	int ret;

	if (first_write) {
		reg_addr = val;
		first_write = false;
	} else {
		LOG_DBG("%s: write reg=0x%x val=0x%x\n", __func__, reg_addr,
			val);
		ret = i2c_reg_write_byte(i2c_dev, I2C_REMOTE_DISP_ADDR,
					 reg_addr, val);
		if (ret < 0)
			LOG_DBG("%s: failed to write reg=0x%x val=0x%x\n",
				__func__, reg_addr, val);
	}

	return 0;
}

static int rpmsg_slave_read_processed(struct i2c_slave_config *config,
				      u8_t *val)
{

	LOG_DBG("%s: reg=0x%x val=0x%x\n", __func__, reg_addr, *val);

	i2c_reg_read_byte(i2c_dev, I2C_REMOTE_DISP_ADDR, reg_addr++, val);

	/* Increment will be done in the next read_processed callback
	 * In case of STOP, the byte won't be taken in account
	 */

	return 0;
}

static int rpmsg_slave_stop(struct i2c_slave_config *config)
{

	LOG_DBG("%s: stop\n", __func__);
	first_write = true;

	return 0;
}

static const struct i2c_slave_callbacks i2c_rpmsg_callbacks = {
	.write_requested = rpmsg_slave_write_requested,
	.read_requested = rpmsg_slave_read_requested,
	.write_received = rpmsg_slave_write_received,
	.read_processed = rpmsg_slave_read_processed,
	.stop = rpmsg_slave_stop,
};

void app_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	unsigned char *msg;
	struct device *i2c_rpmsg_dev;
	struct i2c_slave_config config;
	int len;
	int ret = 0;
	char message[128];
	char display_buf[16];
	u8_t line = 0;

	LOG_INF("\r\nOpenAMP[remote] I2C demo started\n");
	i2c_rpmsg_dev = device_get_binding(I2C_RPMSG_DEV);

	config.address = I2C_REMOTE_DISP_ADDR;
	config.callbacks = &i2c_rpmsg_callbacks;

	/* Attach application Slave */
	ret = i2c_slave_register(i2c_rpmsg_dev, &config);
	if (ret) {
		LOG_ERR("Failed to register I2C SLAVE %#x\n", config.address);
		goto task_end;
	}

	init_remote_display();
	init_sensors();
	init_display();

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
		unsigned int line = 0;

		if (display_available)
			cfb_framebuffer_clear(dev_display, false);

		/* Get sensor samples */
		if (hts221_available && sensor_sample_fetch(hts221) < 0)
			printf("HTS221 Sensor sample update error\n");
		if (lps22hb_available && sensor_sample_fetch(lps22hb) < 0)
			printf("LPS22HB Sensor sample update error\n");

		/* Get sensor data */
		if (hts221_available) {
			sensor_channel_get(hts221, SENSOR_CHAN_AMBIENT_TEMP,
					   &temp1);
			sensor_channel_get(hts221, SENSOR_CHAN_HUMIDITY, &hum);
		}
		if (lps22hb_available) {
			sensor_channel_get(lps22hb, SENSOR_CHAN_PRESS, &press);
			sensor_channel_get(lps22hb, SENSOR_CHAN_AMBIENT_TEMP,
					   &temp2);
		}

		receive_message(&msg, &len);
		/* Display sensor data */

		/* Erase previous */
		if (display_available)
			cfb_print(dev_display, "Zephyr", 0,
				  line++ * display_font_height);

		if (hts221_available) {
			/* temperature */
			snprintf(display_buf, sizeof(display_buf), "T: %1f C",
				 sensor_value_to_double(&temp1));
			if (display_available)
				cfb_print(dev_display, display_buf, 0,
					  line++ * display_font_height);

			/* humidity */
			snprintf(display_buf, sizeof(display_buf), "H: %1f%%",
				 sensor_value_to_double(&hum));
			if (display_available)
				cfb_print(dev_display, display_buf, 0,
					  line++ * display_font_height);
		}

		if (lps22hb_available) {
			/* pressure */
			/* lps22hb temperature */
			len = snprintf(message, sizeof(message),
				       "LPS22HB: Temperature: %.1f C\n",
				       sensor_value_to_double(&temp2));

			snprintf(display_buf, sizeof(display_buf), "P: %3f kpa",
				 sensor_value_to_double(&press) * 10.0);
			if (display_available)
				cfb_print(dev_display, display_buf, 0,
					  line++ * display_font_height);
		}

		if (display_available)
			cfb_framebuffer_finalize(dev_display);

		receive_message(&msg, &len);
	}

task_end:
	cleanup_system();

	printk("sensor task ended\n");
}

static int openamp_init(struct device *dev)
{
	int ret;

	/* Initialize platform */
	ret = platform_init();
	if (ret) {
		LOG_ERR("Failed to initialize platform\n");
		goto error_case;
	}

	rpdev = platform_create_rpmsg_vdev(0, VIRTIO_DEV_SLAVE, NULL,
					   new_service_cb);
	if (!rpdev) {
		LOG_ERR("Failed to create rpmsg virtio device\n");
		goto error_case;
	}

	LOG_DBG("Init OpenAMP rpdev = %p\n", rpdev);
	return 0;

error_case:
	cleanup_system();

	return 1;
}

void main(void)
{

	k_thread_create(&thread_data, thread_stack, APP_TASK_STACK_SIZE,
			(k_thread_entry_t)app_task,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, 0);

}

SYS_INIT(openamp_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

/*
 * Copyright (c) 2017 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sys/util.h>
#include <kernel.h>
#include <errno.h>
#include <drivers/i2c.h>
#include <string.h>
#include <openamp/open_amp.h>

#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(i2c_slave);

#if 0
#undef LOG_ERR
#define LOG_ERR printk
#undef LOG_DBG
#define LOG_DBG printk
#endif

#define RPMSG_I2C_NAME "rpmsg_i2c"
#define I2C_MASTER_READ   0x1

extern struct rpmsg_device *rpdev;

#define RPMSG_I2C_HEADER_SIZE 100

struct rpmsg_i2c_msg {
	u8_t addr;                      /*!< i2c slave addr                */
	u32_t len;                      /*!< Data Buffer size              */
	u8_t result;                    /*!< return value for the master   */
	u8_t buf[RPMSG_I2C_HEADER_SIZE];  /*!< i2c Data buffer             */
} rpmsg_i2c_msg;

#define RMSG_I2C_HEADER_SIZE (sizeof(struct rpmsg_i2c_msg)  - \
				  RPMSG_I2C_HEADER_SIZE)

struct i2c_rpmsg_data {
	sys_slist_t slaves;
	struct rpmsg_endpoint ept;          /*!< rpmsg endpoint */
	struct rpmsg_virtio_device *rvdev;  /*!< the rpmsg virtio device  */
};

#define DEV_DATA(dev)							\
	((struct i2c_rpmsg_data * const)(dev)->driver_data)

static struct i2c_slave_config *find_address(struct i2c_rpmsg_data *data,
					     u16_t address)
{
	struct i2c_slave_config *cfg = NULL;
	sys_snode_t *node;

	SYS_SLIST_FOR_EACH_NODE(&data->slaves, node) {
		cfg = CONTAINER_OF(node, struct i2c_slave_config, node);

		if (cfg->address == address) {
			return cfg;
		}
	}

	return NULL;
}

static int i2c_rpmsg_msg_sendack(struct rpmsg_endpoint *ept,
				 struct rpmsg_i2c_msg *msg, unsigned int ack)
{
	int res;

	msg->len = 0;
	msg->result = ack ? ack : 2;

	res = rpmsg_send(ept, msg, RMSG_I2C_HEADER_SIZE);
	if (res < 0) {
		return res;
	}

	return 0;
}

static int i2c_rpmsg_msg_write(struct rpmsg_endpoint *ept,
			       struct rpmsg_i2c_msg *msg,
			       struct i2c_slave_config *cfg)
{
	unsigned int len = 0U;
	u8_t *buf = msg->buf;
	int ret;

	cfg->callbacks->write_requested(cfg);

	LOG_DBG("%s: write %d bytes for slave %#x\n", __func__, msg->len,
		cfg->address);
	len = msg->len;
	while (len) {

		ret = cfg->callbacks->write_received(cfg, *buf);
		if (ret) {
			goto error;
		}
		buf++;
		len--;
	}

	cfg->callbacks->stop(cfg);

	i2c_rpmsg_msg_sendack(ept, msg, 1);
	return 0;
error:
	LOG_DBG("%s: NACK", __func__);

	i2c_rpmsg_msg_sendack(ept, msg, 0);
	return -EIO;
}

static int i2c_rpmsg_msg_read(struct rpmsg_endpoint *ept,
			      struct rpmsg_i2c_msg *msg,
			      struct i2c_slave_config *cfg)

{
	unsigned int len = msg->len;
	u8_t *buf = msg->buf;

	LOG_DBG("%s: read %d bytes for slave %#x\n", __func__, msg->len,
		cfg->address);
	if (!msg->len) {
		return 0;
	}

	cfg->callbacks->read_requested(cfg, buf);
	buf++;
	len--;

	while (len) {
		cfg->callbacks->read_processed(cfg, buf);
		buf++;
		len--;
	}

	cfg->callbacks->stop(cfg);

	rpmsg_send(ept, msg, len + RMSG_I2C_HEADER_SIZE);

	return 0;
}

static int i2c_rpmsg_slave_event(struct rpmsg_endpoint *ept, void *data,
				 size_t len, uint32_t src, void *priv)
{
	struct i2c_rpmsg_data *d_data = CONTAINER_OF(ept, struct i2c_rpmsg_data,
						     ept);
	struct rpmsg_i2c_msg *msg = data;
	struct i2c_slave_config *cfg;
	u8_t slave;
	int ret;

	slave = (msg->addr & (~I2C_MASTER_READ)) >> 1;
	cfg = find_address(d_data, slave);
	if (!cfg) {
		i2c_rpmsg_msg_sendack(ept, msg, 0);
		return 0;
	}

	if (msg->addr & I2C_MASTER_READ)
		ret = i2c_rpmsg_msg_read(ept, msg, cfg);
	else
		ret = i2c_rpmsg_msg_write(ept, msg, cfg);

	return 0;
}

int i2c_rpmsg_runtime_configure(struct device *dev, u32_t config)
{
	return 0;
}

static int i2c_rpmsg_slave_register(struct device *dev,
				    struct i2c_slave_config *config)
{
	struct i2c_rpmsg_data *data = DEV_DATA(dev);

	if (!config) {
		return -EINVAL;
	}

	/* Check the address is unique */
	if (find_address(data, config->address)) {
		return -EINVAL;
	}

	sys_slist_append(&data->slaves, &config->node);

	LOG_DBG("%s: register slave for address 0x%x\n", __func__,
		config->address);

	return 0;
}

static int i2c_rpmsg_slave_unregister(struct device *dev,
				      struct i2c_slave_config *config)
{
	struct i2c_rpmsg_data *data = DEV_DATA(dev);

	if (!config) {
		return -EINVAL;
	}

	if (!sys_slist_find_and_remove(&data->slaves, &config->node)) {
		return -EINVAL;
	}

	LOG_DBG("%s: unregister slave for address 0x%x\n", __func__,
		config->address);

	return 0;
}

static const struct i2c_driver_api api_funcs = {
	.configure = i2c_rpmsg_runtime_configure,
	.slave_register = i2c_rpmsg_slave_register,
	.slave_unregister = i2c_rpmsg_slave_unregister,
};

static int i2c_rpmsg_init(struct device *dev)
{
	struct i2c_rpmsg_data *data = DEV_DATA(dev);
	int status;

	sys_slist_init(&data->slaves);

	/* Create a endpoint for rmpsg communication */
	status = rpmsg_create_ept(&data->ept, rpdev, RPMSG_I2C_NAME,
				  RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
				  i2c_rpmsg_slave_event, NULL);

	if (status < 0) {
		LOG_ERR("Failed to create rpmsg endpoint\n");
		return -EPIPE;
	}

	return 0;
}

static struct i2c_rpmsg_data i2c_data;

DEVICE_AND_API_INIT(i2c_rpmsg, DT_INST_0_RPMSG_I2C_LABEL,
		    &i2c_rpmsg_init, &i2c_data, NULL,
		    APPLICATION, 0,
		    &api_funcs);

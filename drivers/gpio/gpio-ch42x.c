/**
 * i2c-gpio driver for chip CH423
 * 
 * Copyright (C) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * Web:		http://wch.cn
 * Author:	WCH <tech@wch.cn>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * Update Log:
 * V1.0 - initial version
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/version.h>

#define GPIO_NUM 24

#define CMD_SYSCFG_PARA (0x48 >> 1)
#define BIT_SLEEP_EN BIT(7)
#define BIT_OD_EN BIT(4)
#define BIT_IO_OE BIT(0)

#define CMD_GET_STATE (0x4d >> 1)
#define CMD_SET_IO (0x70 >> 1)
#define CMD_SET_OCL (0x44 >> 1)
#define CMD_SET_OCH (0x46 >> 1)

#define GPIO_DIR_OUT 0x00
#define GPIO_DIR_IN 0xff

struct ch42x_dev {
	struct device device;
	struct i2c_client *client;
	struct gpio_chip gpio;
	struct mutex io_mutex;
	u8 config;
	u16 io_dir;
	u8 gpio_outval;
	u8 oc_outval1;
	u8 oc_outval2;
	int gpio_base;
	int gpio_num;
};

static int ch42x_cfgpara_set(struct i2c_client *client, uint8_t config)
{
	struct i2c_msg msg;
	int error;

	msg.addr = CMD_SYSCFG_PARA;
	msg.flags = 0;
	msg.len = 1;
	msg.buf = &config;

	dev_dbg(&client->dev, "%s - addr:0x%02x val:0x%02x\n",
		__FUNCTION__, msg.addr, config);

	error = i2c_transfer(client->adapter, &msg, 1);
	if (error < 0) {
		return error;
	}
	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
static int ch42x_gpio_get_direction(struct gpio_chip *chip,
				    unsigned offset)
{
	struct ch42x_dev *ch42x =
		(struct ch42x_dev *)gpiochip_get_data(chip);
	struct i2c_client *client = ch42x->client;
	u16 dir;

	if (offset < 8) {
		mutex_lock(&ch42x->io_mutex);
		dir = ch42x->io_dir;
		mutex_unlock(&ch42x->io_mutex);
	} else {
		dir = GPIO_DIR_OUT;
	}
	if (dir)
		dev_dbg(&client->dev, "%s - IO%2d dir:IN\n", __FUNCTION__,
			offset);
	else
		dev_dbg(&client->dev, "%s - IO%2d dir:OUT\n", __FUNCTION__,
			offset);

	return dir;
}
#endif

static int ch42x_gpio_direction_input(struct gpio_chip *chip,
				      unsigned int offset)
{
	struct ch42x_dev *ch42x =
		(struct ch42x_dev *)gpiochip_get_data(chip);
	struct i2c_client *client = ch42x->client;
	int ret;

	if (offset >= 0 && offset < 8) { /* IO0~IO7 */
		mutex_lock(&ch42x->io_mutex);
		ch42x->config &= ~BIT_IO_OE;
		ch42x->io_dir = GPIO_DIR_IN;

		ret = ch42x_cfgpara_set(client, ch42x->config);
		if (ret < 0) {
			mutex_unlock(&ch42x->io_mutex);
			dev_err(&client->dev, "ch42x_cfgpara_set error!");
			return ret;
		}
		mutex_unlock(&ch42x->io_mutex);
	} else {
		dev_err(&client->dev, "offset error!");
		return -1;
	}

	return 0;
}

static int ch42x_gpio_direction_output(struct gpio_chip *chip,
				       unsigned int offset, int value)
{
	struct ch42x_dev *ch42x =
		(struct ch42x_dev *)gpiochip_get_data(chip);
	struct i2c_client *client = ch42x->client;
	int ret;

	mutex_lock(&ch42x->io_mutex);
	if (offset >= 0 && offset < 8) /* IO0~IO7 */
		ch42x->config |= BIT_IO_OE;
	else if (offset > 7 && offset < 24) /* OC0~OC15 */
		ch42x->config &= ~BIT_OD_EN;
	else {
		dev_err(&client->dev, "offset error!");
		return -1;
	}
	ch42x->io_dir = GPIO_DIR_OUT;

	ret = ch42x_cfgpara_set(client, ch42x->config);
	if (ret < 0) {
		mutex_unlock(&ch42x->io_mutex);
		dev_err(&client->dev, "ch42x_cfgpara_set error!");
		return ret;
	}
	mutex_unlock(&ch42x->io_mutex);

	return 0;
}

static int ch42x_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct ch42x_dev *ch42x =
		(struct ch42x_dev *)gpiochip_get_data(chip);
	struct i2c_client *client = ch42x->client;
	struct i2c_msg msg;
	uint8_t data;
	int gpiobit, error;

	mutex_lock(&ch42x->io_mutex);
	msg.addr = CMD_GET_STATE;
	msg.flags = I2C_M_RD;
	msg.len = 1;
	msg.buf = &data;

	error = i2c_transfer(client->adapter, &msg, 1);
	if (error < 0) {
		mutex_unlock(&ch42x->io_mutex);
		return error;
	}
	mutex_unlock(&ch42x->io_mutex);

	if (data & BIT(offset))
		gpiobit = 1;
	else
		gpiobit = 0;

	return gpiobit;
}

static void ch42x_gpio_set(struct gpio_chip *chip, unsigned offset,
			   int value)
{
	struct ch42x_dev *ch42x =
		(struct ch42x_dev *)gpiochip_get_data(chip);
	struct i2c_client *client = ch42x->client;
	struct i2c_msg msg;
	int error;

	mutex_lock(&ch42x->io_mutex);
	if (offset >= 0 && offset <= 7) { /* IO0~IO7 */
		if (value == 1)
			ch42x->gpio_outval |= BIT(offset);
		else if (value == 0)
			ch42x->gpio_outval &= ~BIT(offset);
		else
			return;
		msg.addr = CMD_SET_IO;
		msg.buf = &ch42x->gpio_outval;
	} else if (offset >= 8 && offset <= 15) { /* OC0~OC7 */
		if (value == 1)
			ch42x->oc_outval1 |= BIT(offset - 8);
		else if (value == 0)
			ch42x->oc_outval1 &= ~BIT(offset - 8);
		else
			return;
		msg.addr = CMD_SET_OCL;
		msg.buf = &ch42x->oc_outval1;
	} else if (offset >= 16) { /* OC8~OC15 */
		if (value == 1)
			ch42x->oc_outval2 |= BIT(offset - 16);
		else if (value == 0)
			ch42x->oc_outval2 &= ~BIT(offset - 16);
		else
			return;
		msg.addr = CMD_SET_OCH;
		msg.buf = &ch42x->oc_outval2;
	}

	msg.flags = 0;
	msg.len = 1;

	dev_dbg(&client->dev,
		"i2c write - addr:0x%2x offset:%2d outval:0x%2x outval1:0x%2x outval1:0x%2x\n",
		msg.addr, offset, ch42x->gpio_outval, ch42x->oc_outval1,
		ch42x->oc_outval2);

	error = i2c_transfer(client->adapter, &msg, 1);
	if (error < 0) {
		mutex_unlock(&ch42x->io_mutex);
		return;
	}
	mutex_unlock(&ch42x->io_mutex);
}

static int ch42x_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	int ret;
	struct ch42x_dev *ch42x;

	ch42x = devm_kzalloc(&client->dev, sizeof(*ch42x), GFP_KERNEL);
	if (!ch42x)
		return -ENOMEM;

	i2c_set_clientdata(client, ch42x);
	ch42x->client = client;
	ch42x->device = client->dev;
	ch42x->gpio_num = GPIO_NUM;

	ch42x->io_dir = GPIO_DIR_IN;
	ch42x->config = 0;
	ch42x->gpio_outval = 0;
	ch42x->oc_outval1 = 0;
	ch42x->oc_outval2 = 0;

	mutex_init(&ch42x->io_mutex);

	ch42x->gpio.label = "ch42x-gpio";
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 5, 0)
	ch42x->gpio.parent = &ch42x->device;
#else
	ch42x->gpio.dev = &ch42x->device;
#endif
	ch42x->gpio.owner = THIS_MODULE;
	ch42x->gpio.base = -1;
	ch42x->gpio.ngpio = GPIO_NUM;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)
	ch42x->gpio.get_direction = ch42x_gpio_get_direction;
#endif
	ch42x->gpio.direction_input = ch42x_gpio_direction_input;
	ch42x->gpio.direction_output = ch42x_gpio_direction_output;
	ch42x->gpio.get = ch42x_gpio_get;
	ch42x->gpio.set = ch42x_gpio_set;

	ret = gpiochip_add_data(&ch42x->gpio, ch42x);
	if (ret < 0) {
		ch42x->gpio.base = -1;
		goto err1;
	}
	dev_info(&client->dev, "registered GPIOs from %d to %d\n",
		 ch42x->gpio.base,
		 ch42x->gpio.base + ch42x->gpio.ngpio - 1);
	return 0;

err1:
	mutex_destroy(&ch42x->io_mutex);
	return -1;
}

static void ch42x_remove(struct i2c_client *client)
{
	struct ch42x_dev *ch42x = i2c_get_clientdata(client);

	mutex_destroy(&ch42x->io_mutex);
	gpiochip_remove(&ch42x->gpio);
}

static const struct of_device_id ch42x_dt_ids[] = {
	{ .compatible = "wch,ch422" },
	{ .compatible = "wch,ch423" },
    {}
};
MODULE_DEVICE_TABLE(of, ch42x_dt_ids);

static const struct i2c_device_id ch42x_ids[] = {
	{ "ch422", 0 },
	{ "ch423", 1 },
};
MODULE_DEVICE_TABLE(i2c, ch42x_ids);

static struct i2c_driver ch42x_driver = {
    .driver = {
        .name = "ch42x",
        .of_match_table = ch42x_dt_ids,
    },
    .probe = ch42x_probe,
    .remove = ch42x_remove,
    .id_table = ch42x_ids,
};

module_i2c_driver(ch42x_driver);
MODULE_DESCRIPTION("GPIO expander driver for CH42x");
MODULE_LICENSE("GPL");

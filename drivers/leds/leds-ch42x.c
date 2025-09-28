/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * CH422/423 I2C Led dimmer
 *
 * Driver for 
 *
 * Copyright (C) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * Author:       WCH <tech@wch.cn>
 * Maintainer:   Sergey Kharenko <skharenko@hust.edu.cn>
 *
 * This driver provides support for the CH390H/D Ethernet controller
 * connected via SPI. It integrates with the Linux networking stack
 * through the standard net_device interface.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/leds-pca9532.h>
#include <linux/gpio/driver.h>
#include <linux/of.h>
#include <linux/of_device.h>


#define CND_SET_PARAM       (0x48 >> 1)
#define CMD_SET_OCL         (0x44 >> 1)
#define CMD_SET_OCH         (0x46 >> 1)

#define CMD_SET_IO_VAL      (0x60 >> 1)
#define CMD_SET_IO_VAL      (0x60 >> 1)
#define CMD_GET_IO_VAL      (0x4D >> 1)

#define CMD_LED_VAL         CMD_SET_IO_VAL

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



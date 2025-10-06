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
#include <linux/gpio/driver.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define DIG_GROUP_SZ        8

#define CMD_SET_PARAM       (0x48 >> 1)
#define CMD_SET_LED16       (0x60 >> 1)
#define CMD_SET_LED8       	(0x70 >> 1)

#define PARAM_SLEEP         0x80
#define PARAM_LED16_EN      0x07
#define PARAM_LED8_EN       0x05

#define PARAM_LED_BRT_FULL  0x00
#define PARAM_LED_BRT_HALF  0x40
#define PARAM_LED_BRT_QTR   0x20

#define CH42X_RETURN_ON_ERROR(x, format, ...) do { 								\
		ret=(x);																\
		if(ret<0)																\
			dev_err(&client->dev, "%s: "format, __func__);						\
			return ret; 														\
	} while(0)

#define CH42X_GOTO_ON_ERROR(x, tag, format, ...) do { 						    \
		ret=(x);																\
		if(ret<0)																\
			dev_err(&client->dev, "%s: "format, __func__);						\
			goto tag; 														    \
	} while(0)

struct ch42x_chip_info {
	u16 num_leds;
    u8 num_digs;
    u8 cmd_leds;
    u8 par_led_en;
    bool has_brt;
};

struct ch42x_led {
    struct led_classdev ldev;
    u8 id;
    struct ch42x_data *parent;
};

struct ch42x_data {
	struct i2c_client *client;
	struct mutex io_mutex;
    struct ch42x_led *leds;
    u8 *reg_leds;
	const struct ch42x_chip_info *info;
};

#define ldev_to_led(c)         container_of(c, struct ch42x_led, ldev)

enum {
	ch422,
	ch423,
};

static const struct ch42x_chip_info ch42x_chip_info_tbl[] = {
	[ch422] = {
		.num_leds = 32,
        .num_digs = 4,
        .cmd_leds = CMD_SET_LED8,
        .par_led_en = PARAM_LED8_EN,
        .has_brt = false
	},
	[ch423] = {
		.num_leds = 128,
        .num_digs = 16,
        .cmd_leds = CMD_SET_LED16,
        .par_led_en = PARAM_LED16_EN,
        .has_brt = true
	},
};

static inline int ch42x_led_set(struct ch42x_data *dev, u8 did, u8 sid) {
    int ret = 0;
    struct i2c_client *client = dev->client;
    u8 param = 0;
    struct i2c_msg msg = {
        .len = 1,
        .buf = &param,
        .flags = 0
    };

    if(did>=dev->info->num_digs || sid >= DIG_GROUP_SZ)
        return -ENXIO;

    msg.addr = dev->info->cmd_leds | did;
    param = dev->reg_leds[did] | (1<<sid);

    mutex_lock(&dev->io_mutex);
    CH42X_GOTO_ON_ERROR(i2c_transfer(client->adapter, &msg, 1), err, "set led bit failed\n");
    dev->reg_leds[did] |= (1<<sid);

err:
    mutex_unlock(&dev->io_mutex);
    return ret;
}

static inline int ch42x_led_clr(struct ch42x_data *dev, u8 did, u8 sid) {
    int ret = 0;
    struct i2c_client *client = dev->client;
    u8 param = 0;
    struct i2c_msg msg = {
        .len = 1,
        .buf = &param,
        .flags = 0
    };

    if(did>=dev->info->num_digs || sid >= DIG_GROUP_SZ)
        return -ENXIO;

    msg.addr = dev->info->cmd_leds | did;
    param = dev->reg_leds[did] & ~(1<<sid);

    mutex_lock(&dev->io_mutex);
    CH42X_GOTO_ON_ERROR(i2c_transfer(client->adapter, &msg, 1), err, "clear led bit failed\n");
    dev->reg_leds[did] &= ~(1<<sid);

err:
    mutex_unlock(&dev->io_mutex);
    return ret;
}

static inline int ch42x_led_global_brt(struct ch42x_data *dev, u8 brt) {
    int ret = 0;
    struct i2c_client *client = dev->client;
    u8 param = brt | dev->info->par_led_en;
    struct i2c_msg msg = {
        .addr = CMD_SET_PARAM,
        .len = 1,
        .buf = &param,
        .flags = 0
    };
    
    if(!dev->info->has_brt)
        return -EPERM;

    mutex_lock(&dev->io_mutex);
    CH42X_GOTO_ON_ERROR(i2c_transfer(client->adapter, &msg, 1), err, "set led global brightness failed\n");
    
err:
    mutex_unlock(&dev->io_mutex);
    return ret;
}

static inline void ch42x_led_brt_update(struct ch42x_data *dev, u8 brt) {
    for(u8 did = 0;did<dev->info->num_digs;did++){
        for(u8 sid = 0;sid<DIG_GROUP_SZ;sid++){
            if(dev->reg_leds[did] & (1<<sid)){
                dev->leds[did*DIG_GROUP_SZ+sid].ldev.brightness = brt;
            }
        }
    }
}

static int ch42x_brightness_set_blocking(struct led_classdev *led_cdev,
				       enum led_brightness brightness){
    int ret = 0;
    struct ch42x_led *pled = ldev_to_led(led_cdev);
    struct ch42x_data *ch42x = pled->parent;
    struct i2c_client *client = ch42x->client;

    u8 did = pled->id/DIG_GROUP_SZ;
    u8 sid = pled->id%DIG_GROUP_SZ;

    u8 global_brt = 0;

    if(ch42x->info->has_brt) {
        if(brightness == LED_OFF){
            CH42X_RETURN_ON_ERROR(ch42x_led_clr(ch42x, did, sid), "fail to turn off the led\n");
            ch42x->leds[pled->id].ldev.brightness = LED_OFF;
        }
        else {
            CH42X_RETURN_ON_ERROR(ch42x_led_set(ch42x, did, sid), "fail to turn on the led\n");
            if(brightness<=LED_FULL/4) {
                CH42X_RETURN_ON_ERROR(ch42x_led_global_brt(ch42x,PARAM_LED_BRT_QTR),"fail to set led brightness\n");
                ch42x->leds[pled->id].ldev.brightness = LED_FULL/4;
                global_brt = LED_FULL/4;
            }
            else if(brightness <= LED_HALF) {
                CH42X_RETURN_ON_ERROR(ch42x_led_global_brt(ch42x,PARAM_LED_BRT_QTR),"fail to set led brightness\n");
                ch42x->leds[pled->id].ldev.brightness = LED_HALF;
                global_brt = LED_HALF;
            }
            else {
                CH42X_RETURN_ON_ERROR(ch42x_led_global_brt(ch42x,PARAM_LED_BRT_FULL),"fail to set led brightness\n");
                ch42x->leds[pled->id].ldev.brightness = LED_FULL;
                global_brt = LED_FULL;
            }
            ch42x_led_brt_update(ch42x,global_brt);
        }
    }
    else {
        if(brightness == LED_OFF){
            CH42X_RETURN_ON_ERROR(ch42x_led_clr(ch42x, did, sid), "fail to turn off the led\n");
            ch42x->leds[pled->id].ldev.brightness = LED_OFF;
        }
        else {
            CH42X_RETURN_ON_ERROR(ch42x_led_set(ch42x, did, sid), "fail to turn on the led\n");
            ch42x->leds[pled->id].ldev.brightness = LED_ON;
        }
    }
}

static int ch42x_init(struct ch42x_data *dev) {
    struct i2c_client *client = dev->client;
    int ret;
    u8 param = dev->info->par_led_en;
    struct i2c_msg msg = {
        .addr = CMD_SET_PARAM,
        .len = 1,
        .buf = &param,
        .flags = 0
    };

    mutex_lock(&dev->io_mutex);
    CH42X_GOTO_ON_ERROR(i2c_transfer(client->adapter, &msg, 1), err, "init led mode failed\n");

    param = 0;
    for(u8 did=0;did<dev->info->num_digs;did++){
        msg.addr = dev->info->cmd_leds | did;
        CH42X_GOTO_ON_ERROR(i2c_transfer(client->adapter, &msg, 1), err, "write led value failed\n");
        dev->reg_leds[did]= 0;
    }

err:
    mutex_unlock(&dev->io_mutex);

    return 0;
}

static int ch42x_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct ch42x_data *ch42x;
    int devid;
    int ret = 0;

	ch42x = devm_kzalloc(&client->dev, sizeof(struct ch42x_data), GFP_KERNEL);
	if (!ch42x)
		return -ENOMEM;

	i2c_set_clientdata(client, ch42x);
	ch42x->client = client;
    mutex_init(&ch42x->io_mutex);

    devid = (int)(uintptr_t)of_device_get_match_data(&client->dev);
    ch42x->info = &ch42x_chip_info_tbl[devid];
    ch42x->leds = devm_kzalloc(&client->dev, sizeof(struct ch42x_led)*ch42x->info->num_leds, GFP_KERNEL);
    ch42x->reg_leds = devm_kzalloc(&client->dev,ch42x->info->num_digs,GFP_KERNEL);
	
    for(u16 i=0;i<ch42x->info->num_leds;i++){
        struct ch42x_led *pled = &ch42x->leds[i];
        pled->id= i ;
        pled->ldev.name = "";
        pled->ldev.brightness = LED_OFF;
        pled->ldev.max_brightness = LED_ON;
        pled->ldev.brightness_set_blocking = ch42x_brightness_set_blocking;
        pled->parent = ch42x;
        devm_led_classdev_register(&client->dev, &pled->ldev);
    }

    CH42X_GOTO_ON_ERROR(ch42x_init(ch42x), err, "init device failed\n");
	return 0;

err:
	mutex_destroy(&ch42x->io_mutex);
	return ret;
}

static void ch42x_remove(struct i2c_client *client)
{
	struct ch42x_data *ch42x = i2c_get_clientdata(client);
	mutex_destroy(&ch42x->io_mutex);
}

static const struct of_device_id ch42x_dt_ids[] = {
	{ .compatible = "wch,ch422-leds", .data = (void *)ch422},
	{ .compatible = "wch,ch423-leds", .data = (void *)ch423},
    {}
};
MODULE_DEVICE_TABLE(of, ch42x_dt_ids);

static const struct i2c_device_id ch42x_ids[] = {
	{ "ch422-leds", ch422 },
	{ "ch423-leds", ch423 },
};
MODULE_DEVICE_TABLE(i2c, ch42x_ids);

static struct i2c_driver ch42x_driver = {
    .driver = {
        .name = "ch42x-leds",
        .of_match_table = ch42x_dt_ids,
    },
    .probe = ch42x_probe,
    .remove = ch42x_remove,
    .id_table = ch42x_ids,
};

module_i2c_driver(ch42x_driver);
MODULE_DESCRIPTION("GPIO expander driver for CH42x");
MODULE_LICENSE("GPL");
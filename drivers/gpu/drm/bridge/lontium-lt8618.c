// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025. Sergey Kharenko.
 * Copyright (c) 2024, VeriSilicon Holdings Co., Ltd. All rights reserved
 */

#include <linux/gpio/consumer.h>
#include <linux/i2c-mux.h>
#include <linux/i2c.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_drv.h>
#include <drm/drm_edid.h>
#include <drm/drm_modes.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include <sound/hdmi-codec.h>
#include <video/videomode.h>

#define LT8618_PAGE_CONTROL 		0xFF

#define LT8618_CHIPID0				0x17
#define LT8618_CHIPID1				0x02
#define LT8618_CHIPVER_U2			0xE1
#define LT8618_CHIPVER_U3			0xE2

#define LT8618_REGADDR_READ_EN		0x80EE
#define LT8618_REGADDR_CHIPID0		0x8000
#define LT8618_REGADDR_CHIPID1		0x8001
#define LT8618_REGADDR_CHIPID2		0x8002
#define LT8618_REGADDR_INTCLR_FLAG0	0x8204
#define LT8618_REGADDR_INTCLR_FLAG1	0x8205
#define LT8618_REGADDR_INTCLR_FLAG2	0x8206
#define LT8618_REGADDR_INTCLR_FLAG3	0x8207
#define LT8618_REGADDR_INT_FLAG0	0x820C
#define LT8618_REGADDR_INT_FLAG1	0x820D
#define LT8618_REGADDR_INT_FLAG2	0x820E
#define LT8618_REGADDR_INT_FLAG3	0x820F

struct lt8618 {
	struct device *dev;
	struct drm_bridge bridge;
	struct drm_connector connector;

	struct regmap *regmap;

	struct device_node *rgb_node;
	struct platform_device *audio_pdev;

	bool ac_mode;

	struct gpio_desc *reset_gpio;
	struct gpio_desc *enable_gpio;

	bool power_on;
	bool sleep;

	struct regulator_bulk_data supplies[2];

	struct i2c_client *client;

	enum drm_connector_status status;

	u8 chip_ver;
	u8 edid_buf[EDID_SEG_SIZE];
	u32 vic;
};

struct lt8618_mode {
	u16 hdisplay;
	u16 vdisplay;
	u8 vrefresh;
};

///////////////// Const Config ///////////////////
static const struct regmap_range_cfg lt8618_ranges[] = {
	{
		.name = "register_range",
		.range_min = 0,
		.range_max = 0x85FF,
		.selector_reg = LT8618_PAGE_CONTROL,
		.selector_mask = 0xFF,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 0x100,
	},
};

static const struct regmap_config lt8618_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFFFF,
	.ranges = lt8618_ranges,
	.num_ranges = ARRAY_SIZE(lt8618_ranges),
};

static const struct lt8618_mode lt8618_modes[] = {
	{ 640, 480, 60 },   { 720, 480, 60 },	{ 800, 600, 60 },
	{ 1024, 768, 60 },  { 1280, 720, 30 },	{ 1280, 720, 60 },
	{ 1280, 1024, 60 }, { 1920, 1080, 30 }, { 1920, 1080, 50 },
	{ 1920, 1080, 60 }, { 3840, 2160, 30 },
};
////////////////   Layer 0   ////////////////////
static struct lt8618 *bridge_to_lt8618(struct drm_bridge *bridge)
{
	return container_of(bridge, struct lt8618, bridge);
}

static struct lt8618 *connector_to_lt8618(struct drm_connector *connector)
{
	return container_of(connector, struct lt8618, connector);
}

////////////////   Layer 1   ////////////////////
static struct lt8618_mode *lt8618_find_mode(const struct drm_display_mode *mode)
{
	for (int i = 0; i < ARRAY_SIZE(lt8618_modes); i++) {
		if (lt8618_modes[i].hdisplay == mode->hdisplay &&
		    lt8618_modes[i].vdisplay == mode->vdisplay &&
		    lt8618_modes[i].vrefresh == drm_mode_vrefresh(mode)) {
			return &lt8618_modes[i];
		}
	}
	return NULL;
}

static bool lt8618_check_format(u32 bus_format)
{
	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB888_1X24:
	case MEDIA_BUS_FMT_RBG888_1X24:
	case MEDIA_BUS_FMT_BGR888_1X24:
	case MEDIA_BUS_FMT_GBR888_1X24:
	case MEDIA_BUS_FMT_VUY8_1X24:
	case MEDIA_BUS_FMT_YUV8_1X24:
		return true;
	default:
		return false;
	}
}

////////////////   Layer 2   ////////////////////
static int lt8618_parse_dt(struct device *dev, struct lt8618 *lt8618)
{
	lt8618->rgb_node = of_graph_get_remote_node(dev->of_node, 0, -1);
	if (!lt8618->rgb_node) {
		dev_err(lt8618->dev,
			"failed to get remote node for rgb\n");
		return -ENODEV;
	}

	lt8618->ac_mode = of_property_read_bool(dev->of_node, "lt,ac-mode");
	return 0;
}

static int lt8618_gpio_init(struct lt8618 *lt8618)
{
	struct device *dev = lt8618->dev;

	lt8618->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(lt8618->reset_gpio)) {
		dev_err(dev, "failed to acquire reset gpio\n");
		return PTR_ERR(lt8618->reset_gpio);
	}

	lt8618->enable_gpio = devm_gpiod_get_optional(dev, "enable",
						      GPIOD_OUT_LOW);
	if (IS_ERR(lt8618->enable_gpio)) {
		dev_err(dev, "failed to acquire enable gpio\n");
		return PTR_ERR(lt8618->enable_gpio);
	}

	return 0;
}

static int lt8618_read_device_rev(struct lt8618 *lt8618)
{
	unsigned int rev[3];

	regmap_write(lt8618->regmap, LT8618_REGADDR_READ_EN, 0x01);
	regmap_read(lt8618->regmap, LT8618_REGADDR_CHIPID0, rev);
	regmap_read(lt8618->regmap, LT8618_REGADDR_CHIPID1, rev+1);
	regmap_read(lt8618->regmap, LT8618_REGADDR_CHIPID2, rev+2);

	lt8618->chip_ver=rev[2];
	if(rev[0]!=LT8618_CHIPID0 || (rev[1]!=LT8618_CHIPID1)){
		dev_err(lt8618->dev, "chip id is incorrect\n");
		return -ENODEV;
	}
		
	switch(rev[2]){
		case LT8618_CHIPVER_U2:
			dev_info(lt8618->dev, "chip version: U2\n");
			break;
		case LT8618_CHIPVER_U3:
			dev_info(lt8618->dev, "chip version: U3\n");
			break;
		default:
			dev_err(lt8618->dev, "unknow chip id: %X\n", rev[2]);
			return -ENODEV;
	}

	return 0;
}

static int lt8618_regulator_init(struct lt8618 *lt8618)
{
	int ret;

	lt8618->supplies[0].supply = "vdd";
	lt8618->supplies[1].supply = "vcc";

	ret = devm_regulator_bulk_get(lt8618->dev, 2, lt8618->supplies);
	if (ret < 0)
		return ret;

	return regulator_set_load(lt8618->supplies[0].consumer, 300000);
}

static int lt8618_regulator_enable(struct lt8618 *lt8618)
{
	int ret;

	ret = regulator_enable(lt8618->supplies[0].consumer);
	if (ret < 0)
		return ret;

	usleep_range(1000, 10000);

	ret = regulator_enable(lt8618->supplies[1].consumer);
	if (ret < 0) {
		regulator_disable(lt8618->supplies[0].consumer);
		return ret;
	}

	return 0;
}

static irqreturn_t lt8618_irq_thread_handler(int irq, void *dev_id)
{
	struct lt8618 *lt8618 = dev_id;
	unsigned int irq_flag0 = 0;
	unsigned int irq_flag3 = 0;

	regmap_read(lt8618->regmap, LT8618_REGADDR_INT_FLAG0, &irq_flag0);
	regmap_read(lt8618->regmap, LT8618_REGADDR_INT_FLAG3, &irq_flag3);

	/* hpd changed low */
	if (irq_flag3 & 0x80) {
		dev_info(lt8618->dev, "hdmi cable disconnected\n");

		regmap_write(lt8618->regmap, LT8618_REGADDR_INTCLR_FLAG3, 0xbf);
		regmap_write(lt8618->regmap, LT8618_REGADDR_INTCLR_FLAG3, 0x3f);
	}

	/* hpd changed high */
	if (irq_flag3 & 0x40) {
		dev_info(lt8618->dev, "hdmi cable connected\n");

		regmap_write(lt8618->regmap, LT8618_REGADDR_INTCLR_FLAG3, 0x7f);
		regmap_write(lt8618->regmap, LT8618_REGADDR_INTCLR_FLAG3, 0x3f);
	}

	if (irq_flag3 & 0xc0 && lt8618->bridge.dev)
		drm_kms_helper_hotplug_event(lt8618->bridge.dev);

	/* video input changed */
	if (irq_flag0 & 0x01) {
		dev_info(lt8618->dev, "video input changed\n");
		regmap_write(lt8618->regmap, 0x829e, 0xff);
		regmap_write(lt8618->regmap, 0x829e, 0xf7);
		regmap_write(lt8618->regmap, 0x8204, 0xff);
		regmap_write(lt8618->regmap, 0x8204, 0xfe);
	}

	return IRQ_HANDLED;
}

////////////////   Layer 3   ////////////////////
static int lt8618_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct lt8618 *lt8618;
	struct device *dev = &client->dev;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "device doesn't support I2C\n");
		return -ENODEV;
	}

	lt8618 = devm_kzalloc(dev, sizeof(*lt8618), GFP_KERNEL);
	if (!lt8618)
		return -ENOMEM;

	lt8618->dev = dev;
	lt8618->client = client;
	lt8618->sleep = false;

	lt8618->regmap = devm_regmap_init_i2c(client, &lt8618_regmap_config);
	if (IS_ERR(lt8618->regmap)) {
		dev_err(lt8618->dev, "regmap i2c init failed\n");
		return PTR_ERR(lt8618->regmap);
	}

	ret = lt8618_parse_dt(dev, lt8618);
	if (ret) {
		dev_err(dev, "failed to parse device tree\n");
		return ret;
	}

	ret = lt8618_gpio_init(lt8618);
	if (ret < 0)
		goto err_of_put;

	ret = lt8618_regulator_init(lt8618);
	if (ret < 0)
		goto err_of_put;

	lt8618_assert_5v(lt8618);

	ret = lt8618_regulator_enable(lt8618);
	if (ret)
		goto err_of_put;

	lt8618_reset(lt8618);

	ret = lt8618_read_device_rev(lt8618);
	if (ret) {
		dev_err(dev, "failed to read chip rev\n");
		goto err_disable_regulators;
	}

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					lt8618_irq_thread_handler, IRQF_ONESHOT,
					"lt8618", lt8618);
	if (ret) {
		dev_err(dev, "failed to request irq\n");
		goto err_disable_regulators;
	}

	i2c_set_clientdata(client, lt8618);

	lt8618->bridge.funcs = &lt8618_bridge_funcs;
	lt8618->bridge.of_node = client->dev.of_node;
	lt8618->bridge.ops = DRM_BRIDGE_OP_DETECT | DRM_BRIDGE_OP_EDID |
			     DRM_BRIDGE_OP_HPD | DRM_BRIDGE_OP_MODES;
	lt8618->bridge.type = DRM_MODE_CONNECTOR_HDMIA;

	drm_bridge_add(&lt8618->bridge);

	/* Attach primary DSI */
	lt8618->dsi0 = lt8618_attach_dsi(lt8618, lt8618->dsi0_node);
	if (IS_ERR(lt8618->dsi0)) {
		ret = PTR_ERR(lt8618->dsi0);
		goto err_remove_bridge;
	}

	/* Attach secondary DSI, if specified */
	if (lt8618->dsi1_node) {
		lt8618->dsi1 = lt8618_attach_dsi(lt8618, lt8618->dsi1_node);
		if (IS_ERR(lt8618->dsi1)) {
			ret = PTR_ERR(lt8618->dsi1);
			goto err_remove_bridge;
		}
	}

	lt8618_enable_hpd_interrupts(lt8618);

	ret = lt8618_audio_init(dev, lt8618);
	if (ret)
		goto err_remove_bridge;

	return 0;

err_remove_bridge:
	drm_bridge_remove(&lt8618->bridge);

err_disable_regulators:
	regulator_bulk_disable(ARRAY_SIZE(lt8618->supplies), lt8618->supplies);

err_of_put:
	of_node_put(lt8618->dsi1_node);
	of_node_put(lt8618->dsi0_node);

	return ret;
}

static void lt8618_remove(struct i2c_client *client)
{
	struct lt8618 *lt8618 = i2c_get_clientdata(client);

	disable_irq(client->irq);
	lt8618_audio_exit(lt8618);
	drm_bridge_remove(&lt8618->bridge);

	regulator_bulk_disable(ARRAY_SIZE(lt8618->supplies), lt8618->supplies);

	of_node_put(lt8618->rgb_node);
}

static const struct of_device_id lt8618_dt_ids[] = {
	{
		.compatible = "lontium,lt8618",
	},
	{}
};
MODULE_DEVICE_TABLE(of, lt8618_dt_ids);

static const struct i2c_device_id lt8618_i2c_ids[] = {
	{ "lt8618", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, lt8618_i2c_ids);

static struct i2c_driver lt8618_driver = {
	 .probe = lt8618_probe,
	 .remove = lt8618_remove,
	 .driver = {
		 .name = "lt8618",
		 .of_match_table = lt8618_dt_ids,
	 },
	 .id_table = lt8618_i2c_ids,
 };
module_i2c_driver(lt8618_driver);

MODULE_AUTHOR("Sergey Kharenko <u201810610@hust.edu.cn>");
MODULE_DESCRIPTION("LT8618 RGB -> HDMI bridges");
MODULE_LICENSE("GPL");
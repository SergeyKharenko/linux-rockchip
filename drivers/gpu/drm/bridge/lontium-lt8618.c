// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025. Sergey Kharenko.
 * Copyright (c) 2024, VeriSilicon Holdings Co., Ltd. All rights reserved
 */

// TODO need further development

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

#define EDID_SEG_SIZE 256
#define EDID_LEN 32
#define EDID_LOOP 8

#define LT8618_PAGE_CONTROL 0xFF

#define LT8618_CHIPID0 0x17
#define LT8618_CHIPID1 0x02
#define LT8618_CHIPVER_U2 0xE1
#define LT8618_CHIPVER_U3 0xE2

#define LT8618_REGADDR_READ_EN 0x80EE
#define LT8618_REGADDR_CHIPID0 0x8000
#define LT8618_REGADDR_CHIPID1 0x8001
#define LT8618_REGADDR_CHIPID2 0x8002
#define LT8618_REGADDR_INTCLR_FLAG0 0x8204
#define LT8618_REGADDR_INTCLR_FLAG1 0x8205
#define LT8618_REGADDR_INTCLR_FLAG2 0x8206
#define LT8618_REGADDR_INTCLR_FLAG3 0x8207
#define LT8618_REGADDR_INT_FLAG0 0x820C
#define LT8618_REGADDR_INT_FLAG1 0x820D
#define LT8618_REGADDR_INT_FLAG2 0x820E
#define LT8618_REGADDR_INT_FLAG3 0x820F
#define LT8618_REGADDR_HDMI_TX_STA 0x825E
#define LT8618_REGADDR_SYNC_SYS_INTCLR 0x829E
#define LT8618_REGADDR_HDMI_I2C_DDC 0x8503
#define LT8618_REGADDR_EDID_DEV_ADDR 0x8504
#define LT8618_REGADDR_EDID_OFFSET_ADDR 0x8505
#define LT8618_REGADDR_EDID_READ_LEN 0x8506
#define LT8618_REGADDR_EDID_ACCESS_CMD 0x8507
#define LT8618_REGADDR_HDMI_AEC 0x8514
#define LT8618_REGADDR_DDC_STA 0x8540
#define LT8618_REGADDR_EDID_FIFO 0x8583

#define LT8618_INT_FLAG0_VID_CHECK 0x01

#define LT8618_INT_FLAG3_HDMI_UNPLUG 0x80
#define LT8618_INT_FLAG3_HDMI_PLUG 0x40

#define LT8618_HDMI_STA_TX_HPD 0x04
#define LT8618_HDMI_STA_TX_DC_DET 0x01

#define LT8618_DDC_STA_ACCS_DONE 0x02
#define LT8618_DDC_STA_NO_ACK 0x50

struct lt8618
{
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

struct lt8618_mode
{
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
	{640, 480, 60},
	{720, 480, 60},
	{800, 600, 60},
	{1024, 768, 60},
	{1280, 720, 30},
	{1280, 720, 60},
	{1280, 1024, 60},
	{1920, 1080, 30},
	{1920, 1080, 50},
	{1920, 1080, 60},
	{3840, 2160, 30},
};

static int lt8618_bridge_attach(struct drm_bridge *bridge,
								enum drm_bridge_attach_flags flags);

static enum drm_mode_status lt8618_bridge_mode_valid(struct drm_bridge *bridge,
													 const struct drm_display_info *info,
													 const struct drm_display_mode *mode);

static void lt8618_bridge_mode_set(struct drm_bridge *bridge,
								   const struct drm_display_mode *mode,
								   const struct drm_display_mode *adj_mode);

static enum drm_connector_status lt8618_bridge_detect(struct drm_bridge *bridge);

static struct edid *lt8618_bridge_get_edid(struct drm_bridge *bridge,
										   struct drm_connector *connector);

static void lt8618_bridge_hpd_enable(struct drm_bridge *bridge);

static void lt8618_bridge_atomic_enable(struct drm_bridge *bridge,
										struct drm_bridge_state *old_bridge_state);

static void lt8618_bridge_atomic_disable(struct drm_bridge *bridge,
										 struct drm_bridge_state *old_bridge_state);

static void lt8618_bridge_atomic_post_disable(struct drm_bridge *bridge,
											  struct drm_bridge_state *old_bridge_state);

static const struct drm_bridge_funcs lt8618_bridge_funcs = {
	.attach = lt8618_bridge_attach,
	.mode_valid = lt8618_bridge_mode_valid,
	.mode_set = lt8618_bridge_mode_set,
	.detect = lt8618_bridge_detect,
	.get_edid = lt8618_bridge_get_edid,
	.hpd_enable = lt8618_bridge_hpd_enable,

	.atomic_enable = lt8618_bridge_atomic_enable,
	.atomic_disable = lt8618_bridge_atomic_disable,
	.atomic_post_disable = lt8618_bridge_atomic_post_disable,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
	.atomic_get_input_bus_fmts = lt8618_atomic_get_input_bus_fmts,
};

static enum drm_connector_status lt8618_connector_detect(struct drm_connector *connector, bool force);

static const struct drm_connector_funcs lt8618_bridge_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = lt8618_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int lt8618_connector_get_modes(struct drm_connector *connector);

static enum drm_mode_status lt8618_connector_mode_valid(struct drm_connector *connector,
														struct drm_display_mode *mode);

static struct drm_connector_helper_funcs lt8618_bridge_connector_helper_funcs = {
	.get_modes = lt8618_connector_get_modes,
	.mode_valid = lt8618_connector_mode_valid,
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

static enum drm_connector_status lt8618_detect(struct lt8618 *lt8618)
{
	unsigned int reg_val = 0;
	int connected = 0;

	regmap_read(lt8618->regmap, LT8618_REGADDR_HDMI_TX_STA, &reg_val);
	connected = (reg_val & (LT8618_HDMI_STA_TX_HPD | LT8618_HDMI_STA_TX_DC_DET));

	lt8618->status = connected ? connector_status_connected : connector_status_disconnected;

	return lt8618->status;
}

static struct lt8618_mode *lt8618_find_mode(const struct drm_display_mode *mode)
{
	for (int i = 0; i < ARRAY_SIZE(lt8618_modes); i++)
	{
		if (lt8618_modes[i].hdisplay == mode->hdisplay &&
			lt8618_modes[i].vdisplay == mode->vdisplay &&
			lt8618_modes[i].vrefresh == drm_mode_vrefresh(mode))
		{
			return &lt8618_modes[i];
		}
	}
	return NULL;
}

static bool lt8618_check_format(u32 bus_format)
{
	switch (bus_format)
	{
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

static int lt8618_read_edid(struct lt8618 *lt8618)
{
	unsigned int temp;
	int ret = 0;
	int i, j;

	/* memset to clear old buffer, if any */
	memset(lt8618->edid_buf, 0, sizeof(lt8618->edid_buf));

	regmap_write(lt8618->regmap, LT8618_REGADDR_READ_EN, 0x01);

	regmap_write(lt8618->regmap, LT8618_REGADDR_HDMI_I2C_DDC, 0xC9);

	/* 0xA0 is EDID device address */
	regmap_write(lt8618->regmap, LT8618_REGADDR_EDID_DEV_ADDR, 0xA0);
	/* 0x00 is EDID offset address */
	regmap_write(lt8618->regmap, LT8618_REGADDR_EDID_OFFSET_ADDR, 0x00);

	/* length for read */
	regmap_write(lt8618->regmap, LT8618_REGADDR_EDID_READ_LEN, EDID_LEN);
	regmap_write(lt8618->regmap, LT8618_REGADDR_HDMI_AEC, 0x7F);

	for (i = 0; i < EDID_LOOP; i++)
	{
		/* offset address */
		regmap_write(lt8618->regmap, LT8618_REGADDR_EDID_OFFSET_ADDR, i * EDID_LEN);
		regmap_write(lt8618->regmap, LT8618_REGADDR_EDID_ACCESS_CMD, 0x36);
		regmap_write(lt8618->regmap, LT8618_REGADDR_EDID_ACCESS_CMD, 0x31);
		regmap_write(lt8618->regmap, LT8618_REGADDR_EDID_ACCESS_CMD, 0x37);
		usleep_range(5000, 10000);

		regmap_read(lt8618->regmap, LT8618_REGADDR_DDC_STA, &temp);

		if (temp & LT8618_DDC_STA_ACCS_DONE)
		{
			for (j = 0; j < EDID_LEN; j++)
			{
				regmap_read(lt8618->regmap, LT8618_REGADDR_EDID_FIFO, &temp);
				lt8618->edid_buf[i * EDID_LEN + j] = temp;
			}
		}
		else if (temp & LT8618_DDC_STA_NO_ACK)
		{ /* DDC No Ack or Abitration lost */
			dev_err(lt8618->dev, "read edid failed: no ack\n");
			ret = -EIO;
			goto end;
		}
		else
		{
			dev_err(lt8618->dev, "read edid failed: access not done\n");
			ret = -EIO;
			goto end;
		}
	}

end:
	regmap_write(lt8618->regmap, LT8618_REGADDR_EDID_ACCESS_CMD, 0x1F);
	return ret;
}

static void lt8618_bridge_pre_enable(struct drm_bridge *bridge)
{
	struct lt8618 *lt8618 = bridge_to_lt8618(bridge);
	static const struct reg_sequence reg_cfg[] = {
		{0x8102, 0x66},
		{0x810A, 0x06},
		{0x8115, 0x06},
		{0x814E, 0x00},

		{0x821B, 0x77},
		{0x821C, 0xEC},
	};

	if (!lt8618->sleep)
		return;

	regmap_multi_reg_write(lt8618->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));

	lt8618->sleep = false;
}

static int lt8618_rgb_input_digital(struct lt8618 *lt8618,
									const struct drm_display_mode *mode)
{
	// struct reg_sequence reg_cfg[] = {
	// 	{0x830a, 0x00},
	// 	{0x824f, 0x80},
	// 	{0x8250, 0x10},
	// 	{0x8302, 0x0a},
	// 	{0x8306, 0x0a},
	// };

	// if (mode->hdisplay == 3840)
	// 	reg_cfg[1].def = 0x03;

	// return regmap_multi_reg_write(lt8618->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));
}

static int lt8618_pll_setup(struct lt8618 *lt8618, const struct drm_display_mode *mode, unsigned int *postdiv)
{
	if(lt8618->chip_ver == LT8618_CHIPVER_U2){

	}
	else if(lt8618->chip_ver == LT8618_CHIPVER_U3){

	}
	else{

	}

	unsigned int pclk = mode->clock;
	const struct reg_sequence reg_cfg[] = {
		/* txpll init */
		{ 0x8123, 0x40 },
		{ 0x8124, 0x64 },
		{ 0x8125, 0x80 },
		{ 0x8126, 0x55 },
		{ 0x812c, 0x37 },
		{ 0x812f, 0x01 },
		{ 0x8126, 0x55 },
		{ 0x8127, 0x66 },
		{ 0x8128, 0x88 },
		{ 0x812a, 0x20 },
	};

	regmap_multi_reg_write(lt8618->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));

	if (pclk > 150000) {
		regmap_write(lt8618->regmap, 0x812d, 0x88);
		*postdiv = 1;
	} else if (pclk > 70000) {
		regmap_write(lt8618->regmap, 0x812d, 0x99);
		*postdiv = 2;
	} else {
		regmap_write(lt8618->regmap, 0x812d, 0xaa);
		*postdiv = 4;
	}

	/*
	 * first divide pclk by 2 first
	 *  - write divide by 64k to 19:16 bits which means shift by 17
	 *  - write divide by 256 to 15:8 bits which means shift by 9
	 *  - write remainder to 7:0 bits, which means shift by 1
	 */
	regmap_write(lt8618->regmap, 0x82e3, pclk >> 17); /* pclk[19:16] */
	regmap_write(lt8618->regmap, 0x82e4, pclk >> 9);  /* pclk[15:8]  */
	regmap_write(lt8618->regmap, 0x82e5, pclk >> 1);  /* pclk[7:0]   */

	regmap_write(lt8618->regmap, 0x82de, 0x20);
	regmap_write(lt8618->regmap, 0x82de, 0xe0);

	regmap_write(lt8618->regmap, 0x8016, 0xf1);
	regmap_write(lt8618->regmap, 0x8016, 0xf3);

	return 0;
}

////////////////   Layer 1   ////////////////////

static int lt8618_connector_init(struct drm_bridge *bridge,
								 struct lt8618 *lt8618)
{
	int ret;

	ret = drm_connector_init(bridge->dev, &lt8618->connector,
							 &lt8618_bridge_connector_funcs,
							 DRM_MODE_CONNECTOR_HDMIA);
	if (ret)
	{
		DRM_ERROR("Failed to initialize connector with drm\n");
		return ret;
	}

	drm_connector_helper_add(&lt8618->connector,
							 &lt8618_bridge_connector_helper_funcs);

	if (!bridge->encoder)
	{
		DRM_ERROR("Parent encoder object not found");
		return -ENODEV;
	}

	drm_connector_attach_encoder(&lt8618->connector, bridge->encoder);

	return 0;
}

/* bridge funcs */
static void
lt8618_bridge_atomic_enable(struct drm_bridge *bridge,
							struct drm_bridge_state *old_bridge_state)
{
	struct lt8618 *lt8618 = bridge_to_lt8618(bridge);

	if (lt8618_power_on(lt8618))
	{
		dev_err(lt8618->dev, "power on failed\n");
		return;
	}

	lt8618_mipi_input_analog(lt8618);
	lt8618_hdmi_tx_digital(lt8618);
	lt8618_hdmi_tx_phy(lt8618);

	msleep(500);

	lt8618_video_check(lt8618);

	/* Enable HDMI output */
	regmap_write(lt8618->regmap, 0x8130, 0xea);
}

static void
lt8618_bridge_atomic_disable(struct drm_bridge *bridge,
							 struct drm_bridge_state *old_bridge_state)
{
	struct lt8618 *lt8618 = bridge_to_lt8618(bridge);
	int ret;

	/* Disable HDMI output */
	ret = regmap_write(lt8618->regmap, 0x8130, 0x6a);
	if (ret)
	{
		dev_err(lt8618->dev, "video on failed\n");
		return;
	}

	if (lt8618_power_off(lt8618))
	{
		dev_err(lt8618->dev, "power on failed\n");
		return;
	}
}

static int
lt8618_get_edid_block(void *data, u8 *buf, unsigned int block, size_t len)
{
	struct lt8618 *lt8618 = data;
	int ret;

	if (len > 128)
		return -EINVAL;

	/* supports up to 1 extension block */
	/* TODO: add support for more extension blocks */
	if (block > 1)
		return -EINVAL;

	if (block == 0)
	{
		ret = lt8618_read_edid(lt8618);
		if (ret)
		{
			dev_err(lt8618->dev, "edid read failed\n");
			return ret;
		}
	}

	block %= 2;
	memcpy(buf, lt8618->edid_buf + (block * 128), len);

	return 0;
}

////////////////   Layer 2   ////////////////////
static enum drm_connector_status
lt8618_connector_detect(struct drm_connector *connector, bool force)
{
	return lt8618_detect(connector_to_lt8618(connector));
}

static int lt8618_connector_get_modes(struct drm_connector *connector)
{
	struct lt8618 *lt8618 = connector_to_lt8618(connector);
	unsigned int count;
	struct edid *edid;

	lt8618_power_on(lt8618);
	edid = drm_do_get_edid(connector, lt8618_get_edid_block, lt8618);
	drm_connector_update_edid_property(connector, edid);
	count = drm_add_edid_modes(connector, edid);
	kfree(edid);

	return count;
}

static enum drm_mode_status
lt8618_connector_mode_valid(struct drm_connector *connector,
							struct drm_display_mode *mode)
{
	struct lt8618_mode *lt8618_mode = lt8618_find_mode(mode);

	return lt8618_mode ? MODE_OK : MODE_BAD;
}

static int lt8618_bridge_attach(struct drm_bridge *bridge,
								enum drm_bridge_attach_flags flags)
{
	struct lt8618 *lt8618 = bridge_to_lt8618(bridge);
	int ret;

	if (!(flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR))
	{
		ret = lt8618_connector_init(bridge, lt8618);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static enum drm_mode_status
lt8618_bridge_mode_valid(struct drm_bridge *bridge,
						 const struct drm_display_info *info,
						 const struct drm_display_mode *mode)
{
	struct lt8618_mode *lt8618_mode = lt8618_find_mode(mode);
	struct lt8618 *lt8618 = bridge_to_lt8618(bridge);

	if (!lt8618_mode)
		return MODE_BAD;
	else
		return MODE_OK;
}

static void lt8618_bridge_mode_set(struct drm_bridge *bridge,
								   const struct drm_display_mode *mode,
								   const struct drm_display_mode *adj_mode)
{
	struct lt8618 *lt8618 = bridge_to_lt8618(bridge);
	struct hdmi_avi_infoframe avi_frame;
	unsigned int postdiv;
	int ret;

	lt8618_bridge_pre_enable(bridge);

	lt8618_rgb_input_digital(lt8618, mode);
	lt8618_pll_setup(lt8618, mode, &postdiv);
	lt8618_mipi_video_setup(lt8618, mode);
	lt8618_pcr_setup(lt8618, mode, postdiv);

	ret = drm_hdmi_avi_infoframe_from_display_mode(
		&avi_frame, &lt8618->connector, mode);
	if (!ret)
		lt8618->vic = avi_frame.video_code;
}

static void
lt8618_bridge_atomic_post_disable(struct drm_bridge *bridge,
								  struct drm_bridge_state *old_bridge_state)
{
	struct lt8618 *lt8618 = bridge_to_lt8618(bridge);

	lt8618_sleep_setup(lt8618);
}

static enum drm_connector_status lt8618_bridge_detect(struct drm_bridge *bridge)
{
	return lt8618_detect(bridge_to_lt8618(bridge));
}

static struct edid *lt8618_bridge_get_edid(struct drm_bridge *bridge,
										   struct drm_connector *connector)
{
	struct lt8618 *lt8618 = bridge_to_lt8618(bridge);

	lt8618_power_on(lt8618);
	return drm_do_get_edid(connector, lt8618_get_edid_block, lt8618);
}

static void lt8618_bridge_hpd_enable(struct drm_bridge *bridge)
{
	struct lt8618 *lt8618 = bridge_to_lt8618(bridge);

	lt8618_enable_hpd_interrupts(lt8618);
}

static u32 *
lt8618_atomic_get_input_bus_fmts(struct drm_bridge *bridge,
								 struct drm_bridge_state *bridge_state,
								 struct drm_crtc_state *crtc_state,
								 struct drm_connector_state *conn_state,
								 u32 output_fmt, unsigned int *num_input_fmts)
{
	u32 *input_fmts;

	*num_input_fmts = 0;

	input_fmts =
		kcalloc(MAX_INPUT_SEL_FORMATS, sizeof(*input_fmts), GFP_KERNEL);
	if (!input_fmts)
		return NULL;

	/* This is the DSI-end bus format */
	input_fmts[0] = MEDIA_BUS_FMT_RGB888_1X24;
	*num_input_fmts = 1;

	return input_fmts;
}

static int lt8618_parse_dt(struct device *dev, struct lt8618 *lt8618)
{
	lt8618->rgb_node = of_graph_get_remote_node(dev->of_node, 0, -1);
	if (!lt8618->rgb_node)
	{
		dev_err(lt8618->dev, "failed to get remote node for rgb\n");
		return -ENODEV;
	}

	lt8618->ac_mode = of_property_read_bool(dev->of_node, "lt,ac-mode");
	return 0;
}

static int lt8618_gpio_init(struct lt8618 *lt8618)
{
	struct device *dev = lt8618->dev;

	lt8618->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(lt8618->reset_gpio))
	{
		dev_err(dev, "failed to acquire reset gpio\n");
		return PTR_ERR(lt8618->reset_gpio);
	}

	lt8618->enable_gpio =
		devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_LOW);
	if (IS_ERR(lt8618->enable_gpio))
	{
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
	regmap_read(lt8618->regmap, LT8618_REGADDR_CHIPID1, rev + 1);
	regmap_read(lt8618->regmap, LT8618_REGADDR_CHIPID2, rev + 2);

	lt8618->chip_ver = rev[2];
	if (rev[0] != LT8618_CHIPID0 || (rev[1] != LT8618_CHIPID1))
	{
		dev_err(lt8618->dev, "chip id is incorrect\n");
		return -ENODEV;
	}

	switch (rev[2])
	{
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
	if (ret < 0)
	{
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
	if (irq_flag3 & LT8618_INT_FLAG3_HDMI_UNPLUG)
	{
		dev_info(lt8618->dev, "hdmi cable disconnected\n");

		regmap_write(lt8618->regmap, LT8618_REGADDR_INTCLR_FLAG3, 0xBF);
		regmap_write(lt8618->regmap, LT8618_REGADDR_INTCLR_FLAG3, 0x3F);
	}

	/* hpd changed high */
	if (irq_flag3 & LT8618_INT_FLAG3_HDMI_PLUG)
	{
		dev_info(lt8618->dev, "hdmi cable connected\n");

		regmap_write(lt8618->regmap, LT8618_REGADDR_INTCLR_FLAG3, 0x7F);
		regmap_write(lt8618->regmap, LT8618_REGADDR_INTCLR_FLAG3, 0x3F);
	}

	if (irq_flag3 & (LT8618_INT_FLAG3_HDMI_UNPLUG |
					 LT8618_INT_FLAG3_HDMI_PLUG) &&
		lt8618->bridge.dev)
		drm_kms_helper_hotplug_event(lt8618->bridge.dev);

	/* video input changed */
	if (irq_flag0 & LT8618_INT_FLAG0_VID_CHECK)
	{
		dev_info(lt8618->dev, "video input changed\n");
		regmap_write(lt8618->regmap, LT8618_REGADDR_SYNC_SYS_INTCLR,
					 0xFF);
		regmap_write(lt8618->regmap, LT8618_REGADDR_SYNC_SYS_INTCLR,
					 0xF7);
		regmap_write(lt8618->regmap, LT8618_REGADDR_INTCLR_FLAG0, 0xFF);
		regmap_write(lt8618->regmap, LT8618_REGADDR_INTCLR_FLAG0, 0xFE);
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

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
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
	if (IS_ERR(lt8618->regmap))
	{
		dev_err(lt8618->dev, "regmap i2c init failed\n");
		return PTR_ERR(lt8618->regmap);
	}

	ret = lt8618_parse_dt(dev, lt8618);
	if (ret)
	{
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
	if (ret)
	{
		dev_err(dev, "failed to read chip rev\n");
		goto err_disable_regulators;
	}

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
									lt8618_irq_thread_handler, IRQF_ONESHOT,
									"lt8618", lt8618);
	if (ret)
	{
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
	if (IS_ERR(lt8618->dsi0))
	{
		ret = PTR_ERR(lt8618->dsi0);
		goto err_remove_bridge;
	}

	/* Attach secondary DSI, if specified */
	if (lt8618->dsi1_node)
	{
		lt8618->dsi1 = lt8618_attach_dsi(lt8618, lt8618->dsi1_node);
		if (IS_ERR(lt8618->dsi1))
		{
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
	{}};
MODULE_DEVICE_TABLE(of, lt8618_dt_ids);

static const struct i2c_device_id lt8618_i2c_ids[] = {
	{"lt8618", 0},
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
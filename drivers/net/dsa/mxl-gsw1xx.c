// SPDX-License-Identifier: GPL-2.0
/*
 * drivers/net/dsa/mxl-gsw1xx.c - DSA Driver for MaxLinear GSW1xx switch devices
 *
 * Copyright (C) 2023 - 2024 MaxLinear Inc.
 * Copyright (C) 2022 Snap One, LLC.  All rights reserved.
 * Copyright (C) 2017 - 2019 Hauke Mehrtens <hauke@hauke-m.de>
 * Copyright (C) 2012 John Crispin <john@phrozen.org>
 * Copyright (C) 2010 Lantiq Deutschland
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * General hint about the driver:
 * The GSW VLAN Mode 2.2 of GSW1xx used by this driver has 4096 possible table
 * entries for VLANs with a single FID per VID and a list of ports for
 * each VLAN/bridge. The same VID cannot be used in multiple bridges, since
 * the isolation between bridges would not be intact anymore due to same FID.
 * The driver rejects the assignment of the same VID in multiple bridges,
 * throws an error print and ignores the port vlan add action. On vlan removal,
 * it only prints an error log and executes the action, to be able to recover
 * from this situation. In case of vlan filtering enabled and use of multiple bridges,
 * the default PVID assigned by Linux should be changed, before setting up the
 * next bridge to prevent this conflict.
 *
 * During initialization the driver disables learning on all standalone ports and
 * enables learning only in bridge mode. This ensures isolation between standalone ports
 * as well as towards bridges, since flooding of all unknown unicast/multicast/broadcast
 * frames will only happen to the CPU port which is configured as a monitoring port.
 * The CPU receives all the exception frames which do not match any forwarding
 * rule and the CPU port is also added to all bridges and VLANs.
 * Bridges will get a default PVID assigned to improve isolation between bridges and
 * standalone ports. Bridge PVIDs start with GSW1XX_BRIDGE_PVID_START and use
 * consecutive VIDs in case multiple bridges are configured. It is recommended not
 * to use VIDs in the range from GSW1XX_BRIDGE_PVID_START to GSW1XX_BRIDGE_PVID_END to
 * prevent conflicts.
 * If 8021q support is enabled in Linux kernel conf, a VID 0 will be assigned
 * to standalone ports and VID 1 to bridges. Otherwise it can also be enabled
 * in config section of the driver to assign a PVID 0 to standalone ports.
 *
 * Flooding of multicast, broadcast and unknown unicast packets will be configured
 * via a general portmap. A bridge related flooding port map is not supported.
 * Therefore it is currently not possible to offload the CPU completely from the forwarding
 * task for unknown unicast/multicast/broadcast frames.
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/if_bridge.h>
#include <linux/if_vlan.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of_device.h>
#include <linux/phy.h>
#include <net/dsa.h>
#include "mxl-gsw1xx.h"
#include "mxl-gsw1xx_pce.h"

#ifndef LINUX_VERSION_CODE
#include <linux/version.h>
#else
#define KERNEL_VERSION(a, b, c) (((a) << 16) + ((b) << 8) + (c))
#endif

struct gsw1xx_vlan {
	struct net_device *bridge;
	u16 vid;
	u8 fid;
	u8 pvid;
};

struct gsw1xx_hw_info {
	int max_ports;
	int phy_ports;
	const struct dsa_switch_ops *ops;
};

struct dts_options {
	u8 smdio_badr;
	u32 rx_mii_delay;
	u32 tx_mii_delay;
	u8 rmii_clk_custom;
	u8 auto_ldclk_disable;
	u8 rgmii_slew_rx_tx_volt;
	u8 use_sgmii;
	u8 sgmii_mode;
	u8 sgmii_auto_neg;
	u8 led_settings;
	u8 led0h_settings;
	u8 led0l_settings;
	u8 led1h_settings;
	u8 led1l_settings;
	u8 led2h_settings;
	u8 led2l_settings;
};

struct gsw1xx_priv {
	struct mii_bus *bus;
	struct device *dev;
	int sw_addr;
	const struct gsw1xx_hw_info *hw_info;
	struct dsa_switch *ds;
	struct gsw1xx_vlan vlans[4096];
	struct gsw1xx_vlan bridge_vlans[GSW1xx_PORTS];
	u32 port_vlan_filter;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0))
	/* pce_table_lock required for kernel 5.16 or later, since rtnl_lock has been dropped
	 * from DSA. port_fdb_{add,del} might cause dead-locks / hang in previous versions.
	 */
	struct mutex pce_table_lock;
#endif
	struct dts_options dts;
};

struct gsw1xx_pce_table_entry {
	u16 index;      /* PCE_TBL_ADDR.ADDR = pData->table_index */
	u16 table;      /* PCE_TBL_CTRL.ADDR = pData->table */
	u16 key[8];
	u16 val[5];
	u16 mask;
	u8 gmap;
	bool type;
	bool valid;
	bool key_mode;
};

struct gsw1xx_rmon_cnt_desc {
	unsigned int size;
	unsigned int offset;
	const char *name;
};

#define MIB_DESC(_size, _offset, _name) {.size = _size, .offset = _offset, .name = _name}

static const struct gsw1xx_rmon_cnt_desc gsw1xx_rmon_cnt[] = {
	/** Receive Packet Count (only packets that are accepted and not discarded). */
	MIB_DESC(1, 0x1F, "RxGoodPkts"),
	MIB_DESC(1, 0x23, "RxUnicastPkts"),
	MIB_DESC(1, 0x22, "RxMulticastPkts"),
	MIB_DESC(1, 0x21, "RxFCSErrorPkts"),
	MIB_DESC(1, 0x1D, "RxUnderSizeGoodPkts"),
	MIB_DESC(1, 0x1E, "RxUnderSizeErrorPkts"),
	MIB_DESC(1, 0x1B, "RxOversizeGoodPkts"),
	MIB_DESC(1, 0x1C, "RxOversizeErrorPkts"),
	MIB_DESC(1, 0x20, "RxGoodPausePkts"),
	MIB_DESC(1, 0x1A, "RxAlignErrorPkts"),
	MIB_DESC(1, 0x12, "Rx64BytePkts"),
	MIB_DESC(1, 0x13, "Rx127BytePkts"),
	MIB_DESC(1, 0x14, "Rx255BytePkts"),
	MIB_DESC(1, 0x15, "Rx511BytePkts"),
	MIB_DESC(1, 0x16, "Rx1023BytePkts"),
	/** Receive Size 1024-1522 (or more, if configured) Packet Count. */
	MIB_DESC(1, 0x17, "RxMaxBytePkts"),
	MIB_DESC(1, 0x18, "RxDroppedPkts"),
	MIB_DESC(1, 0x19, "RxFilteredPkts"),
	MIB_DESC(2, 0x24, "RxGoodBytes"),
	MIB_DESC(2, 0x26, "RxBadBytes"),
	MIB_DESC(1, 0x11, "TxAcmDroppedPkts"),
	MIB_DESC(1, 0x0C, "TxGoodPkts"),
	MIB_DESC(1, 0x06, "TxUnicastPkts"),
	MIB_DESC(1, 0x07, "TxMulticastPkts"),
	MIB_DESC(1, 0x00, "Tx64BytePkts"),
	MIB_DESC(1, 0x01, "Tx127BytePkts"),
	MIB_DESC(1, 0x02, "Tx255BytePkts"),
	MIB_DESC(1, 0x03, "Tx511BytePkts"),
	MIB_DESC(1, 0x04, "Tx1023BytePkts"),
	/** Transmit Size 1024-1522 (or more, if configured) Packet Count. */
	MIB_DESC(1, 0x05, "TxMaxBytePkts"),
	MIB_DESC(1, 0x08, "TxSingleCollCount"),
	MIB_DESC(1, 0x09, "TxMultCollCount"),
	MIB_DESC(1, 0x0A, "TxLateCollCount"),
	MIB_DESC(1, 0x0B, "TxExcessCollCount"),
	MIB_DESC(1, 0x0D, "TxPauseCount"),
	MIB_DESC(1, 0x10, "TxDroppedPkts"),
	MIB_DESC(2, 0x0E, "TxGoodBytes"),
};

static int get_cpu_port(struct gsw1xx_priv *priv)
{
	if (priv->dts.use_sgmii == GSW1XX_SGMII_IS_CPU_PORT)
		return 4;
	return 5;
}

static u32 gsw1xx_read_reg(struct gsw1xx_priv *priv, u32 offset)
{
	struct mii_bus *bus = priv->bus;
	int sw_addr = priv->sw_addr;
	int res;

	/* lock mdio bus */
	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	/* Configure the Switch Base Address */
	res = __mdiobus_write(bus, sw_addr, priv->dts.smdio_badr, offset);
	if (res < 0) {
		dev_err(priv->dev, "%s: Error %d, configuring switch base\n", __func__, res);
		goto error;
	}

	res = __mdiobus_read(bus, sw_addr, 0);
	if (res < 0) {
		dev_err(priv->dev, "%s: Error %d reading 0x%x\n", __func__, res,
				offset);
	}

error:
	/* unlock mdio bus */
	mutex_unlock(&bus->mdio_lock);
	return res;
}

static int gsw1xx_write_reg(struct gsw1xx_priv *priv, u32
			    offset, u32 val)
{
	struct mii_bus *bus = priv->bus;
	int sw_addr = priv->sw_addr;
	int res;

	/* lock mdio bus */
	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	/* Configure the Switch Base Address */
	res = __mdiobus_write(bus, sw_addr, priv->dts.smdio_badr, offset);
	if (res < 0) {
		dev_err(priv->dev, "%s: Error %d, configuring switch base\n", __func__, res);
		goto error;
	}

	res = __mdiobus_write(bus, sw_addr, 0, val);
	if (res < 0) {
		dev_err(priv->dev, "%s: Error %d, writing 0x%x:0x%x\n", __func__, res,
				offset, val);
		goto error;
	}

error:
	/* unlock mdio bus */
	mutex_unlock(&bus->mdio_lock);
	return res;
}

static void gsw1xx_modify_reg(struct gsw1xx_priv *priv, u32 clear, u32 set,
			      u32 offset)
{
	u32 val = gsw1xx_read_reg(priv, offset);

	val &= ~(clear);
	val |= set;
	gsw1xx_write_reg(priv, offset, val);
}

static void gsw1xx_modify_mii_cfg_reg(struct gsw1xx_priv *priv, u32 clear,
				      u32 set, int port)
{
	/* There's only an MII_CFG register for the RGMII / RMII */
	if (port == GSW1XX_MII_PORT)
		gsw1xx_modify_reg(priv, clear, set, RGMII_BASE + GSW1XX_MII_CFG);
}

static void gsw1xx_modify_pcdu_reg(struct gsw1xx_priv *priv, u32 clear,
				   u32 set, int port)
{
	switch (port) {
	case 5:
		gsw1xx_modify_reg(priv, clear, set, RGMII_BASE+MII_PCDU5_OFFS);
		break;
	default:
		dev_err(priv->dev, "%s: Error port %d is internal - no xmii\n", __func__, port);
		break;
	}
}

static u32 gsw1xx_read_switch_reg(struct gsw1xx_priv *priv, u32 offset)
{
	return gsw1xx_read_reg(priv, SWITCH_BASE + offset);
}

static int gsw1xx_switch_write(struct gsw1xx_priv *priv, u32 val,
			       u32 offset)
{
	return gsw1xx_write_reg(priv, SWITCH_BASE+offset, val);
}

static int gsw1xx_modify_switch_reg(struct gsw1xx_priv *priv, u32 clear,
				    u32 set, u32 offset)
{
	u32 val = gsw1xx_read_switch_reg(priv, offset);

	val &= ~(clear);
	val |= set;

	return gsw1xx_switch_write(priv, val, offset);
}

static int gsw1xx_read_switch_reg_timeout(struct gsw1xx_priv *priv, u32 offset,
					  u32 cleared)
{
	struct mii_bus *bus = priv->bus;
	int sw_addr = priv->sw_addr;
	u32 timeout = 100;
	int res;

	/* lock mdio bus */
	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	/* Configure the Switch Base Address */
	res = __mdiobus_write(bus, sw_addr, priv->dts.smdio_badr, SWITCH_BASE + offset);
	if (res < 0) {
		dev_err(priv->dev, "%s: Error %d, configuring switch base\n", __func__, res);
		goto error;
	}

	/* Poll register until bit 'cleared' is '0' */
	do {
		res = __mdiobus_read(bus, sw_addr, 0);
		if (res >= 0 && ((res & cleared) == 0)) {
			/* bit 'cleared' is '0' */
			res = 0;
			goto error;
		}
		mdelay(1);
	} while (--timeout);

	dev_err(priv->dev, "%s: Timeout Error on Switch Read\n", __func__);

	res = -ETIMEDOUT;

error:
	/* unlock mdio bus */
	mutex_unlock(&bus->mdio_lock);
	return res;
}

/* Wait for the current MMDIO indirect command to complete */
static int gsw1xx_wait_for_mmdio_ready(struct gsw1xx_priv *priv, struct mii_bus *bus, int addr)
{
	int val;
	u32 timeout = 100;

	/* check if MDIO operation has been completed and the bus is no longer busy */
	while (likely(timeout--)) {
		val = __mdiobus_read(bus, addr, GSW1XX_MDIO_CTRL);
		/* check if no error occurred -> val >=0 and check for busy bit cleared */
		if (val >= 0 && ((val & GSW1XX_MDIO_CTRL_BUSY) == 0))
			return 0;
		usleep_range(20, 40);
	}

	dev_err(priv->dev, "GSW1xx SMI busy timeout\n");

	return -ETIMEDOUT;
}

/**
 *  gsw1xx_read_mdio_master_switch_reg()
 *  Read MDIO Master register of switch PDI toplevel registers.
 *  not usuable for mmdio register access
 */
static u32 gsw1xx_read_mdio_master_switch_reg(struct gsw1xx_priv *priv, int offset)
{
	struct mii_bus *bus = priv->bus;
	int sw_addr = priv->sw_addr;
	int res;

	/* lock mdio bus */
	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	/* Configure the Base Address */
	res = __mdiobus_write(bus, sw_addr, priv->dts.smdio_badr, MMDIO_BASE);
	if (res < 0) {
		dev_err(priv->dev, "Error setting mmdio base address\n");
		goto error;
	}

	/* Read the data */
	res = __mdiobus_read(bus, sw_addr, offset);
	if (res < 0)
		goto error;

error:
	/* unlock mdio bus */
	mutex_unlock(&bus->mdio_lock);
	return res;
}

/**
 *  gsw1xx_write_mdio_master_switch_reg()
 *  Read MDIO Master register of switch PDI toplevel registers.
 *  Not for mmdio register access.
 */
static int gsw1xx_write_mdio_master_switch_reg(struct gsw1xx_priv *priv, u32 offset, u32 val)
{
	struct mii_bus *bus = priv->bus;
	int sw_addr = priv->sw_addr;
	int res;

	/* lock mdio bus */
	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	/* Configure the Base Address */
	res = __mdiobus_write(bus, sw_addr, priv->dts.smdio_badr, MMDIO_BASE);
	if (res < 0) {
		dev_err(priv->dev, "%s: Error setting mmdio base address\n", __func__);
		goto error;
	}

	/* Write the data */
	res = __mdiobus_write(bus, sw_addr, offset, val);
	if (res < 0)
		goto error;

error:
	/* unlock mdio bus */
	mutex_unlock(&bus->mdio_lock);
	return res;
}

/**
 *  gsw1xx_modify_mdio_master_switch_reg()
 *  Modify MDIO Master register of switch PDI toplevel registers.
 *  Not for mmdio register access.
 */
static void gsw1xx_modify_mdio_master_switch_reg(struct gsw1xx_priv *priv, u32 clear,
		u32 set, u32 offset)
{
	u32 val = gsw1xx_read_mdio_master_switch_reg(priv, offset);

	val &= ~(clear);
	val |= set;

	gsw1xx_write_mdio_master_switch_reg(priv, offset, val);
}

/**
 *  gsw1xx_mmd_write()
 *  write access to MMD register of PHYs
 *	-> only required for optional LED configuration of internal GPY
 */
static int gsw1xx_mmd_write(struct gsw1xx_priv *priv, int port, int dev,
			    int reg, u16 val)
{
	struct mii_bus *bus = priv->bus;
	int sw_addr = priv->sw_addr;
	int res;

	/* lock mdio bus */
	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	/* Configure the Base Address */
	res = __mdiobus_write(bus, sw_addr, priv->dts.smdio_badr, MMDIO_BASE);
	if (res < 0)
		goto error;

	/* Wait for the bus to become free */
	res = gsw1xx_wait_for_mmdio_ready(priv, bus, sw_addr);
	if (res < 0)
		goto error;

	/* Set the DevID for Write Command */
	res = __mdiobus_write(bus, sw_addr, GSW1XX_MDIO_WRITE, dev);
	if (res < 0)
		goto error;

	/* Issue the write command */
	res = __mdiobus_write(bus, sw_addr, GSW1XX_MDIO_CTRL, OP_CODE_WR |
			      PHY_ADDR(port) | GSW1XX_MMDCTRL);
	if (res < 0)
		goto error;

	/* Wait for the bus to become free */
	res = gsw1xx_wait_for_mmdio_ready(priv, bus, sw_addr);
	if (res < 0)
		goto error;

	/* Set the MMD reg Addr for Write Command */
	res = __mdiobus_write(bus, sw_addr, GSW1XX_MDIO_WRITE, (reg & 0xffff));
	if (res < 0)
		goto error;

	/* Issue the write command */
	res = __mdiobus_write(bus, sw_addr, GSW1XX_MDIO_CTRL, OP_CODE_WR |
			      PHY_ADDR(port) | GSW1XX_MMDDATA);
	if (res < 0)
		goto error;

	/* Wait for the bus to become free */
	res = gsw1xx_wait_for_mmdio_ready(priv, bus, sw_addr);
	if (res < 0)
		goto error;

	/* Set ((0x4000) | dev) for Write Command */
	res = __mdiobus_write(bus, sw_addr, GSW1XX_MDIO_WRITE, ((0x4000) | dev));
	if (res < 0)
		goto error;

	/* Issue the write command */
	res = __mdiobus_write(bus, sw_addr, GSW1XX_MDIO_CTRL, OP_CODE_WR |
			      PHY_ADDR(port) | GSW1XX_MMDCTRL);
	if (res < 0)
		goto error;

	/* Wait for the bus to become free */
	res = gsw1xx_wait_for_mmdio_ready(priv, bus, sw_addr);
	if (res < 0)
		goto error;

	/* Set the Data for Write Command */
	res = __mdiobus_write(bus, sw_addr, GSW1XX_MDIO_WRITE, (val & 0xffff));
	if (res < 0)
		goto error;

	/* Issue the write command */
	res = __mdiobus_write(bus, sw_addr, GSW1XX_MDIO_CTRL, OP_CODE_WR |
			      PHY_ADDR(port) | GSW1XX_MMDDATA);
	if (res < 0)
		goto error;

	/* Wait for the Command to complete */
	res = gsw1xx_wait_for_mmdio_ready(priv, bus, sw_addr);
	if (res < 0)
		goto error;

error:
	/* unlock mdio bus */
	mutex_unlock(&bus->mdio_lock);

	return res;
}

/**
 *  gsw1xx_mmdio_read()
 *  read access to MMD register of PHYs via MMDIO
 */
static int gsw1xx_mmdio_read(struct gsw1xx_priv *priv, int port,
			     int reg)
{
	struct mii_bus *bus = priv->bus;
	int sw_addr = priv->sw_addr;
	int res;

	/* lock mdio bus */
	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	/* Configure the Base Address */
	res = __mdiobus_write(bus, sw_addr, priv->dts.smdio_badr, MMDIO_BASE);
	if (res < 0) {
		dev_err(priv->dev, "Error setting mmdio base address\n");
		goto error;
	}

	/* Wait for the bus to become free */
	res = gsw1xx_wait_for_mmdio_ready(priv, bus, sw_addr);
	if (res < 0)
		goto error;

	/* Write the MMDIO Ctrl Command */
	res = __mdiobus_write(bus, sw_addr, GSW1XX_MDIO_CTRL, OP_CODE_RD |
			      PHY_ADDR(port) | (reg & PHY_REG_MASK));
	if (res < 0)
		goto error;

	/* Wait for the bus to become free */
	res = gsw1xx_wait_for_mmdio_ready(priv, bus, sw_addr);
	if (res < 0)
		goto error;

	/* Read the data */
	res = __mdiobus_read(bus, sw_addr, GSW1XX_MDIO_READ);
	if (res < 0)
		goto error;

error:
	/* unlock mdio bus */
	mutex_unlock(&bus->mdio_lock);
	return res;
}

/**
 *  gsw1xx_mmdio_write()
 *  write access to MDIO register of PHYs
 */
static int gsw1xx_mmdio_write(struct gsw1xx_priv *priv, int port,
			      int reg, u16 val)
{
	struct mii_bus *bus = priv->bus;
	int sw_addr = priv->sw_addr;
	int res;

	/* lock mdio bus */
	mutex_lock_nested(&bus->mdio_lock, MDIO_MUTEX_NESTED);

	/* Configure the Base Address */
	res = __mdiobus_write(bus, sw_addr, priv->dts.smdio_badr, MMDIO_BASE);
	if (res < 0)
		goto error;

	/* Wait for the bus to become free */
	res = gsw1xx_wait_for_mmdio_ready(priv, bus, sw_addr);
	if (res < 0)
		goto error;

	/* Set the Data for Write Command */
	res = __mdiobus_write(bus, sw_addr, GSW1XX_MDIO_WRITE, val);
	if (res < 0)
		goto error;

	/* Issue the write command */
	res = __mdiobus_write(bus, sw_addr, GSW1XX_MDIO_CTRL, OP_CODE_WR |
			      PHY_ADDR(port) | (reg & PHY_REG_MASK));
	if (res < 0)
		goto error;

	/* Wait for the Command to complete */
	res = gsw1xx_wait_for_mmdio_ready(priv, bus, sw_addr);
	if (res < 0)
		goto error;

error:
	/* unlock mdio bus */
	mutex_unlock(&bus->mdio_lock);
	return res;
}

static int gsw1xx_phy_read(struct dsa_switch *ds, int port, int reg)
{
	return gsw1xx_mmdio_read(ds->priv, port, reg);
}

static int gsw1xx_phy_write(struct dsa_switch *ds, int port,
			    int reg, u16 data)
{
	return gsw1xx_mmdio_write(ds->priv, port, reg, data);
}

#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 5, 0))
static enum dsa_tag_protocol gsw1xx_get_tag_protocol(struct dsa_switch *ds,
						     int port,
						     enum dsa_tag_protocol m)
#else
static enum dsa_tag_protocol gsw1xx_get_tag_protocol(struct dsa_switch *ds,
						     int port)
#endif
{
	return DSA_TAG_PROTO_MXL_GSW1XX;
}

static int gsw1xx_pce_table_entry_read(struct gsw1xx_priv *priv,
				       struct gsw1xx_pce_table_entry *tbl)
{
	int i;
	int err;
	u16 crtl;
	u16 addr_mode = tbl->key_mode ? GSW1XX_PCE_TBL_CTRL_OPMOD_KSRD :
					GSW1XX_PCE_TBL_CTRL_OPMOD_ADRD;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0))
	mutex_lock(&priv->pce_table_lock);
#endif

	err = gsw1xx_read_switch_reg_timeout(priv, GSW1XX_PCE_TBL_CTRL,
					     GSW1XX_PCE_TBL_CTRL_BAS);
	if (err) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0))
		mutex_unlock(&priv->pce_table_lock);
#endif
		return err;
	}

	gsw1xx_switch_write(priv, tbl->index, GSW1XX_PCE_TBL_ADDR);
	gsw1xx_modify_switch_reg(priv, GSW1XX_PCE_TBL_CTRL_ADDR_MASK |
				 GSW1XX_PCE_TBL_CTRL_OPMOD_MASK,
				 tbl->table | addr_mode | GSW1XX_PCE_TBL_CTRL_BAS,
				 GSW1XX_PCE_TBL_CTRL);

	err = gsw1xx_read_switch_reg_timeout(priv, GSW1XX_PCE_TBL_CTRL,
					     GSW1XX_PCE_TBL_CTRL_BAS);
	if (err) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0))
		mutex_unlock(&priv->pce_table_lock);
#endif
		return err;
	}

	for (i = 0; i < ARRAY_SIZE(tbl->key); i++)
		tbl->key[i] = gsw1xx_read_switch_reg(priv, GSW1XX_PCE_TBL_KEY(i));

	for (i = 0; i < ARRAY_SIZE(tbl->val); i++)
		tbl->val[i] = gsw1xx_read_switch_reg(priv, GSW1XX_PCE_TBL_VAL(i));

	tbl->mask = gsw1xx_read_switch_reg(priv, GSW1XX_PCE_TBL_MASK);

	crtl = gsw1xx_read_switch_reg(priv, GSW1XX_PCE_TBL_CTRL);

	tbl->type = !!(crtl & GSW1XX_PCE_TBL_CTRL_TYPE);
	tbl->valid = !!(crtl & GSW1XX_PCE_TBL_CTRL_VLD);
	tbl->gmap = (crtl & GSW1XX_PCE_TBL_CTRL_GMAP_MASK) >> 7;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0))
	mutex_unlock(&priv->pce_table_lock);
#endif

	return 0;
}

static int gsw1xx_pce_table_entry_write(struct gsw1xx_priv *priv,
					struct gsw1xx_pce_table_entry *tbl)
{
	int i;
	int err;
	u16 crtl;
	u16 addr_mode = tbl->key_mode ? GSW1XX_PCE_TBL_CTRL_OPMOD_KSWR :
					GSW1XX_PCE_TBL_CTRL_OPMOD_ADWR;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0))
	mutex_lock(&priv->pce_table_lock);
#endif

	err = gsw1xx_read_switch_reg_timeout(priv, GSW1XX_PCE_TBL_CTRL,
					     GSW1XX_PCE_TBL_CTRL_BAS);
	if (err) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0))
		mutex_unlock(&priv->pce_table_lock);
#endif
		return err;
	}

	gsw1xx_switch_write(priv, tbl->index, GSW1XX_PCE_TBL_ADDR);
	gsw1xx_modify_switch_reg(priv, GSW1XX_PCE_TBL_CTRL_ADDR_MASK |
				 GSW1XX_PCE_TBL_CTRL_OPMOD_MASK,
				 tbl->table | addr_mode,
				 GSW1XX_PCE_TBL_CTRL);

	for (i = 0; i < ARRAY_SIZE(tbl->key); i++)
		gsw1xx_switch_write(priv, tbl->key[i], GSW1XX_PCE_TBL_KEY(i));

	for (i = 0; i < ARRAY_SIZE(tbl->val); i++)
		gsw1xx_switch_write(priv, tbl->val[i], GSW1XX_PCE_TBL_VAL(i));

	gsw1xx_modify_switch_reg(priv, GSW1XX_PCE_TBL_CTRL_ADDR_MASK |
				 GSW1XX_PCE_TBL_CTRL_OPMOD_MASK,
				 tbl->table | addr_mode,
				 GSW1XX_PCE_TBL_CTRL);

	gsw1xx_switch_write(priv, tbl->mask, GSW1XX_PCE_TBL_MASK);

	crtl = gsw1xx_read_switch_reg(priv, GSW1XX_PCE_TBL_CTRL);
	crtl &= ~(GSW1XX_PCE_TBL_CTRL_TYPE | GSW1XX_PCE_TBL_CTRL_VLD |
		  GSW1XX_PCE_TBL_CTRL_GMAP_MASK);
	if (tbl->type)
		crtl |= GSW1XX_PCE_TBL_CTRL_TYPE;
	if (tbl->valid)
		crtl |= GSW1XX_PCE_TBL_CTRL_VLD;
	crtl |= (tbl->gmap << 7) & GSW1XX_PCE_TBL_CTRL_GMAP_MASK;
	crtl |= GSW1XX_PCE_TBL_CTRL_BAS;
	gsw1xx_switch_write(priv, crtl, GSW1XX_PCE_TBL_CTRL);

	err = gsw1xx_read_switch_reg_timeout(priv, GSW1XX_PCE_TBL_CTRL,
					     GSW1XX_PCE_TBL_CTRL_BAS);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0))
	mutex_unlock(&priv->pce_table_lock);
#endif

	return err;
}

static int gsw1xx_is_valid_non_cpu_port(struct gsw1xx_priv *priv, int port)
{
	int valid = 0;

	if (port < priv->hw_info->phy_ports)
		valid = 1;
	if (priv->dts.use_sgmii == GSW1XX_SGMII_WITH_EXTERNAL_PHY &&
	    port == GSW1XX_SGMII_PORT)
		valid = 1;
	if (priv->dts.use_sgmii == GSW1XX_SGMII_IS_CPU_PORT &&
	    port == GSW1XX_MII_PORT)
		valid = 1;

	return valid;
}

static int gsw1xx_get_fid(struct gsw1xx_priv *priv, struct net_device *bridge)
{
	bool used[GSW1xx_PORTS + 1] = {false};
	int idx = -1;
	int i;

	/* Search for bridge fid */
	for (i = 0; i < GSW1xx_PORTS; i++) {
		if (priv->bridge_vlans[i].bridge == bridge) {
			if (priv->bridge_vlans[i].fid != 0) {
				idx = i;
				break;
			}
		}
	}

	/* Find first unused fid in case no fid yet defined for this bridge */
	if (idx == -1) {
		for (i = 0; i < GSW1xx_PORTS; i++) {
			if (priv->bridge_vlans[i].fid > 0) {
				used[priv->bridge_vlans[i].fid] = true;
				break;
			}
		}
		for (i = 1; i <= GSW1xx_PORTS; i++) {
			if (!used[i]) {
				idx = i;
				break;
			}
		}
	} else {
		idx = priv->bridge_vlans[idx].fid;
	}

	/* Standalone ports use FID 0 */
	if (!bridge)
		idx = 0;

	if (idx == -1) {
		dev_err(priv->dev, "%s No free FID found!\n", __func__);
		return 0;
	}

	return idx;
}

static int gsw1xx_vlan_add_unaware(struct dsa_switch *ds, int port, struct net_device *bridge)
{
	struct gsw1xx_priv *priv = ds->priv;
	struct gsw1xx_pce_table_entry vlan_mapping = {0,};
	unsigned int max_ports = priv->hw_info->max_ports;
	unsigned int cpu_port = get_cpu_port(priv);
	unsigned int port_bitmap = BIT(cpu_port);

	int i;
	int err = 0;
	int idx = -1;

	for (i = 0; i < max_ports; i++) {
		/* Add this port to the port matrix of the other ports in the
		 * same bridge.
		 */
		if (dsa_is_user_port(ds, i) && i != port) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
			if (dsa_to_port(ds, i)->bridge_dev != bridge)
#else
			if (dsa_port_bridge_dev_get(dsa_to_port(ds, i)) != bridge)
#endif
				continue;
			port_bitmap |= BIT(i);
		}
	}

	for (i = GSW1XX_BRIDGE_PVID_START; i < GSW1XX_BRIDGE_PVID_END; i++) {
		if (port_bitmap == BIT(cpu_port)) {
			/* Search for free PVID for isolation */
			if (!priv->vlans[i].bridge) {
				priv->vlans[i].bridge = bridge;
				priv->vlans[i].fid = gsw1xx_get_fid(priv, bridge);
				idx = i;
				break;
			}
		} else {
			/* find the existing and matching bridge */
			if (priv->vlans[i].bridge == bridge)
				idx = i;
		}
	}

	if (idx == -1) {
		dev_err(priv->dev, "no free bridge PVID for bridge on port %d! %d\n", port, err);
		return -ENOSPC;
	}

	port_bitmap |= BIT(port);
	vlan_mapping.index = idx;
	vlan_mapping.table = GSW1XX_TABLE_VLAN_MAPPING;
	vlan_mapping.val[0] = priv->vlans[idx].fid;
	vlan_mapping.val[1] = port_bitmap;
	err = gsw1xx_pce_table_entry_write(priv, &vlan_mapping);
	if (err) {
		dev_err(priv->dev, "failed to write VLAN mapping: %d\n", err);
		return err;
	}
	gsw1xx_switch_write(priv, idx, GSW1XX_PCE_DEFPVID(port));

	/* update bridge_vlan information */
	priv->bridge_vlans[port].bridge = bridge;
	priv->bridge_vlans[port].vid = idx;
	priv->bridge_vlans[port].fid = priv->vlans[idx].fid;

	return 0;
}

static int gsw1xx_vlan_del_unaware(struct dsa_switch *ds, int port, struct net_device *bridge)
{
	struct gsw1xx_priv *priv = ds->priv;
	struct gsw1xx_pce_table_entry vlan_mapping = {0,};
	unsigned int cpu_port = get_cpu_port(priv);
	int i;
	int err;
	int idx = -1;

	/* Search for PVID for isolation */
	for (i = GSW1XX_BRIDGE_PVID_START; i < GSW1XX_BRIDGE_PVID_END; i++) {
		if (priv->vlans[i].bridge == bridge) {
			idx = i;
			break;
		}
	}
	/* Read the existing VLAN mapping entry from the switch */
	vlan_mapping.index = idx;
	vlan_mapping.table = GSW1XX_TABLE_VLAN_MAPPING;
	err = gsw1xx_pce_table_entry_read(priv, &vlan_mapping);
	if (err) {
		dev_err(priv->dev, "failed to read VLAN mapping: %d\n",
			err);
		return err;
	}
	vlan_mapping.val[0] = priv->vlans[idx].fid;
	vlan_mapping.val[1] &= ~BIT(port);
	vlan_mapping.val[2] = 0;
	/* Clear CPU port from bridge, when bridge is empty */
	if ((vlan_mapping.val[1] & ~BIT(cpu_port)) == 0) {
		vlan_mapping.val[0] = 0;
		vlan_mapping.val[1] = 0;
		priv->vlans[i].bridge = NULL;
		priv->vlans[i].fid = 0;
	}
	err = gsw1xx_pce_table_entry_write(priv, &vlan_mapping);
	if (err)
		dev_err(priv->dev, "failed to write VLAN mapping: %d\n", err);

	/* update bridge_vlan information */
	priv->bridge_vlans[port].bridge = NULL;
	priv->bridge_vlans[port].vid = 0;
	priv->bridge_vlans[port].fid = 0;

	return err;
}

static void gsw1xx_port_set_learning(struct gsw1xx_priv *priv, int port,
				     bool enable)
{
	/* learning disable bit */
	gsw1xx_modify_switch_reg(priv, GSW1XX_PCE_PCTRL_3_LNDIS,
				 enable ? 0 : GSW1XX_PCE_PCTRL_3_LNDIS,
				 GSW1XX_PCE_PCTRL_3p(port));
}

static int gsw1xx_vlan_add_aware(struct gsw1xx_priv *priv,
				struct net_device *bridge, int port,
				u16 vid, bool untagged,
				bool pvid)
{
	struct gsw1xx_pce_table_entry vlan_mapping = {0,};
	unsigned int cpu_port = get_cpu_port(priv);
	int idx = -1;
	int fid = -1;
	int err;

	if (!bridge) {
		/* searching for a VID entry for a bridge */
		if (!priv->vlans[vid].bridge) {
			if (priv->vlans[vid].bridge == bridge) {
				/* a bridge for this VID already exists and can be updated */
				idx = vid;
				/* only 8 of 12 bits of vid used as fid */
				fid = gsw1xx_get_fid(priv, bridge);
				/* update bridge_vlan information */
				priv->bridge_vlans[port].bridge = bridge;
				priv->bridge_vlans[port].vid = vid;
				priv->bridge_vlans[port].fid = fid;
				if (pvid)
					priv->bridge_vlans[port].pvid = vid;
				else
					if (priv->bridge_vlans[port].pvid == vid)
						priv->bridge_vlans[port].pvid = 0;
			} else {
				/* another bridge uses this VID entry already*/
				dev_err(priv->dev, "%s: VLAN Add Aware bridge: another bridge on this vid: %d, config ignored!!!\n", __func__, vid);
				return -ENOSPC;
			}
		}
	} else {
		/* searching for a VID entry for a standalone port */
		if (priv->vlans[vid].bridge != NULL) {
			/* a bridge uses this VID entry already*/
			dev_err(priv->dev, "%s: VLAN Add Aware standalone: a bridge is already on this vid: %d, config ignored!!!\n", __func__, vid);
			return -ENOSPC;
		} else {
			/* VID entry is not used by a bridge, but could be used by another
			 * standalone port. We would use it for this port, too.
			 */
			idx = vid;
			fid = gsw1xx_get_fid(priv, bridge);
			/* update bridge_vlan information */
			priv->bridge_vlans[port].bridge = NULL;
			priv->bridge_vlans[port].vid = vid;
			priv->bridge_vlans[port].fid = 0;
			priv->bridge_vlans[port].pvid = 0;
		}
	}

	/* If this bridge is not programmed yet, create VLAN mapping table
	 * entry.
	 */
	if (idx == -1) {
		priv->vlans[vid].bridge = bridge;
		/* only 8 of 12 bits of vid used as fid */
		fid = gsw1xx_get_fid(priv, bridge);
		priv->vlans[vid].fid = fid;

		vlan_mapping.index = vid;
		vlan_mapping.table = GSW1XX_TABLE_VLAN_MAPPING;
		/* VLAN ID byte, maps to the VLAN ID of vlan active table */
		vlan_mapping.val[0] = fid;
		/* update bridge_vlan information */
		priv->bridge_vlans[port].bridge = bridge;
		priv->bridge_vlans[port].vid = vid;
		priv->bridge_vlans[port].fid = fid;
		if (pvid)
			priv->bridge_vlans[port].pvid = vid;
	} else {
		/* Read the existing VLAN mapping entry from the switch */
		vlan_mapping.index = vid;
		vlan_mapping.table = GSW1XX_TABLE_VLAN_MAPPING;
		err = gsw1xx_pce_table_entry_read(priv, &vlan_mapping);
		if (err) {
			dev_err(priv->dev, "failed to read VLAN mapping: %d\n",
				err);
			return err;
		}
	}

	/* Update the VLAN mapping entry and write it to the switch */
	vlan_mapping.val[1] |= BIT(cpu_port);
	vlan_mapping.val[1] |= BIT(port);
	if (untagged) {
		vlan_mapping.val[2] &= ~BIT(port);
		vlan_mapping.val[2] &= ~BIT(cpu_port);
	} else {
		vlan_mapping.val[2] |= BIT(port);
		vlan_mapping.val[2] |= BIT(cpu_port);
	}
	err = gsw1xx_pce_table_entry_write(priv, &vlan_mapping);
	if (err) {
		dev_err(priv->dev, "failed to write VLAN mapping: %d\n", err);
		return err;
	}

	if (pvid) {
		/* Accept all frames with valid VLAN ID tag and untagged frames only if PVID
		 * is enabled.
		 */
		if (dsa_port_is_vlan_filtering(dsa_to_port(priv->ds, port))) {
			gsw1xx_modify_switch_reg(priv,
						 GSW1XX_PCE_VCTRL_VINR_MASK,
						 GSW1XX_PCE_VCTRL_VINR_ALL,
						 GSW1XX_PCE_VCTRL(port));
		}

		/* Configure PVID to Vid */
		gsw1xx_switch_write(priv, vid, GSW1XX_PCE_DEFPVID(port));

	} else if (priv->bridge_vlans[port].pvid == vid) {
		/* The PVID VLAN is overwritten with non-PVID VLAN */
		priv->bridge_vlans[port].pvid = 0;
		gsw1xx_switch_write(priv, 0, GSW1XX_PCE_DEFPVID(port));

		/* In case of non PVID ports only tagged frames are allowed. */
		if (dsa_port_is_vlan_filtering(dsa_to_port(priv->ds, port))) {
			gsw1xx_modify_switch_reg(priv,
						 GSW1XX_PCE_VCTRL_VINR_MASK,
						 GSW1XX_PCE_VCTRL_VINR_VTAG,
						 GSW1XX_PCE_VCTRL(port));
		}

	} else {
		/* In case of non PVID ports only tagged frames are allowed. */
		if (dsa_port_is_vlan_filtering(dsa_to_port(priv->ds, port))) {
			if (priv->bridge_vlans[port].pvid == 0)
				gsw1xx_modify_switch_reg(priv,
							 GSW1XX_PCE_VCTRL_VINR_MASK,
							 GSW1XX_PCE_VCTRL_VINR_VTAG,
							 GSW1XX_PCE_VCTRL(port));
			}
	}
	return 0;
}

static int gsw1xx_vlan_remove(struct gsw1xx_priv *priv,
			      struct net_device *bridge, int port,
			      u16 vid, bool pvid, bool vlan_aware)
{
	struct gsw1xx_pce_table_entry vlan_mapping = {0,};
	unsigned int cpu_port = get_cpu_port(priv);
	int err;

	if (!bridge) {
		/* searching for a VID entry for a bridge */
		if (!priv->vlans[vid].bridge) {
			if (priv->vlans[vid].bridge != bridge) {
				/* another bridge uses this VID entry already - treat as warning
				 * only in case of removal.
				 */
				dev_err(priv->dev, "%s: VLAN Remove: another bridge on this vid: %d!\n", __func__, vid);
			}
		} else {
			/* special handling for 8021q VLAN 0 applied to standalone ports */
			if (vid != 0) {
				/* no bridge is assigned to this VID entry. This can be a valid
				 * case when VID was changed from default 1 to another VID.
				 * Therefore removing the old entry will not be blocked.
				 */
				dev_warn(priv->dev, "%s: VLAN Remove: no bridge found on this vid: %d, only 1 VID per bridge allowed\n",
					 __func__, vid);
			}
		}
	} else {
		/* searching for a VID entry of a standalone port */
		if (!priv->vlans[vid].bridge) {
			/* a bridge uses this VID entry */
			dev_err(priv->dev, "%s: VLAN Remove standalone: a bridge is on this vid: %d!\n",
				__func__, vid);
			return -ENOENT;
		}
	}

	vlan_mapping.index = vid;
	vlan_mapping.table = GSW1XX_TABLE_VLAN_MAPPING;
	err = gsw1xx_pce_table_entry_read(priv, &vlan_mapping);
	if (err) {
		dev_err(priv->dev, "failed to read VLAN mapping: %d\n",	err);
		return err;
	}
	vlan_mapping.val[1] &= ~BIT(port);
	vlan_mapping.val[2] &= ~BIT(port);

	/* In case all ports are removed from the bridge, remove the VLAN */
	if ((vlan_mapping.val[1] & ~BIT(cpu_port)) == 0) {
		vlan_mapping.val[0] = 0;
		vlan_mapping.val[1] = 0;
		vlan_mapping.val[2] = 0;
		priv->vlans[vid].bridge = NULL;
		priv->vlans[vid].fid = 0;
	}
	err = gsw1xx_pce_table_entry_write(priv, &vlan_mapping);
	if (err) {
		dev_err(priv->dev, "%s: failed to write VLAN mapping: %d\n", __func__, err);
		return err;
	}

	/* configure default PVID if VID to remove is PVID */
	if (vid == gsw1xx_read_switch_reg(priv, GSW1XX_PCE_DEFPVID(port))) {
		if (bridge == NULL) {
			gsw1xx_switch_write(priv, 0, GSW1XX_PCE_DEFPVID(port));
			/* update bridge_vlan information */
			priv->bridge_vlans[port].bridge = NULL;
			priv->bridge_vlans[port].vid = 0;
			priv->bridge_vlans[port].fid = 0;
		} else {
			gsw1xx_switch_write(priv, 0, GSW1XX_PCE_DEFPVID(port));
			/* update bridge_vlan information */
			priv->bridge_vlans[port].bridge = bridge;
			priv->bridge_vlans[port].vid = 0;
			priv->bridge_vlans[port].fid = gsw1xx_get_fid(priv, bridge);
			/* In case of non PVID ports only tagged frames are allowed. */
			priv->bridge_vlans[port].pvid = 0;
		}
	}

	/* In case of non PVID ports only tagged frames are allowed. */
	if (dsa_port_is_vlan_filtering(dsa_to_port(priv->ds, port)))
		if (priv->bridge_vlans[port].pvid == 0) {
			gsw1xx_modify_switch_reg(priv,
						 GSW1XX_PCE_VCTRL_VINR_MASK,
						 GSW1XX_PCE_VCTRL_VINR_VTAG,
						 GSW1XX_PCE_VCTRL(port));
	}
	return 0;
}

/* Assign PVID to zero to add the standalone LAN ports into a bridge with the CPU port by
 * default. In conjunction with learning disabled and fast aging,
 * this prevents automatic forwarding of packets between the standalone
 * and bridge ports.
 */
static int gsw1xx_standalone_port(struct gsw1xx_priv *priv, int port, bool enable, bool restore_default_pvid)
{
#if (GSW1XX_STANDALONE_SINGLE_PORT_BR == 1)
	int err;
#endif

	if (enable) {
		/* disable MAC address learning on standalone ports */
		gsw1xx_port_set_learning(priv, port, false);

#if (GSW1XX_STANDALONE_SINGLE_PORT_BR == 1)
		err = gsw1xx_vlan_add_aware(priv, NULL, port, GSW1XX_STANDALONE_PORT_PVID, true, false);
		if (err) {
			dev_err(priv->dev, "failed to write VLAN mapping: %d\n",
				err);
			return err;
		}
#endif

	} else {
		/* enable MAC address learning on bridge ports */
		gsw1xx_port_set_learning(priv, port, true);
#if (GSW1XX_STANDALONE_SINGLE_PORT_BR == 1)
		err = gsw1xx_vlan_remove(priv, NULL, port, GSW1XX_STANDALONE_PORT_PVID, true, false);
		if (err) {
			dev_err(priv->dev, "failed to write VLAN mapping: %d\n",
				err);
			return err;
		}
#endif
	}

	if (restore_default_pvid)
		gsw1xx_switch_write(priv, GSW1XX_STANDALONE_PORT_PVID, GSW1XX_PCE_DEFPVID(port));

	return 0;
}

static void gsw1xx_port_disable(struct dsa_switch *ds, int port)
{
	if (!dsa_is_user_port(ds, port))
		return;

	gsw1xx_modify_switch_reg(ds->priv, GSW1XX_FDMA_PCTRL_EN, 0,
				 GSW1XX_FDMA_PCTRLp(port));

	gsw1xx_modify_switch_reg(ds->priv, GSW1XX_SDMA_PCTRL_EN, 0,
				 GSW1XX_SDMA_PCTRLp(port));
}

static int gsw1xx_led_cfg(struct dsa_switch *ds, int port)
{
	struct gsw1xx_priv *priv = ds->priv;
	int ret = 0;

	ret = gsw1xx_mmd_write(priv, port, ((GSW1XX_LEDXH_CFG_pin(0) >> 16) & 0xFF),
			       (GSW1XX_LEDXH_CFG_pin(0) & 0xFFFF), priv->dts.led0h_settings);
	if (ret < 0)
		return ret;
	ret = gsw1xx_mmd_write(priv, port, ((GSW1XX_LEDXL_CFG_pin(0) >> 16) & 0xFF),
			       (GSW1XX_LEDXL_CFG_pin(0) & 0xFFFF), priv->dts.led0l_settings);
	if (ret < 0)
		return ret;

	ret = gsw1xx_mmd_write(priv, port, ((GSW1XX_LEDXH_CFG_pin(1) >> 16) & 0xFF),
			       (GSW1XX_LEDXH_CFG_pin(1) & 0xFFFF), priv->dts.led1h_settings);
	if (ret < 0)
		return ret;
	ret = gsw1xx_mmd_write(priv, port, ((GSW1XX_LEDXL_CFG_pin(1) >> 16) & 0xFF),
			       (GSW1XX_LEDXL_CFG_pin(1) & 0xFFFF), priv->dts.led1h_settings);
	if (ret < 0)
		return ret;

	ret = gsw1xx_mmd_write(priv, port, ((GSW1XX_LEDXH_CFG_pin(2) >> 16) & 0xFF),
			       (GSW1XX_LEDXH_CFG_pin(2) & 0xFFFF), priv->dts.led2h_settings);
	if (ret < 0)
		return ret;
	ret = gsw1xx_mmd_write(priv, port, ((GSW1XX_LEDXL_CFG_pin(2) >> 16) & 0xFF),
			       (GSW1XX_LEDXL_CFG_pin(2) & 0xFFFF), priv->dts.led2h_settings);
	if (ret < 0)
		return ret;

	return ret;
}

static int gsw1xx_port_enable(struct dsa_switch *ds, int port,
								struct phy_device *phydev)
{
	struct gsw1xx_priv *priv = ds->priv;
	int err;

	if (!dsa_is_user_port(ds, port))
		return 0;

	if ((!dsa_is_cpu_port(ds, port)) && !priv->bridge_vlans[port].bridge) {
		err = gsw1xx_standalone_port(priv, port, true, false);
		if (err)
			return err;
	}

	/* RMON Counter Enable for port */
	gsw1xx_switch_write(priv, GSW1XX_BM_PCFG_CNTEN, GSW1XX_BM_PCFGp(port));

	/* enable port fetch/store dma & VLAN Modification */
	gsw1xx_modify_switch_reg(priv, 0, GSW1XX_FDMA_PCTRL_EN |
				 GSW1XX_FDMA_PCTRL_VLANMOD_BOTH,
				 GSW1XX_FDMA_PCTRLp(port));
	gsw1xx_modify_switch_reg(priv, 0, GSW1XX_SDMA_PCTRL_EN,
				 GSW1XX_SDMA_PCTRLp(port));

	if (!dsa_is_cpu_port(ds, port)) {
		u32 mdio_phy = port;

		if (phydev)
			mdio_phy = phydev->mdio.addr & GSW1XX_MDIO_PHY_ADDR_MASK;

		/* set MDIO addr in PHY_ADDR_x register */
		gsw1xx_modify_mdio_master_switch_reg(priv, GSW1XX_MDIO_PHY_ADDR_MASK, mdio_phy,
						     GSW1XX_MDIO_PHYp(port));
	}

	return 0;
}

static int gsw1xx_pce_load_microcode(struct gsw1xx_priv *priv)
{
	int i;
	int err;

	gsw1xx_modify_switch_reg(priv, GSW1XX_PCE_TBL_CTRL_ADDR_MASK |
				 GSW1XX_PCE_TBL_CTRL_OPMOD_MASK,
				 GSW1XX_PCE_TBL_CTRL_OPMOD_ADWR, GSW1XX_PCE_TBL_CTRL);
	gsw1xx_switch_write(priv, 0, GSW1XX_PCE_TBL_MASK);

	for (i = 0; i < ARRAY_SIZE(gsw1xx_pce_microcode); i++) {
		gsw1xx_switch_write(priv, i, GSW1XX_PCE_TBL_ADDR);
		gsw1xx_switch_write(priv, gsw1xx_pce_microcode[i].val_0,
				    GSW1XX_PCE_TBL_VAL(0));
		gsw1xx_switch_write(priv, gsw1xx_pce_microcode[i].val_1,
				    GSW1XX_PCE_TBL_VAL(1));
		gsw1xx_switch_write(priv, gsw1xx_pce_microcode[i].val_2,
				    GSW1XX_PCE_TBL_VAL(2));
		gsw1xx_switch_write(priv, gsw1xx_pce_microcode[i].val_3,
				    GSW1XX_PCE_TBL_VAL(3));

		/* start the table access */
		gsw1xx_modify_switch_reg(priv, 0, GSW1XX_PCE_TBL_CTRL_BAS,
					 GSW1XX_PCE_TBL_CTRL);
		err = gsw1xx_read_switch_reg_timeout(priv, GSW1XX_PCE_TBL_CTRL,
						     GSW1XX_PCE_TBL_CTRL_BAS);
		if (err)
			return err;
	}

	/* tell the switch that the microcode is loaded */
	gsw1xx_modify_switch_reg(priv, 0, GSW1XX_PCE_GCTRL_0_MC_VALID,
				 GSW1XX_PCE_GCTRL_0);

	return 0;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
static int gsw1xx_port_vlan_filtering(struct dsa_switch *ds, int port,
				      bool vlan_filtering)
{
	struct gsw1xx_priv *priv = ds->priv;
	struct net_device *bridge = dsa_to_port(ds, port)->bridge_dev;

	if (!!(priv->port_vlan_filter & BIT(port)) != vlan_filtering && bridge)
		return -EIO;

	/* When VLAN filtering is turned on, the hardware must be programmed with rejecting 802.1Q
	 * frames which have VLAN IDs outside of the programmed allowed VLAN ID map/rules.
	 * If there is no PVID programmed into the switch port, untagged frames must be rejected as well.
	 * When turned off the switch must accept any 802.1Q frames irrespective of their VLAN ID,
	 * and untagged frames are allowed.
	 */

	if (vlan_filtering) {
		/* Enable Ingress / Egress Member Violation and discard frames which are no members.
		 * VLAN Security Rule disabled -> use frame VLAN received in ingress frames
		 */
		gsw1xx_modify_switch_reg(priv,
					 GSW1XX_PCE_VCTRL_VSR,
					 GSW1XX_PCE_VCTRL_VIMR | GSW1XX_PCE_VCTRL_VEMR,
					 GSW1XX_PCE_VCTRL(port));
		/* Disable Transparent VLAN Mode */
		gsw1xx_modify_switch_reg(priv, GSW1XX_PCE_PCTRL_0_TVM, 0,
					 GSW1XX_PCE_PCTRL_0p(port));
	} else {
		/* Disable Ingress / Egress Member Violation and accept all frames
		 * VLAN Security Rule enabled -> use port VLAN, replace outer VLAN on egress
		 */
		gsw1xx_modify_switch_reg(priv,
					 GSW1XX_PCE_VCTRL_VIMR | GSW1XX_PCE_VCTRL_VEMR,
					 GSW1XX_PCE_VCTRL_VSR,
					 GSW1XX_PCE_VCTRL(port));
		/* Enable Transparent VLAN Mode */
		gsw1xx_modify_switch_reg(priv, 0, GSW1XX_PCE_PCTRL_0_TVM,
					 GSW1XX_PCE_PCTRL_0p(port));
	}

	return 0;
}
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(5, 12, 0))
static int gsw1xx_port_vlan_filtering(struct dsa_switch *ds, int port,
				      bool vlan_filtering,
				      struct switchdev_trans *trans)
{
	struct gsw1xx_priv *priv = ds->priv;

	/* Do not allow changing the VLAN filtering options while in bridge */
	if (switchdev_trans_ph_prepare(trans)) {
		struct dsa_port *gsw1xx_dsa_port = dsa_to_port(ds, port);
		struct net_device *bridge;

		if (gsw1xx_dsa_port != NULL)
			bridge = gsw1xx_dsa_port->bridge_dev;
		else
			return -EINVAL;

		if (!bridge)
			return 0;

		if (!!(priv->port_vlan_filter & BIT(port)) != vlan_filtering)
			return -EIO;

		return 0;
	}

	if (vlan_filtering) {
		/* Enable Ingress / Egress Member Violation and discard frames which are no members.
		 * VLAN Security Rule disabled -> use frame VLAN received in ingress frames
		 */
		gsw1xx_modify_switch_reg(priv,
				GSW1XX_PCE_VCTRL_VSR,
				GSW1XX_PCE_VCTRL_VIMR | GSW1XX_PCE_VCTRL_VEMR,
				GSW1XX_PCE_VCTRL(port));
		/* Disable Transparent VLAN Mode */
		gsw1xx_modify_switch_reg(priv, GSW1XX_PCE_PCTRL_0_TVM, 0,
				GSW1XX_PCE_PCTRL_0p(port));
	} else {
		/* Disable Ingress / Egress Member Violation and accept all frames
		 * VLAN Security Rule enabled -> use port VLAN, replace outer VLAN on egress
		 */
		gsw1xx_modify_switch_reg(priv,
				GSW1XX_PCE_VCTRL_VIMR | GSW1XX_PCE_VCTRL_VEMR | GSW1XX_PCE_VCTRL_VINR_MASK,
				GSW1XX_PCE_VCTRL_VSR,
				GSW1XX_PCE_VCTRL(port));
		/* Enable Transparent VLAN Mode */
		gsw1xx_modify_switch_reg(priv, 0, GSW1XX_PCE_PCTRL_0_TVM,
				GSW1XX_PCE_PCTRL_0p(port));
	}

	return 0;
}
#else
static int gsw1xx_port_vlan_filtering(struct dsa_switch *ds, int port,
				      bool vlan_filtering,
				      struct netlink_ext_ack *extack)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
	struct dsa_port *gsw1xx_dsa_port = dsa_to_port(ds, port);
	struct net_device *bridge;
#else
	struct net_device *bridge = dsa_port_bridge_dev_get(dsa_to_port(ds, port));
#endif
	struct gsw1xx_priv *priv = ds->priv;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
	if (gsw1xx_dsa_port)
		bridge = gsw1xx_dsa_port->bridge_dev;
	else
		return -EINVAL;
#endif

	/* Do not allow changing the VLAN filtering options while in bridge */
	if (bridge && !!(priv->port_vlan_filter & BIT(port)) != vlan_filtering) {
		NL_SET_ERR_MSG_MOD(extack,
				   "Dynamic toggling of vlan_filtering not supported");
		return -EIO;
	}

	if (vlan_filtering) {
		/* Enable Ingress / Egress Member Violation and discard frames which are no members.
		 * VLAN Security Rule disabled -> use frame VLAN received in ingress frames
		 */
		gsw1xx_modify_switch_reg(priv,
					 GSW1XX_PCE_VCTRL_VSR,
					 GSW1XX_PCE_VCTRL_VIMR | GSW1XX_PCE_VCTRL_VEMR,
					 GSW1XX_PCE_VCTRL(port));
		/* Disable Transparent VLAN Mode */
		gsw1xx_modify_switch_reg(priv, GSW1XX_PCE_PCTRL_0_TVM, 0,
					 GSW1XX_PCE_PCTRL_0p(port));
	} else {
		/* Disable Ingress / Egress Member Violation and accept all frames
		 * VLAN Security Rule enabled -> use port VLAN, replace outer VLAN on egress
		 */
		gsw1xx_modify_switch_reg(priv,
					 GSW1XX_PCE_VCTRL_VIMR | GSW1XX_PCE_VCTRL_VEMR | GSW1XX_PCE_VCTRL_VINR_MASK,
					 GSW1XX_PCE_VCTRL_VSR,
					 GSW1XX_PCE_VCTRL(port));
		/* Enable Transparent VLAN Mode */
		gsw1xx_modify_switch_reg(priv, 0, GSW1XX_PCE_PCTRL_0_TVM,
					 GSW1XX_PCE_PCTRL_0p(port));
	}

	return 0;
}
#endif

static int gsw1xx_sgmii_configuration(struct gsw1xx_priv *priv)
{
	int ret;

	dev_err(priv->dev, "%s: SGMII mode: %d\n", __func__, priv->dts.sgmii_mode);

	/* SGMII  shell reset is enabled */
	ret = gsw1xx_write_reg(priv, GSW1XX_RST_REQ, 0x0030);
	if (ret < 0)
		return ret;
	/* SGMII shell reset is disabled*/
	ret = gsw1xx_write_reg(priv, GSW1XX_RST_REQ, 0x0010);
	if (ret < 0)
		return ret;
	/* Hardware Bringup FSM Enable  */
	ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_PHY_HWBU_CTRL, 0x0009);
	if (ret < 0)
		return ret;

#if defined(GSW1XX_SGMII_RX_SIGNAL_INV) && GSW1XX_SGMII_RX_SIGNAL_INV
	ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_PHY_RX0_CFG2, 0x053A);
	if (ret < 0)
		return ret;
#else
	ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_PHY_RX0_CFG2, 0x0532);
	if (ret < 0)
		return ret;
#endif  /* SGMII_RX_SIGNAL_INV */

	/*  Reset and Release TBI */
	ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_TBI_TBICTL, 0x0033);
	if (ret < 0)
		return ret;
	ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_TBI_TBICTL, 0x0032);
	if (ret < 0)
		return ret;
	/* Release Tx Data Buffers */
	ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_PCS_TXB_CTL, 0x0003);
	if (ret < 0)
		return ret;
	ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_PCS_TXB_CTL, 0x0001);
	if (ret < 0)
		return ret;
	/* Release Rx Data Buffers */
	ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_PCS_RXB_CTL, 0x0003);
	if (ret < 0)
		return ret;
	ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_PCS_RXB_CTL, 0x0001);
	if (ret < 0)
		return ret;
	if (priv->dts.sgmii_mode == GSW1XX_SGMII_PHY) {
		/*  SGMII_PHY mode configuration Start */
		ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_TBI_TXANEGH, 0x0018);
		if (ret < 0)
			return ret;
		ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_TBI_TXANEGL, 0x0001);
		if (ret < 0)
			return ret;
		if (priv->dts.sgmii_auto_neg) {
			/*Setup Autoneg */
			ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_TBI_ANEGCTL, 0x00BD);
			if (ret < 0)
				return ret;
			ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_TBI_ANEGCTL, 0x00B5);
			if (ret < 0)
				return ret;
		} else {
			ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_TBI_ANEGCTL, 0x00BA);
			if (ret < 0)
				return ret;
			ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_TBI_ANEGCTL, 0x00B1);
			if (ret < 0)
				return ret;
			ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_TBI_LPSTAT, 0x0041);
			if (ret < 0)
				return ret;
		}
		/*  SGMII_PHY mode configuration End */
	} else if (priv->dts.sgmii_mode == GSW1XX_SGMII_MAC) {
		/*  SGMII_MAC mode configuration Start */
		ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_TBI_TXANEGH, 0x0040);
		if (ret < 0)
			return ret;
		ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_TBI_TXANEGL, 0x0001);
		if (ret < 0)
			return ret;
		/*Setup Autoneg */
		ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_TBI_ANEGCTL, 0x00FD);
		if (ret < 0)
			return ret;
		ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_TBI_ANEGCTL, 0x00F5);
		if (ret < 0)
			return ret;
		/*  SGMII_MAC mode configuration End */
	} else if (priv->dts.sgmii_mode == GSW1XX_SGMII_1000BX) {
		/*  1000 Base-X mode configuration Start  */
		ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_TBI_TXANEGH, 0x0000);
		if (ret < 0)
			return ret;
		ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_TBI_TXANEGL, 0x00A0);
		if (ret < 0)
			return ret;
		/*Setup Autoneg */
		ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_TBI_ANEGCTL, 0x007D);
		if (ret < 0)
			return ret;
		ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_TBI_ANEGCTL, 0x0075);
		if (ret < 0)
			return ret;
		/* 1000 Base-X mode configuration End */
	} else {
		dev_err(priv->dev, "%s: SGMII wrong mode: %d\n", __func__, priv->dts.sgmii_mode);
	}

	ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_PHY_D, 0x80);
	if (ret < 0)
		return ret;
	ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_PHY_A, 0x30);
	if (ret < 0)
		return ret;
	ret = gsw1xx_write_reg(priv, GSW1XX_SGMII_PHY_C, 0x1100);
	if (ret < 0)
		return ret;

	return 0;
}

static void gsw1xx_parse_dts_options(struct gsw1xx_priv *priv)
{
	struct device_node *np = priv->dev->of_node;
	u8 rx_mii_delay = GSW1XX_MII_PCDU_RXDLY_CUSTOM;
	u8 tx_mii_delay = GSW1XX_MII_PCDU_TXDLY_CUSTOM;

	priv->dts.smdio_badr = SMDIO_BADR;
	priv->dts.rmii_clk_custom = GSW1XX_MII_CFG_RMII_CLK_CUSTOM;
	priv->dts.auto_ldclk_disable = GSW1XX_MII_CFG_AUTO_LDCLK_DISABLE;
	priv->dts.rgmii_slew_rx_tx_volt = RGMII_SLEW_RX_TX_VOLT;
	priv->dts.use_sgmii = GSW1XX_USE_SGMII;
	priv->dts.sgmii_mode = GSW1XX_SGMII_MODE;
	priv->dts.sgmii_auto_neg = GSW1XX_SGMII_AUTO_NEG_ENABLED;
	priv->dts.led_settings = GSW1XX_CONFIG_CUSTOM_LED_SETTINGS;
	priv->dts.led0h_settings = GSW1XX_LED0H_CFG_CUSTOM;
	priv->dts.led0l_settings = GSW1XX_LED0L_CFG_CUSTOM;
	priv->dts.led1h_settings = GSW1XX_LED1H_CFG_CUSTOM;
	priv->dts.led1l_settings = GSW1XX_LED1L_CFG_CUSTOM;
	priv->dts.led2h_settings = GSW1XX_LED2H_CFG_CUSTOM;
	priv->dts.led2l_settings = GSW1XX_LED2L_CFG_CUSTOM;

	of_property_read_u8(np, "smdio-badr", &priv->dts.smdio_badr);
	of_property_read_u8(np, "mii-pcdu-rxdly", &rx_mii_delay);
	of_property_read_u8(np, "mii-pcdu-txdly", &tx_mii_delay);
	of_property_read_u8(np, "rmii-clk-custom", &priv->dts.rmii_clk_custom);
	of_property_read_u8(np, "auto-ldclk-disable", &priv->dts.auto_ldclk_disable);
	of_property_read_u8(np, "rgmii-slew-rx-tx-volt", &priv->dts.rgmii_slew_rx_tx_volt);
	of_property_read_u8(np, "use-sgmii", &priv->dts.use_sgmii);
	of_property_read_u8(np, "sgmii-mode", &priv->dts.sgmii_mode);
	of_property_read_u8(np, "sgmii-auto-neg", &priv->dts.sgmii_auto_neg);
	of_property_read_u8(np, "led-settings", &priv->dts.led_settings);
	of_property_read_u8(np, "led0l-settings", &priv->dts.led0l_settings);
	of_property_read_u8(np, "led0h-settings", &priv->dts.led0h_settings);
	of_property_read_u8(np, "led1l-settings", &priv->dts.led1l_settings);
	of_property_read_u8(np, "led1h-settings", &priv->dts.led1h_settings);
	of_property_read_u8(np, "led2l-settings", &priv->dts.led2l_settings);
	of_property_read_u8(np, "led2h-settings", &priv->dts.led2h_settings);

	priv->dts.rx_mii_delay = rx_mii_delay << GSW1XX_MII_PCDU_RXDLY_SHIFT;
	priv->dts.tx_mii_delay = tx_mii_delay << GSW1XX_MII_PCDU_TXDLY_SHIFT;
}

static int gsw1xx_setup(struct dsa_switch *ds)
{
	struct gsw1xx_priv *priv = ds->priv;
	unsigned int cpu_port = get_cpu_port(priv);
	int err;
	int i;

	dev_dbg(ds->dev, "%s: cpu_port: %d, max_ports:%d, internal_phy_ports:%d\n", __func__, cpu_port, priv->hw_info->max_ports, priv->hw_info->phy_ports);

	priv->ds = ds;

	gsw1xx_parse_dts_options(priv);
	gsw1xx_switch_write(priv, GSW1XX_SWRES_R0, GSW1XX_SWRES);
	/* wait until reset has been processed internally before applying new configuration */
	usleep_range(5000, 10000);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
	for (i = 0; i < priv->hw_info->max_ports; i++) {
		if (!gsw1xx_is_valid_non_cpu_port(priv, i))
			continue;
		/* switch port disable */
		gsw1xx_modify_switch_reg(ds->priv, GSW1XX_FDMA_PCTRL_EN, 0, GSW1XX_FDMA_PCTRLp(i));
		gsw1xx_modify_switch_reg(ds->priv, GSW1XX_SDMA_PCTRL_EN, 0, GSW1XX_SDMA_PCTRLp(i));
		/* Configure port based VLAN tag by default*/
		gsw1xx_port_vlan_filtering(ds, i, false);
		/* enable LPI - internal PHY ports advertise EEE in 1G mode */
		gsw1xx_modify_switch_reg(priv, 0, GSW1XX_MAC_CTRL_4_LPIEN, GSW1XX_MAC_CTRL_4p(i));
	}
#elif (LINUX_VERSION_CODE < KERNEL_VERSION(5, 12, 0))
	/* disable port fetch/store dma on all ports */
	for (i = 0; i < priv->hw_info->max_ports; i++) {
		struct switchdev_trans trans;

		if (!gsw1xx_is_valid_non_cpu_port(priv, i))
			continue;
		/* Skip the prepare phase, this shouldn't return an error
		 * during setup.
		 */
		trans.ph_prepare = false;
		/* switch port disable */
		gsw1xx_modify_switch_reg(ds->priv, GSW1XX_FDMA_PCTRL_EN, 0, GSW1XX_FDMA_PCTRLp(i));
		gsw1xx_modify_switch_reg(ds->priv, GSW1XX_SDMA_PCTRL_EN, 0, GSW1XX_SDMA_PCTRLp(i));
		/* Disable VLAN filtering by default */
		gsw1xx_port_vlan_filtering(ds, i, false, &trans);
		/* enable LPI - internal PHY ports advertise EEE in 1G mode */
		gsw1xx_modify_switch_reg(priv, 0, GSW1XX_MAC_CTRL_4_LPIEN, GSW1XX_MAC_CTRL_4p(i));
	}
#else
	/* disable port fetch/store dma on all ports */
	for (i = 0; i < priv->hw_info->max_ports; i++) {
		if (!gsw1xx_is_valid_non_cpu_port(priv, i))
			continue;
		/* switch port disable */
		gsw1xx_modify_switch_reg(ds->priv, GSW1XX_FDMA_PCTRL_EN, 0, GSW1XX_FDMA_PCTRLp(i));
		gsw1xx_modify_switch_reg(ds->priv, GSW1XX_SDMA_PCTRL_EN, 0, GSW1XX_SDMA_PCTRLp(i));
		/* Disable VLAN filtering by default */
		gsw1xx_port_vlan_filtering(ds, i, false, NULL);
		/* enable LPI - internal PHY ports advertise EEE in 1G mode */
		gsw1xx_modify_switch_reg(priv, 0, GSW1XX_MAC_CTRL_4_LPIEN, GSW1XX_MAC_CTRL_4p(i));
	}
#endif

	/* enable Switch */
	gsw1xx_modify_mdio_master_switch_reg(priv, 0, GSW1XX_MDIO_GLOB_ENABLE, GSW1XX_MDIO_GLOB);

	err = gsw1xx_pce_load_microcode(priv);
	if (err) {
		dev_err(priv->dev, "writing PCE microcode failed, %i", err);
		return err;
	}

	/* Monitoring Port Map */
	gsw1xx_switch_write(priv, BIT(cpu_port), GSW1XX_PCE_PMAP1);
	/* Default unknown Broadcast/Multicast Port Map */
	gsw1xx_switch_write(priv, BIT(cpu_port), GSW1XX_PCE_PMAP2);
	/* Default unknown Unicast Port Map */
	gsw1xx_switch_write(priv, BIT(cpu_port), GSW1XX_PCE_PMAP3);


	for (i = 0; i < priv->hw_info->max_ports; i++) {
		/* disable MAC address learning by default on all ports */
		gsw1xx_port_set_learning(priv, i, false);
	}

	/* disable autopolling on all ports */
	gsw1xx_write_mdio_master_switch_reg(priv, GSW1XX_MDIO_MDC_CFG0, 0x00);

	/* Configure the MDIO Clock 2.5 MHz */
	gsw1xx_modify_mdio_master_switch_reg(priv, 0xff, 0x109, GSW1XX_MDIO_MDC_CFG1);

	/* configure GPIO pin-mux for MMDIO in case of external PHY connected to SGMII or RGMII
	 * as slave interface
	 */
	gsw1xx_modify_reg(priv, 0, 3, GPIO_BASE + GPIO_ALTSEL0);
	gsw1xx_modify_reg(priv, 0, 3, GPIO_BASE + GPIO_ALTSEL1);

	if (priv->dts.rgmii_slew_rx_tx_volt == 1) {
		gsw1xx_modify_reg(ds->priv, 0,
				  RGMII_SLEW_CFG_RX_2_5_V | RGMII_SLEW_CFG_TX_2_5_V,
				  RGMII_SLEW_CFG);
	}
	/* enable special tag insertion on cpu port */
	gsw1xx_modify_switch_reg(priv, 0, GSW1XX_FDMA_PCTRL_STEN,
				 GSW1XX_FDMA_PCTRLp(cpu_port));

	/* accept special tag in ingress direction */
	gsw1xx_modify_switch_reg(priv, 0, GSW1XX_PCE_PCTRL_0_INGRESS,
				 GSW1XX_PCE_PCTRL_0p(cpu_port));

	/* WRED Mode GLOB - global WRED thresholds apply */
	gsw1xx_modify_switch_reg(priv, 0, GSW1XX_BM_QUEUE_GCTRL_GL_MOD,
				 GSW1XX_BM_QUEUE_GCTRL);

	/* Enable VLANMD 2.2 for 4k VLANs entries */
	gsw1xx_modify_switch_reg(priv, 0, GSW1XX_PCE_GCTRL_1_VLANMD, GSW1XX_PCE_GCTRL_1);

	/* VLAN aware Switching */
	gsw1xx_modify_switch_reg(priv, 0, GSW1XX_PCE_GCTRL_0_VLAN, GSW1XX_PCE_GCTRL_0);

	if (priv->dts.use_sgmii == GSW1XX_SGMII_WITH_EXTERNAL_PHY ||
	    priv->dts.use_sgmii == GSW1XX_SGMII_IS_CPU_PORT)
		gsw1xx_sgmii_configuration(priv);

	/* Flush MAC Table */
	gsw1xx_modify_switch_reg(priv, 0, GSW1XX_PCE_GCTRL_0_MTFL, GSW1XX_PCE_GCTRL_0);

	err = gsw1xx_read_switch_reg_timeout(priv, GSW1XX_PCE_GCTRL_0,
					     GSW1XX_PCE_GCTRL_0_MTFL);
	if (err) {
		dev_err(priv->dev, "MAC flushing didn't finish\n");
		return err;
	}

#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 6, 0))
	ds->mtu_enforcement_ingress = true;
#endif

	gsw1xx_port_enable(ds, cpu_port, NULL);

#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 7, 0))
	ds->configure_vlan_while_not_filtering = false;
#endif

	/* Initialize priv-struct */
	for (i = 0; i < ARRAY_SIZE(priv->vlans); i++) {
		priv->vlans[i].bridge = NULL;
		priv->vlans[i].fid = 0;
	}

	return 0;
}

static void gsw1xx_get_strings(struct dsa_switch *ds, int port, u32 stringset,
			       uint8_t *data)
{
	int i;

	if (stringset != ETH_SS_STATS)
		return;
	for (i = 0; i < ARRAY_SIZE(gsw1xx_rmon_cnt); i++)
		strncpy(data + i * ETH_GSTRING_LEN, gsw1xx_rmon_cnt[i].name,
			ETH_GSTRING_LEN);
}

static u32 gsw1xx_bcm_ram_entry_read(struct gsw1xx_priv *priv, u32 table,
				     u32 index)
{
	u32 result;
	int err;

	gsw1xx_switch_write(priv, index, GSW1XX_BM_RAM_ADDR);
	gsw1xx_modify_switch_reg(priv, GSW1XX_BM_RAM_CTRL_ADDR_MASK |
				 GSW1XX_BM_RAM_CTRL_OPMOD,
				 table | GSW1XX_BM_RAM_CTRL_BAS,
				 GSW1XX_BM_RAM_CTRL);

	err = gsw1xx_read_switch_reg_timeout(priv, GSW1XX_BM_RAM_CTRL,
					     GSW1XX_BM_RAM_CTRL_BAS);
	if (err) {
		dev_err(priv->dev, "timeout while reading table: %u, index: %u",
			table, index);
		return 0;
	}

	result = gsw1xx_read_switch_reg(priv, GSW1XX_BM_RAM_VAL(0));
	result |= gsw1xx_read_switch_reg(priv, GSW1XX_BM_RAM_VAL(1)) << 16;

	return result;
}

static void gsw1xx_get_ethtool_stats(struct dsa_switch *ds, int port,
				     u64 *data)
{
	struct gsw1xx_priv *priv = ds->priv;
	const struct gsw1xx_rmon_cnt_desc *rmon_cnt;
	int i;
	u64 high;

	for (i = 0; i < ARRAY_SIZE(gsw1xx_rmon_cnt); i++) {
		rmon_cnt = &gsw1xx_rmon_cnt[i];

		data[i] = gsw1xx_bcm_ram_entry_read(priv, port,
						    rmon_cnt->offset);
		if (rmon_cnt->size == 2) {
			high = gsw1xx_bcm_ram_entry_read(priv, port,
							 rmon_cnt->offset + 1);
			data[i] |= high << 32;
		}
	}
}

static int gsw1xx_get_sset_count(struct dsa_switch *ds, int port, int sset)
{
	if (sset != ETH_SS_STATS)
		return 0;

	return ARRAY_SIZE(gsw1xx_rmon_cnt);
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
static int gsw1xx_port_bridge_join(struct dsa_switch *ds, int port,
				   struct net_device *bridge)
{
	struct gsw1xx_priv *priv = ds->priv;
	int err;

	if (!br_vlan_enabled(bridge)) {
		err = gsw1xx_vlan_add_unaware(ds, port, bridge);
		if (err)
			return err;
		priv->port_vlan_filter &= ~BIT(port);
	} else {
		/* When the bridge uses VLAN filtering we have to configure VLAN
		 * specific bridges. No bridge is configured here in this case.
		 */
		priv->port_vlan_filter |= BIT(port);
		/* Stop traffic in case of VLAN to prevent isolation issues for standalone ports.
		 * It will be activated in vlan add again.
		 */
		gsw1xx_modify_switch_reg(ds->priv, GSW1XX_FDMA_PCTRL_EN, 0, GSW1XX_FDMA_PCTRLp(port));
		gsw1xx_modify_switch_reg(ds->priv, GSW1XX_SDMA_PCTRL_EN, 0, GSW1XX_SDMA_PCTRLp(port));
	}

	/* change port from standalone to bridge mode */
	err = gsw1xx_standalone_port(priv, port, false, false);

	return err;
}
#else
static int gsw1xx_port_bridge_join(struct dsa_switch *ds, int port,
				   struct dsa_bridge bridge,
				   bool *tx_fwd_offload,
				   struct netlink_ext_ack *extack)
{
	struct net_device *br = bridge.dev;
	struct gsw1xx_priv *priv = ds->priv;
	int err;

	if (!br_vlan_enabled(br)) {
		err = gsw1xx_vlan_add_unaware(ds, port, br);
		if (err)
			return err;
		priv->port_vlan_filter &= ~BIT(port);
	} else {
		/* When the bridge uses VLAN filtering we have to configure VLAN
		 * specific bridges. No bridge is configured here in this case.
		 */
		priv->port_vlan_filter |= BIT(port);
	}

	/* change port from standalone to bridge mode */
	err = gsw1xx_standalone_port(priv, port, false, false);

	return err;
}
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
static void gsw1xx_port_bridge_leave(struct dsa_switch *ds, int port,
				     struct net_device *bridge)
{
	struct gsw1xx_priv *priv = ds->priv;

	/* change port from bridge to standalone mode */
	gsw1xx_standalone_port(priv, port, true, true);

	/* Remove bridge When the bridge uses VLAN filtering we have to configure VLAN
	 * specific bridges. No bridge is configured here.
	 */
	if (!br_vlan_enabled(bridge))
		gsw1xx_vlan_del_unaware(ds, port, bridge);

	/* update bridge_vlan information */
	priv->bridge_vlans[port].bridge = NULL;
	priv->bridge_vlans[port].vid = 0;
	priv->bridge_vlans[port].fid = 0;
	priv->bridge_vlans[port].pvid = 0;
}
#else
static void gsw1xx_port_bridge_leave(struct dsa_switch *ds, int port,
				     struct dsa_bridge bridge)
{
	struct net_device *br = bridge.dev;
	struct gsw1xx_priv *priv = ds->priv;

	/* change port from bridge to standalone mode */
	gsw1xx_standalone_port(priv, port, true, true);

	/* Remove bridge: When the bridge uses VLAN filtering we have to
	 * configure VLAN specific bridges. No bridge is configured here.
	 */
	if (!br_vlan_enabled(br))
		gsw1xx_vlan_del_unaware(ds, port, br);

	/* update bridge_vlan information */
	priv->bridge_vlans[port].bridge = NULL;
	priv->bridge_vlans[port].vid = 0;
	priv->bridge_vlans[port].fid = 0;
	priv->bridge_vlans[port].pvid = 0;
}
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 12, 0))
static int gsw1xx_port_vlan_prepare(struct dsa_switch *ds, int port,
				    const struct switchdev_obj_port_vlan *vlan)
{
	const struct dsa_port *gsw1xx_dsa_port = dsa_to_port(ds, port);
	struct net_device *bridge;

	if (!gsw1xx_dsa_port)
		return -EINVAL;
	/* in case of standalone ports */

	return 0;
}
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 12, 0))
static void gsw1xx_port_vlan_add(struct dsa_switch *ds, int port,
				 const struct switchdev_obj_port_vlan *vlan)
{
	const struct dsa_port *gsw1xx_dsa_port = dsa_to_port(ds, port);
	bool untagged = vlan->flags & BRIDGE_VLAN_INFO_UNTAGGED;
	bool pvid = vlan->flags & BRIDGE_VLAN_INFO_PVID;
	struct gsw1xx_priv *priv = ds->priv;
	struct net_device *bridge;
	u16 vid;

	if (gsw1xx_dsa_port != NULL)
		bridge = gsw1xx_dsa_port->bridge_dev;
	else
		return;

	/* We have to receive all packets on the CPU port and should not
	 * do any VLAN filtering here. This is also called with bridge
	 * NULL and then we do not know for which bridge to configure
	 * this.
	 */
	if (dsa_is_cpu_port(ds, port))
		return;

	for (vid = vlan->vid_begin; vid <= vlan->vid_end; ++vid)
		gsw1xx_vlan_add_aware(priv, bridge, port, vid, untagged, pvid);

	/* Re-enable traffic in case of VLAN to ensure isolation issues for standalone ports */
	gsw1xx_modify_switch_reg(ds->priv, 0, GSW1XX_FDMA_PCTRL_EN, GSW1XX_FDMA_PCTRLp(port));
	gsw1xx_modify_switch_reg(ds->priv, 0, GSW1XX_SDMA_PCTRL_EN, GSW1XX_SDMA_PCTRLp(port));
}
#else
static int gsw1xx_port_vlan_add(struct dsa_switch *ds, int port,
				const struct switchdev_obj_port_vlan *vlan,
				struct netlink_ext_ack *extack)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
	struct dsa_port *gsw1xx_dsa_port = dsa_to_port(ds, port);
	struct net_device *bridge;
#else
	struct net_device *bridge = dsa_port_bridge_dev_get(dsa_to_port(ds, port));
#endif
	struct gsw1xx_priv *priv = ds->priv;
	bool untagged = vlan->flags & BRIDGE_VLAN_INFO_UNTAGGED;
	bool pvid = vlan->flags & BRIDGE_VLAN_INFO_PVID;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
	if (gsw1xx_dsa_port)
		bridge = gsw1xx_dsa_port->bridge_dev;
	else
		return -EINVAL;
#endif

	/* We have to receive all packets on the CPU port and should not
	 * do any VLAN filtering here. This is also called with bridge
	 * NULL and then we do not know for which bridge to configure
	 * this.
	 */
	if (dsa_is_cpu_port(ds, port))
		return 0;

	gsw1xx_vlan_add_aware(priv, bridge, port, vlan->vid,
			      untagged, pvid);

	/* Re-enable traffic in case of VLAN to ensure isolation issues for standalone ports */
	gsw1xx_modify_switch_reg(ds->priv, 0, GSW1XX_FDMA_PCTRL_EN, GSW1XX_FDMA_PCTRLp(port));
	gsw1xx_modify_switch_reg(ds->priv, 0, GSW1XX_SDMA_PCTRL_EN, GSW1XX_SDMA_PCTRLp(port));
	return 0;
}
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 12, 0))
static int gsw1xx_port_vlan_del(struct dsa_switch *ds, int port,
				const struct switchdev_obj_port_vlan *vlan)
{
	struct gsw1xx_priv *priv = ds->priv;
	const struct dsa_port *gsw1xx_dsa_port = dsa_to_port(ds, port);
	/* hint: pvid flag seems not to be supported in Linux for vlan del */
	bool pvid = vlan->flags & BRIDGE_VLAN_INFO_PVID;
	struct net_device *bridge;
	u16 vid;
	int err;

	if (gsw1xx_dsa_port != NULL)
		bridge = gsw1xx_dsa_port->bridge_dev;
	else
		return -EINVAL;

	/* We have to receive all packets on the CPU port and should not
	 * do any VLAN filtering here. This is also called with bridge
	 * NULL and then we do not know for which bridge to configure
	 * this.
	 */
	if (dsa_is_cpu_port(ds, port))
		return 0;

	for (vid = vlan->vid_begin; vid <= vlan->vid_end; ++vid) {
		err = gsw1xx_vlan_remove(priv, bridge, port, vid, pvid, true);
		if (err)
			return err;
	}

	/* if pvid will be removed, we have to discard untagged frames */
	if (dsa_port_is_vlan_filtering(dsa_to_port(priv->ds, port)))
		if (pvid) {
			/* Discard untagged frames */
			gsw1xx_modify_switch_reg(priv,
						 GSW1XX_PCE_VCTRL_VINR_MASK,
						 GSW1XX_PCE_VCTRL_VINR_VTAG,
						 GSW1XX_PCE_VCTRL(port));
		}

	return 0;
}
#else
static int gsw1xx_port_vlan_del(struct dsa_switch *ds, int port,
				const struct switchdev_obj_port_vlan *vlan)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
	struct dsa_port *gsw1xx_dsa_port = dsa_to_port(ds, port);
	struct net_device *bridge;
#else
	struct net_device *bridge = dsa_port_bridge_dev_get(dsa_to_port(ds, port));
#endif
	struct gsw1xx_priv *priv = ds->priv;
	bool pvid = vlan->flags & BRIDGE_VLAN_INFO_PVID;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
	if (gsw1xx_dsa_port)
		bridge = gsw1xx_dsa_port->bridge_dev;
	else
		return -EINVAL;
#endif

	/* We have to receive all packets on the CPU port and should not
	 * do any VLAN filtering here. This is also called with bridge
	 * NULL and then we do not know for which bridge to configure
	 * this.
	 */
	if (dsa_is_cpu_port(ds, port))
		return 0;

	/* if pvid will be removed, we have to discard untagged frames */
	if (dsa_port_is_vlan_filtering(dsa_to_port(ds, port)))
		if (pvid)
			/* Discard untagged frames */
			gsw1xx_modify_switch_reg(priv,
						 GSW1XX_PCE_VCTRL_VINR_MASK,
						 GSW1XX_PCE_VCTRL_VINR_VTAG,
						 GSW1XX_PCE_VCTRL(port));

	return gsw1xx_vlan_remove(priv, bridge, port, vlan->vid, pvid, true);
}
#endif

static void gsw1xx_port_fast_age(struct dsa_switch *ds, int port)
{
	struct gsw1xx_priv *priv = ds->priv;
	struct gsw1xx_pce_table_entry mac_bridge = {0,};
	int i;
	int err;

	for (i = 0; i < GSW1XX_MAC_LEARN_TABLE_SIZE; i++) {
		mac_bridge.table = GSW1XX_TABLE_MAC_BRIDGE;
		mac_bridge.index = i;

		err = gsw1xx_pce_table_entry_read(priv, &mac_bridge);
		if (err) {
			dev_err(priv->dev, "failed to read mac bridge: %d\n",
				err);
			return;
		}

		if (!mac_bridge.valid)
			continue;

		if ((mac_bridge.val[1] & GSW1XX_TABLE_MAC_BRIDGE_STATIC)
		    && (mac_bridge.val[1] & GSW1XX_TABLE_MAC_BRIDGE_STATIC_VALID))
			continue;

		if (((mac_bridge.val[0] & GENMASK(7, 4)) >> 4) != port)
			continue;

		mac_bridge.valid = false;
		err = gsw1xx_pce_table_entry_write(priv, &mac_bridge);
		if (err) {
			dev_err(priv->dev, "failed to write mac bridge: %d\n",
				err);
			return;
		}
	}
}

static void gsw1xx_port_stp_state_set(struct dsa_switch *ds, int port, u8 state)
{
	struct gsw1xx_priv *priv = ds->priv;
	u32 stp_state;

	if (!gsw1xx_is_valid_non_cpu_port(priv, port))
		return;

	switch (state) {
	case BR_STATE_DISABLED:
		gsw1xx_modify_switch_reg(priv, GSW1XX_SDMA_PCTRL_EN, 0,
					 GSW1XX_SDMA_PCTRLp(port));
		return;
	case BR_STATE_BLOCKING:
	case BR_STATE_LISTENING:
		stp_state = GSW1XX_PCE_PCTRL_0_PSTATE_LISTEN;
		break;
	case BR_STATE_LEARNING:
		stp_state = GSW1XX_PCE_PCTRL_0_PSTATE_LEARNING;
		break;
	case BR_STATE_FORWARDING:
		stp_state = GSW1XX_PCE_PCTRL_0_PSTATE_FORWARDING;
		break;
	default:
		dev_err(priv->dev, "invalid STP state: %d\n", state);
		return;
	}

	gsw1xx_modify_switch_reg(priv, 0, GSW1XX_SDMA_PCTRL_EN,
				 GSW1XX_SDMA_PCTRLp(port));
	gsw1xx_modify_switch_reg(priv, GSW1XX_PCE_PCTRL_0_PSTATE_MASK, stp_state,
				 GSW1XX_PCE_PCTRL_0p(port));
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0))
static int gsw1xx_port_fdb(struct dsa_switch *ds, int port,
			   const unsigned char *addr, u16 vid, bool add)
{
	struct gsw1xx_priv *priv = ds->priv;
	struct net_device *br;
	struct gsw1xx_pce_table_entry mac_bridge = {0,};
	unsigned int max_ports = priv->hw_info->max_ports;
	int fid = -1;
	int err;
	int i;

	if (dsa_to_port(ds, port))
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
		br = dsa_to_port(ds, port)->bridge_dev;
#else
		br = dsa_port_bridge_dev_get(dsa_to_port(ds, port));
#endif
	else
		return -EINVAL;

	for (i = max_ports; i < ARRAY_SIZE(priv->vlans); i++) {
		if (priv->vlans[i].bridge == br) {
			fid = priv->vlans[i].fid;
			break;
		}
	}

	if (fid == -1)
		/* standalone ports are using fid = 0 */
		fid = 0;

	mac_bridge.table = GSW1XX_TABLE_MAC_BRIDGE;
	mac_bridge.key_mode = true;
	mac_bridge.key[0] = addr[5] | (addr[4] << 8);
	mac_bridge.key[1] = addr[3] | (addr[2] << 8);
	mac_bridge.key[2] = addr[1] | (addr[0] << 8);
	mac_bridge.key[3] = fid;
	mac_bridge.val[0] = add ? BIT(port) : 0; /* port map */
	/* static, STAG vid and valid indicator */
	mac_bridge.val[1] = add ? (GSW1XX_TABLE_MAC_BRIDGE_STATIC |
				   GSW1XX_TABLE_MAC_BRIDGE_STATIC_VALID) : 0;
	mac_bridge.valid = add;

	err = gsw1xx_pce_table_entry_write(priv, &mac_bridge);
	if (err)
		dev_err(priv->dev, "failed to write mac bridge: %d\n", err);

	return err;
}
#else
static int gsw1xx_port_fdb(struct dsa_switch *ds, int port,
			   const unsigned char *addr, u16 vid, struct dsa_db db,
			   bool add)
{
	struct gsw1xx_priv *priv = ds->priv;
	struct gsw1xx_pce_table_entry mac_bridge = {0,};
	int fid = -1;
	int i;
	int err;

	switch (db.type) {
	case DSA_DB_BRIDGE:
		for (i = 0; i < ARRAY_SIZE(priv->vlans); i++) {
			if (priv->vlans[i].bridge == db.bridge.dev) {
				fid = priv->vlans[i].fid;
				break;
			}
		}
		if (fid == -1) {
			dev_err(priv->dev, "Port %d not part of a bridge\n", port);
			return -EINVAL;
		}
		break;
	case DSA_DB_PORT:
		if (dsa_is_cpu_port(ds, port) &&
			dsa_fdb_present_in_other_db(ds, port, addr, vid, db))
			return 0;
		/* FID of a standalone / single port bridge */
		fid = 0;
		break;
	default:
		return -EOPNOTSUPP;
	}

	mac_bridge.table = GSW1XX_TABLE_MAC_BRIDGE;
	mac_bridge.key_mode = true;
	mac_bridge.key[0] = addr[5] | (addr[4] << 8);
	mac_bridge.key[1] = addr[3] | (addr[2] << 8);
	mac_bridge.key[2] = addr[1] | (addr[0] << 8);
	mac_bridge.key[3] = fid;
	mac_bridge.val[0] = add ? BIT(port) : 0; /* port map */
	/* static, STAG vid and valid indicator */
	mac_bridge.val[1] = add ? (GSW1XX_TABLE_MAC_BRIDGE_STATIC
				 | GSW1XX_TABLE_MAC_BRIDGE_STATIC_VALID) : 0;
	mac_bridge.valid = add;

	err = gsw1xx_pce_table_entry_write(priv, &mac_bridge);
	if (err)
		dev_err(priv->dev, "failed to write mac bridge: %d\n", err);

	return err;
}
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 18, 0))
static int gsw1xx_port_fdb_add(struct dsa_switch *ds, int port,
				  const unsigned char *addr, u16 vid)
{
	return gsw1xx_port_fdb(ds, port, addr, vid, true);
}

static int gsw1xx_port_fdb_del(struct dsa_switch *ds, int port,
			       const unsigned char *addr, u16 vid)
{
	return gsw1xx_port_fdb(ds, port, addr, vid, false);
}
#else
static int gsw1xx_port_fdb_add(struct dsa_switch *ds, int port,
			       const unsigned char *addr, u16 vid,
			       struct dsa_db db)
{
	return gsw1xx_port_fdb(ds, port, addr, vid, db, true);
}

static int gsw1xx_port_fdb_del(struct dsa_switch *ds, int port,
			       const unsigned char *addr, u16 vid,
			       struct dsa_db db)
{
	return gsw1xx_port_fdb(ds, port, addr, vid, db, false);
}
#endif

static int gsw1xx_port_fdb_dump(struct dsa_switch *ds, int port,
				dsa_fdb_dump_cb_t *cb, void *data)
{
	struct gsw1xx_priv *priv = ds->priv;
	struct gsw1xx_pce_table_entry mac_bridge = {0,};
	unsigned char addr[6];
	int i;
	int err;

	for (i = 0; i < GSW1XX_MAC_LEARN_TABLE_SIZE; i++) {
		mac_bridge.table = GSW1XX_TABLE_MAC_BRIDGE;
		mac_bridge.index = i;

		err = gsw1xx_pce_table_entry_read(priv, &mac_bridge);
		if (err) {
			dev_err(priv->dev, "failed to read mac bridge entry %d: %d\n", i, err);
			return err;
		}

		if (!mac_bridge.valid)
			continue;

		addr[5] = mac_bridge.key[0] & 0xff;
		addr[4] = (mac_bridge.key[0] >> 8) & 0xff;
		addr[3] = mac_bridge.key[1] & 0xff;
		addr[2] = (mac_bridge.key[1] >> 8) & 0xff;
		addr[1] = mac_bridge.key[2] & 0xff;
		addr[0] = (mac_bridge.key[2] >> 8) & 0xff;

		if ((mac_bridge.val[1] & GSW1XX_TABLE_MAC_BRIDGE_STATIC)
		    && (mac_bridge.val[1] & GSW1XX_TABLE_MAC_BRIDGE_STATIC_VALID))
			if (mac_bridge.val[0] & BIT(port)) {
				cb(addr, 0, true, data); /* mac, vid, noarp, data */
			}
		else
			if (((mac_bridge.val[0] & GENMASK(7, 4)) >> 4) == port)
				cb(addr, 0, false, data);
	}
	return 0;
}

#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 6, 0))
static int gsw1xx_port_max_mtu(struct dsa_switch *ds, int port)
{
	/* Includes 8 bytes for special header. */
	return GSW1XX_MAX_PACKET_LENGTH - VLAN_ETH_HLEN - ETH_FCS_LEN;
}

static int gsw1xx_port_change_mtu(struct dsa_switch *ds, int port, int new_mtu)
{
	struct gsw1xx_priv *priv = ds->priv;
	int cpu_port = get_cpu_port(priv);

	/* CPU port always has maximum mtu of user ports, so use it to set
	 * switch frame size, including 8 byte special header.
	 */
	if (port == cpu_port) {
		new_mtu += 8;
		gsw1xx_switch_write(priv, VLAN_ETH_HLEN + new_mtu + ETH_FCS_LEN,
				    GSW1XX_MAC_FLEN);
	}

	/* Enable MLEN for ports with non-standard MTUs, including the special
	 * header on the CPU port added above.
	 */
	if (new_mtu != ETH_DATA_LEN)
		gsw1xx_modify_switch_reg(priv, 0, GSW1XX_MAC_CTRL_2_MLEN,
					 GSW1XX_MAC_CTRL_2p(port));
	else
		gsw1xx_modify_switch_reg(priv, GSW1XX_MAC_CTRL_2_MLEN, 0,
					 GSW1XX_MAC_CTRL_2p(port));

	return 0;
}
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
static void gsw1xx_phylink_set_capab(unsigned long *supported,
				     struct phylink_link_state *state)
{
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mask) = { 0, };

	/* Allow all the expected bits */
	phylink_set(mask, Autoneg);
	phylink_set_port_modes(mask);
	phylink_set(mask, Asym_Pause);
	phylink_set(mask, Pause);

	if (state->interface == PHY_INTERFACE_MODE_SGMII) {
		phylink_set(mask, 2500baseX_Full);
		phylink_set(mask, 1000baseX_Full);
	}

	if (state->interface != PHY_INTERFACE_MODE_RMII) {
		phylink_set(mask, 1000baseT_Full);
		phylink_set(mask, 1000baseT_Half);
		phylink_set(mask, 10baseT_Half);
		phylink_set(mask, 10baseT_Full);
	}

	/* RMII only supports 100BASE-T */
	phylink_set(mask, 100baseT_Half);
	phylink_set(mask, 100baseT_Full);

	bitmap_and(supported, supported, mask,
		   __ETHTOOL_LINK_MODE_MASK_NBITS);
	bitmap_and(state->advertising, state->advertising, mask,
		   __ETHTOOL_LINK_MODE_MASK_NBITS);
}
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
static void gsw12x_phylink_validate(struct dsa_switch *ds, int port,
				    unsigned long *supported,
				    struct phylink_link_state *state)
{
	switch (port) {
	case 0:
	case 1:
		if (state->interface != PHY_INTERFACE_MODE_INTERNAL)
			goto unsupported;
		break;
	case 2:
	case 3:
		if (state->interface != PHY_INTERFACE_MODE_NA)
			goto unsupported;
		break;
	case 4: /* port 4: SGMII */
		if (state->interface != PHY_INTERFACE_MODE_SGMII)
			goto unsupported;
		break;
	case 5: /* port 5: RGMII or RMII */
		if (!phy_interface_mode_is_rgmii(state->interface) &&
			state->interface != PHY_INTERFACE_MODE_RMII)
			goto unsupported;
		break;
	default:
		bitmap_zero(supported, __ETHTOOL_LINK_MODE_MASK_NBITS);
		dev_err(ds->dev, "Unsupported port: %i\n", port);
		return;
	}

	gsw1xx_phylink_set_capab(supported, state);

return;

unsupported:
	bitmap_zero(supported, __ETHTOOL_LINK_MODE_MASK_NBITS);
	dev_err(ds->dev, "Unsupported interface '%s' for port %d\n",
		phy_modes(state->interface), port);
}
#else
static void gsw12x_phylink_get_caps(struct dsa_switch *ds, int port,
				    struct phylink_config *config)
{
	switch (port) {
	case 0:
	case 1:
		__set_bit(PHY_INTERFACE_MODE_INTERNAL,
			  config->supported_interfaces);
		break;
	case 2:
	case 3:
		__set_bit(PHY_INTERFACE_MODE_NA,
			  config->supported_interfaces);
		break;
	case 4: /* port 4: SGMII */
		__set_bit(PHY_INTERFACE_MODE_SGMII, config->supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_1000BASEX, config->supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_2500BASEX, config->supported_interfaces);
		config->mac_capabilities |= MAC_2500FD;
		break;
	case 5: /* port 5: RGMII or RMII */
		__set_bit(PHY_INTERFACE_MODE_RGMII,
			  config->supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_RMII,
			  config->supported_interfaces);
		phy_interface_set_rgmii(config->supported_interfaces);
		break;
	}

	config->mac_capabilities = MAC_ASYM_PAUSE | MAC_SYM_PAUSE |
				   MAC_10 | MAC_100 | MAC_1000;
}
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
static void gsw14x_phylink_validate(struct dsa_switch *ds, int port,
				    unsigned long *supported,
				    struct phylink_link_state *state)
{
	switch (port) {
	case 0:
	case 1:
	case 2:
	case 3:
		if (state->interface != PHY_INTERFACE_MODE_INTERNAL)
			goto unsupported;
		break;
	case 4: /* port 4: SGMII */
		if (state->interface != PHY_INTERFACE_MODE_SGMII)
			goto unsupported;
		break;
	case 5: /* port 5: RGMII or RMII */
		if (!phy_interface_mode_is_rgmii(state->interface) &&
		    state->interface != PHY_INTERFACE_MODE_RMII)
			goto unsupported;
		break;
	default:
		bitmap_zero(supported, __ETHTOOL_LINK_MODE_MASK_NBITS);
		dev_err(ds->dev, "Unsupported port: %i\n", port);
		return;
	}

	gsw1xx_phylink_set_capab(supported, state);

	return;

unsupported:
	bitmap_zero(supported, __ETHTOOL_LINK_MODE_MASK_NBITS);
	dev_err(ds->dev, "Unsupported interface '%s' for port %d\n",
		phy_modes(state->interface), port);
}
#else
static void gsw14x_phylink_get_caps(struct dsa_switch *ds, int port,
				    struct phylink_config *config)
{
	switch (port) {
	case 0:
	case 1:
	case 2:
	case 3:
		__set_bit(PHY_INTERFACE_MODE_INTERNAL,
			  config->supported_interfaces);
		break;
	case 4: /* port 4: SGMII */
		__set_bit(PHY_INTERFACE_MODE_SGMII, config->supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_1000BASEX, config->supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_2500BASEX, config->supported_interfaces);
		config->mac_capabilities |= MAC_2500FD;
		break;
	case 5: /* port 5: RGMII or RMII */
		__set_bit(PHY_INTERFACE_MODE_RGMII,
			  config->supported_interfaces);
		__set_bit(PHY_INTERFACE_MODE_RMII,
			  config->supported_interfaces);
		phy_interface_set_rgmii(config->supported_interfaces);
		break;
	}

	config->mac_capabilities = MAC_ASYM_PAUSE | MAC_SYM_PAUSE |
		MAC_10 | MAC_100 | MAC_1000;
}
#endif

static void gsw1xx_port_set_link(struct gsw1xx_priv *priv, int port, bool link)
{
	u32 mdio_phy;

	if (link)
		mdio_phy = GSW1XX_MDIO_PHY_LINK_UP;
	else
		mdio_phy = GSW1XX_MDIO_PHY_LINK_DOWN;

	gsw1xx_modify_mdio_master_switch_reg(priv, GSW1XX_MDIO_PHY_LINK_MASK, mdio_phy,
					     GSW1XX_MDIO_PHYp(port));
}

static void gsw1xx_port_set_speed(struct gsw1xx_priv *priv, int port, int speed,
				  phy_interface_t interface)
{
	u32 mdio_phy = 0, mii_cfg = 0, mac_ctrl_0 = 0;

	switch (speed) {
	case SPEED_10:
		mdio_phy = GSW1XX_MDIO_PHY_SPEED_M10;

		if (interface == PHY_INTERFACE_MODE_RMII) {
			dev_err(priv->dev, "RMII does not support 10M for port %d\n", port);
			mii_cfg = GSW1XX_MII_CFG_RATE_M50 | GSW1XX_MII_CFG_MODE_RMIIM;
		} else {
			mii_cfg = GSW1XX_MII_CFG_RATE_M2P5 | GSW1XX_MII_CFG_MODE_RGMII;
		}

		mac_ctrl_0 = GSW1XX_MAC_CTRL_0_GMII_MII;
		break;

	case SPEED_100:
		mdio_phy = GSW1XX_MDIO_PHY_SPEED_M100;

		if (interface == PHY_INTERFACE_MODE_RMII)
			mii_cfg = GSW1XX_MII_CFG_RATE_M50 | GSW1XX_MII_CFG_MODE_RMIIM;
		else
			mii_cfg = GSW1XX_MII_CFG_RATE_M25 | GSW1XX_MII_CFG_MODE_RGMII;

		mac_ctrl_0 = GSW1XX_MAC_CTRL_0_GMII_MII;
		break;

	case SPEED_1000:
		mdio_phy = GSW1XX_MDIO_PHY_SPEED_G1;

		mii_cfg = GSW1XX_MII_CFG_RATE_M125 | GSW1XX_MII_CFG_MODE_RGMII;

		mac_ctrl_0 = GSW1XX_MAC_CTRL_0_GMII_RGMII;

		if (interface == PHY_INTERFACE_MODE_RMII)
			dev_err(priv->dev, "RMII does not support 1G for port %d\n", port);

		if ((priv->dts.use_sgmii == GSW1XX_SGMII_WITH_EXTERNAL_PHY ||
		    priv->dts.use_sgmii == GSW1XX_SGMII_IS_CPU_PORT) &&
		    port == GSW1XX_SGMII_PORT) {
			/* special configuration for SGMII 1G support could be applied here.
			 * otherwise pinstrapping related settings would be used
			 */
			gsw1xx_modify_reg(priv, GSW1XX_SGMII_HSP_MASK | GSW1XX_SGMII_SEL, GSW1XX_SGMII_1G | GSW1XX_SGMII_1G_NCO1,
			  GSW1XX_NCO_CTRL);
		}
		break;
	case SPEED_2500:
		if ((priv->dts.use_sgmii == GSW1XX_SGMII_WITH_EXTERNAL_PHY ||
		    priv->dts.use_sgmii == GSW1XX_SGMII_IS_CPU_PORT) &&
		    port == GSW1XX_SGMII_PORT) {
			/* special configuration for SGMII 2.5G support could be applied here.
			 * otherwise pinstrapping related settings would be used
			 */
			gsw1xx_modify_reg(priv, 0, GSW1XX_SGMII_2G5 | GSW1XX_SGMII_2G5_NCO2,
			  GSW1XX_NCO_CTRL);
			break;
		}
		dev_err(priv->dev, "2.5G only supported for SGMII on port 4, but not on port %d\n", port);
		break;
	default:
		dev_err(priv->dev, "Speed %d not supported for port %d\n", speed, port);
		return;
	}

	gsw1xx_modify_mdio_master_switch_reg(priv, GSW1XX_MDIO_PHY_SPEED_MASK, mdio_phy,
					     GSW1XX_MDIO_PHYp(port));
	gsw1xx_modify_switch_reg(priv, GSW1XX_MAC_CTRL_0_GMII_MASK, mac_ctrl_0,
				 GSW1XX_MAC_CTRL_0p(port));
	/* MII_CFG only works for RGMII/RMII - otherwise ignored */
	gsw1xx_modify_mii_cfg_reg(priv, GSW1XX_MII_CFG_RATE_MASK | GSW1XX_MII_CFG_MODE_MASK,
				  mii_cfg, port);

}

static void gsw1xx_port_set_duplex(struct gsw1xx_priv *priv, int port, int duplex)
{
	u32 mac_ctrl_0, mdio_phy;

	if (duplex == DUPLEX_FULL) {
		mac_ctrl_0 = GSW1XX_MAC_CTRL_0_FDUP_EN;
		mdio_phy = GSW1XX_MDIO_PHY_FDUP_EN;
	} else {
		mac_ctrl_0 = GSW1XX_MAC_CTRL_0_FDUP_DIS;
		mdio_phy = GSW1XX_MDIO_PHY_FDUP_DIS;
	}

	gsw1xx_modify_switch_reg(priv, GSW1XX_MAC_CTRL_0_FDUP_MASK, mac_ctrl_0,
				 GSW1XX_MAC_CTRL_0p(port));
	gsw1xx_modify_mdio_master_switch_reg(priv, GSW1XX_MDIO_PHY_FDUP_MASK, mdio_phy,
					     GSW1XX_MDIO_PHYp(port));
}

static void gsw1xx_port_set_pause(struct gsw1xx_priv *priv, int port,
				  bool tx_pause, bool rx_pause)
{
	u32 mac_ctrl_0, mdio_phy;

	if (tx_pause && rx_pause) {
		mac_ctrl_0 = GSW1XX_MAC_CTRL_0_FCON_RXTX;
		mdio_phy = GSW1XX_MDIO_PHY_FCONTX_EN |
			   GSW1XX_MDIO_PHY_FCONRX_EN;
	} else if (tx_pause) {
		mac_ctrl_0 = GSW1XX_MAC_CTRL_0_FCON_TX;
		mdio_phy = GSW1XX_MDIO_PHY_FCONTX_EN |
			   GSW1XX_MDIO_PHY_FCONRX_DIS;
	} else if (rx_pause) {
		mac_ctrl_0 = GSW1XX_MAC_CTRL_0_FCON_RX;
		mdio_phy = GSW1XX_MDIO_PHY_FCONTX_DIS |
			   GSW1XX_MDIO_PHY_FCONRX_EN;
	} else {
		mac_ctrl_0 = GSW1XX_MAC_CTRL_0_FCON_NONE;
		mdio_phy = GSW1XX_MDIO_PHY_FCONTX_DIS |
			   GSW1XX_MDIO_PHY_FCONRX_DIS;
	}

	gsw1xx_modify_switch_reg(priv, GSW1XX_MAC_CTRL_0_FCON_MASK,
				 mac_ctrl_0, GSW1XX_MAC_CTRL_0p(port));
	gsw1xx_modify_mdio_master_switch_reg(priv,
					     GSW1XX_MDIO_PHY_FCONTX_MASK |
					     GSW1XX_MDIO_PHY_FCONRX_MASK,
					     mdio_phy, GSW1XX_MDIO_PHYp(port));
}

static void gsw1xx_phylink_mac_config(struct dsa_switch *ds, int port,
				      unsigned int mode,
				      const struct phylink_link_state *state)
{
	struct gsw1xx_priv *priv = ds->priv;
	u32 miicfg = 0;

	if (priv->dts.auto_ldclk_disable == 1)
		miicfg |= GSW1XX_MII_CFG_LDCLKDIS;
	switch (state->interface) {
	case PHY_INTERFACE_MODE_INTERNAL:
		return;
	case PHY_INTERFACE_MODE_SGMII:
		return;
	case PHY_INTERFACE_MODE_RMII:
		miicfg |= GSW1XX_MII_CFG_MODE_RMIIM;
		/* Configure the RMII clock as output depending on customer setting */
		if (priv->dts.rmii_clk_custom)
			miicfg |= GSW1XX_MII_CFG_RMII_CLK;
		break;
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		miicfg |= GSW1XX_MII_CFG_MODE_RGMII;
		break;
	default:
		dev_err(ds->dev,
			"Unsupported interface: %d\n", state->interface);
		return;
	}
	/* MII_CFG only works for RGMII/RMII port - otherwise ignored */
	gsw1xx_modify_mii_cfg_reg(priv,
				  GSW1XX_MII_CFG_MODE_MASK | GSW1XX_MII_CFG_RMII_CLK |
				  GSW1XX_MII_CFG_RGMII_IBS | GSW1XX_MII_CFG_LDCLKDIS,
				  miicfg, port);

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(5, 6, 0))
	gsw1xx_port_set_speed(priv, port, state->speed, state->interface);
	gsw1xx_port_set_duplex(priv, port, state->duplex);
	gsw1xx_port_set_pause(priv, port, !!(state->pause & MLO_PAUSE_TX),
			      !!(state->pause & MLO_PAUSE_RX));
#endif

	/* configure interface delays */
	switch (state->interface) {
	case PHY_INTERFACE_MODE_RGMII_ID:
		miicfg = (priv->dts.tx_mii_delay & GSW1XX_MII_PCDU_TXDLY_MASK) |
			 (priv->dts.rx_mii_delay & GSW1XX_MII_PCDU_RXDLY_MASK);
		gsw1xx_modify_pcdu_reg(priv, GSW1XX_MII_PCDU_TXDLY_MASK |
				       GSW1XX_MII_PCDU_RXDLY_MASK, 0, port);
		gsw1xx_modify_pcdu_reg(priv, 0, miicfg, port);
		miicfg = gsw1xx_read_reg(ds->priv, RGMII_BASE + MII_PCDU5_OFFS);
		break;
	case PHY_INTERFACE_MODE_RGMII_RXID:
		miicfg = (priv->dts.rx_mii_delay & GSW1XX_MII_PCDU_RXDLY_MASK);
		gsw1xx_modify_pcdu_reg(priv, GSW1XX_MII_PCDU_RXDLY_MASK, 0, port);
		gsw1xx_modify_pcdu_reg(priv, 0, miicfg, port);
		break;
	case PHY_INTERFACE_MODE_RGMII_TXID:
		miicfg = (priv->dts.tx_mii_delay & GSW1XX_MII_PCDU_TXDLY_MASK);
		gsw1xx_modify_pcdu_reg(priv, GSW1XX_MII_PCDU_TXDLY_MASK, 0, port);
		gsw1xx_modify_pcdu_reg(priv, 0, miicfg, port);
		break;
	default:
		break;
	}
}

static void gsw1xx_phylink_mac_link_down(struct dsa_switch *ds, int port,
					 unsigned int mode, phy_interface_t interface)
{
	struct gsw1xx_priv *priv = ds->priv;

	/* MII_CFG only works for RGMII/RMII port */
	gsw1xx_modify_mii_cfg_reg(priv, GSW1XX_MII_CFG_EN, 0, port);

	if (!dsa_is_cpu_port(ds, port))
		gsw1xx_port_set_link(priv, port, false);
}

#ifdef GSW1XX_CHECK_PHYRXERR
static void gsw1xx_check_phy_rxerr(struct dsa_switch *ds, int port)
{
	struct gsw1xx_priv *priv = ds->priv;
	int ret;

	if (port <= priv->hw_info->phy_ports) {
		/* check for phy rxerr after link up and trigger re-aneg in this situation */
		ret = gsw1xx_phy_read(ds, port, 0x15);
		if (ret == 0xFF) {
			/* trigger re-aneg via ANRS in case RXERR is saturated immediately
			 * after link-up
			 */
			ret = gsw1xx_phy_read(ds, port, 0);
			ret |= BIT(9);
			gsw1xx_phy_write(ds, port, 0, ret);
		}
	}
}
#endif

#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 6, 0))
static void gsw1xx_phylink_mac_link_up(struct dsa_switch *ds, int port,
				       unsigned int mode,
				       phy_interface_t interface,
				       struct phy_device *phydev,
				       int speed, int duplex,
				       bool tx_pause, bool rx_pause)
{
	struct gsw1xx_priv *priv = ds->priv;

	gsw1xx_port_set_link(priv, port, true);
	gsw1xx_port_set_speed(priv, port, speed, interface);
	gsw1xx_port_set_duplex(priv, port, duplex);
	gsw1xx_port_set_pause(priv, port, tx_pause, rx_pause);

	/* MII_CFG only works for RGMII/RMII port */
	gsw1xx_modify_mii_cfg_reg(priv, 0, GSW1XX_MII_CFG_EN, port);

#ifdef GSW1XX_CHECK_PHYRXERR
	/* check for phy rxerr after link up */
	gsw1xx_check_phy_rxerr(ds, port);
#endif
}
#else
static void gsw1xx_phylink_mac_link_up(struct dsa_switch *ds, int port,
				       unsigned int mode,
				       phy_interface_t interface,
				       struct phy_device *phydev)
{
	struct gsw1xx_priv *priv = ds->priv;

	if (!dsa_is_cpu_port(ds, port))
		gsw1xx_port_set_link(priv, port, true);
	/* MII_CFG only works for RGMII/RMII port */
	gsw1xx_modify_mii_cfg_reg(priv, 0, GSW1XX_MII_CFG_EN, port);

#ifdef GSW1XX_CHECK_PHYRXERR
	/* check for phy rxerr after link up */
	gsw1xx_check_phy_rxerr(ds, port);
#endif
}
#endif

static int gsw1xx_get_mac_eee(struct dsa_switch *ds, int port,
			      struct ethtool_eee *e)
{
	struct gsw1xx_priv *priv = (struct gsw1xx_priv *)ds->priv;
	int val;

	val = gsw1xx_read_switch_reg(priv, GSW1XX_MAC_CTRL_4p(port));
	e->tx_lpi_enabled = !!(val & GSW1XX_MAC_CTRL_4_LPIEN);
	e->tx_lpi_timer = (val & GSW1XX_MAC_CTRL_4_GWAIT_MASK) >> GSW1XX_MAC_CTRL_4_GWAIT;

	return 0;
}

static int gsw1xx_set_mac_eee(struct dsa_switch *ds, int port,
			      struct ethtool_eee *e)
{
	struct gsw1xx_priv *priv = (struct gsw1xx_priv *)ds->priv;

	if (e->tx_lpi_enabled) {
		gsw1xx_modify_switch_reg(priv, 0, GSW1XX_MAC_CTRL_4_LPIEN,
					 GSW1XX_MAC_CTRL_4p(port));
	} else {
		gsw1xx_modify_switch_reg(priv, GSW1XX_MAC_CTRL_4_LPIEN, 0,
					 GSW1XX_MAC_CTRL_4p(port));
	}
	gsw1xx_modify_switch_reg(priv, GSW1XX_MAC_CTRL_4_GWAIT_MASK,
				 ((e->tx_lpi_timer << GSW1XX_MAC_CTRL_4_GWAIT)
				 & GSW1XX_MAC_CTRL_4_GWAIT_MASK), GSW1XX_MAC_CTRL_4p(port));

	return 0;
}

static const struct dsa_switch_ops gsw12x_switch_ops = {
	.get_ethtool_stats	= gsw1xx_get_ethtool_stats,
	.get_strings		= gsw1xx_get_strings,
	.get_sset_count		= gsw1xx_get_sset_count,
	.get_tag_protocol	= gsw1xx_get_tag_protocol,
	.phy_read		= gsw1xx_phy_read,
	.phy_write		= gsw1xx_phy_write,
	.phylink_mac_config	= gsw1xx_phylink_mac_config,
	.phylink_mac_link_down	= gsw1xx_phylink_mac_link_down,
	.phylink_mac_link_up	= gsw1xx_phylink_mac_link_up,
	.set_mac_eee		= gsw1xx_set_mac_eee,
	.get_mac_eee		= gsw1xx_get_mac_eee,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
	.phylink_validate	= gsw12x_phylink_validate,
#else
	.phylink_get_caps		= gsw12x_phylink_get_caps,
#endif
	.port_bridge_join	= gsw1xx_port_bridge_join,
	.port_bridge_leave	= gsw1xx_port_bridge_leave,
	.port_disable		= gsw1xx_port_disable,
	.port_enable		= gsw1xx_port_enable,
	.port_fast_age		= gsw1xx_port_fast_age,
	.port_fdb_add		= gsw1xx_port_fdb_add,
	.port_fdb_del		= gsw1xx_port_fdb_del,
	.port_fdb_dump		= gsw1xx_port_fdb_dump,
#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 6, 0))
	.port_change_mtu	= gsw1xx_port_change_mtu,
	.port_max_mtu		= gsw1xx_port_max_mtu,
#endif
	.port_stp_state_set	= gsw1xx_port_stp_state_set,
	.port_vlan_filtering	= gsw1xx_port_vlan_filtering,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 12, 0))
	.port_vlan_prepare	= gsw1xx_port_vlan_prepare,
#endif
	.port_vlan_add		= gsw1xx_port_vlan_add,
	.port_vlan_del		= gsw1xx_port_vlan_del,
	.setup			= gsw1xx_setup,
};

static const struct dsa_switch_ops gsw14x_switch_ops = {
	.get_ethtool_stats	= gsw1xx_get_ethtool_stats,
	.get_strings		= gsw1xx_get_strings,
	.get_sset_count		= gsw1xx_get_sset_count,
	.get_tag_protocol	= gsw1xx_get_tag_protocol,
	.phy_read		= gsw1xx_phy_read,
	.phy_write		= gsw1xx_phy_write,
	.phylink_mac_config	= gsw1xx_phylink_mac_config,
	.phylink_mac_link_down	= gsw1xx_phylink_mac_link_down,
	.phylink_mac_link_up	= gsw1xx_phylink_mac_link_up,
	.set_mac_eee		= gsw1xx_set_mac_eee,
	.get_mac_eee		= gsw1xx_get_mac_eee,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 17, 0))
	.phylink_validate	= gsw14x_phylink_validate,
#else
	.phylink_get_caps	= gsw14x_phylink_get_caps,
#endif
	.port_bridge_join	= gsw1xx_port_bridge_join,
	.port_bridge_leave	= gsw1xx_port_bridge_leave,
	.port_disable		= gsw1xx_port_disable,
	.port_enable		= gsw1xx_port_enable,
	.port_fast_age		= gsw1xx_port_fast_age,
	.port_fdb_add		= gsw1xx_port_fdb_add,
	.port_fdb_del		= gsw1xx_port_fdb_del,
	.port_fdb_dump		= gsw1xx_port_fdb_dump,
#if (LINUX_VERSION_CODE > KERNEL_VERSION(5, 6, 0))
	.port_change_mtu	= gsw1xx_port_change_mtu,
	.port_max_mtu		= gsw1xx_port_max_mtu,
#endif
	.port_stp_state_set	= gsw1xx_port_stp_state_set,
	.port_vlan_filtering	= gsw1xx_port_vlan_filtering,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 12, 0))
	.port_vlan_prepare	= gsw1xx_port_vlan_prepare,
#endif
	.port_vlan_add		= gsw1xx_port_vlan_add,
	.port_vlan_del		= gsw1xx_port_vlan_del,
	.setup			= gsw1xx_setup,
};

static int gsw1xx_probe(struct mdio_device *mdiodev)
{
	struct device *dev = &mdiodev->dev;
	struct gsw1xx_priv *priv;
	struct dsa_switch *ds;
	u32 version;
	int ret;
	int i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		dev_err(dev, "%s: Error allocating gsw1xx chip\n", __func__);
		return -ENOMEM;
	}

	priv->dev = dev;
	priv->bus = mdiodev->bus;
	priv->sw_addr = mdiodev->addr;
	priv->hw_info = of_device_get_match_data(dev);
	if (!priv->hw_info)
		return -EINVAL;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 16, 0))
	mutex_init(&priv->pce_table_lock);
#endif

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 5, 0))
	ds = devm_kzalloc(dev, sizeof(*ds), GFP_KERNEL);
#else
	ds = dsa_switch_alloc(dev, GSW1xx_PORTS);
#endif
	if (!ds) {
		dev_err(dev, "%s: Error allocating DSA switch\n", __func__);
		return -ENOMEM;
	}

	ds->dev = dev;
	ds->num_ports = GSW1xx_PORTS;
	ds->priv = priv;
	ds->ops = priv->hw_info->ops;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
	ds->fdb_isolation = true;
#endif

	dev_set_drvdata(dev, ds);

	ret = dsa_register_switch(ds);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "%s: Error %d register DSA switch\n", __func__, ret);
		return ret;
	}

	/* preconfigure LEDs setting for internal PHY ports in case generic PHY driver is used.
	 * Will be overwritten by intel_xway.c phy driver and will be applied in
	 * gsw1xx_phylink_mac_link_down
	 */
	if (priv->dts.led_settings == 1) {
		usleep_range(2000, 4000);
		for (i = 0; i < priv->hw_info->phy_ports; i++) {
			if (dsa_is_user_port(ds, i))
				gsw1xx_led_cfg(ds, i);
		}
	}

	version = gsw1xx_read_switch_reg(priv, GSW1XX_VERSION);

	dev_info(dev, "%s: GSW1xx Switch Detected - swapi version: 0x%x, max ports %d, internal phy ports %d, cpu_port: %d, SGMII mode: %d\n", __func__,
		version, priv->hw_info->max_ports,  priv->hw_info->phy_ports,
		get_cpu_port(priv), priv->dts.use_sgmii);

	if (!dsa_is_cpu_port(ds, get_cpu_port(priv))) {
		dev_err(dev, "wrong CPU port defined, HW only supports port: %i",
			get_cpu_port(priv));
		ret = -EINVAL;
		goto disable_switch;
	}
	return ret;
disable_switch:
	dsa_unregister_switch(ds);

	return ret;
}

static void gsw1xx_remove(struct mdio_device *mdiodev)
{
	struct dsa_switch *ds = dev_get_drvdata(&mdiodev->dev);

	dsa_unregister_switch(ds);
}

static const struct gsw1xx_hw_info gsw12x_data = {
	.max_ports = 6,
	.phy_ports = 2,
	.ops = &gsw12x_switch_ops,
};

static const struct gsw1xx_hw_info gsw14x_data = {
	.max_ports = 6,
	.phy_ports = 4,
	.ops = &gsw14x_switch_ops,
};

static const struct of_device_id gsw1xx_of_match[] = {
	{ .compatible = "maxlinear,gsw120", .data = &gsw12x_data },
	{ .compatible = "maxlinear,gsw125", .data = &gsw12x_data },
	{ .compatible = "maxlinear,gsw12x", .data = &gsw12x_data },
	{ .compatible = "maxlinear,gsw140", .data = &gsw14x_data },
	{ .compatible = "maxlinear,gsw141", .data = &gsw14x_data },
	{ .compatible = "maxlinear,gsw145", .data = &gsw14x_data },
	{ .compatible = "maxlinear,gsw14x", .data = &gsw14x_data },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, gsw1xx_of_match);

static struct mdio_driver gsw1xx_driver = {
	.probe	= gsw1xx_probe,
	.remove = gsw1xx_remove,
	.mdiodrv.driver = {
	.name = "mxl-gsw1xx",
	.of_match_table = gsw1xx_of_match,
	},
};

mdio_module_driver(gsw1xx_driver);

MODULE_DESCRIPTION("Driver for MaxLinear gsw1xx ethernet switch");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:mxl-gsw1xx");

/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * USB ethernet driver for USB2.0 to 100Mbps ethernet chip ch336/ch339/ch396/ch397.
 *
 * Copyright (C) 2025 Nanjing Qinheng Microelectronics Co., Ltd.
 * 				http://wch.cn
 * Author:   	WCH <tech@wch.cn>
 * Contributor: Sergey Kharenko <skharenko@hust.edu.cn>
 */

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/usb/usbnet.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/if_vlan.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/crc32.h>

/* control requests */
#define CH397_USB_GET_INFO   			0x10
#define CH397_USB_RD_REG     			0x11
#define CH397_USB_WR_REG     			0x12
#define CH397_USB_RD_OTP     			0x13
#define CH397_USB_WR_OTP     			0x14
#define CH397_USB_RD_PHY     			0x15
#define CH397_USB_WR_PHY     			0x16
#define CH397_RESET_CMD	     			0x18
#define CH397_WR_ETH_MACCFG  			0x1E
#define CH397_WR_ETH_AUTONEG 			0x1F

#define CH397_MAX_MCAST					12

#define CH397_TX_OVERHEAD 				8
#define CH397_RX_OVERHEAD 				8

#define CH397_ETH_MAC_CFG  				0x40000700
#define CH397_ETH_MAC_H	   				0x40000710
#define CH397_ETH_MAC_L	   				0x40000714
#define CH397_ETH_BMSR	   				0x00000740
#define CH397_ETH_MAC_HTHR 				0x40000730
#define CH397_ETH_MAC_HTLR 				0x40000734

#define CH397_LINK_STATUS 				(1 << 6)

#define CH397_MEDIUM_PS	  				(1 << 7)
#define CH397_MEDIUM_FD	  				(1 << 8)
#define CH397_FLOWCTRL_EN 				(1 << 11)
#define CH397_TX_CTRL_EN  				(1 << 13)
#define CH397_RX_CTRL_EN  				(1 << 14)
#define CH397_RX_CTRL_PLM 				(1 << 23)
#define CH397_RX_CTRL_PBD 				(1 << 22)
#define CH397_RX_CTRL_PUF 				(1 << 21)
#define CH397_RX_CTRL_PAM 				(1 << 20)
#define CH397_RX_CTRL_RA  				(1 << 19)

#define CH397_USB_DELAY	 				10
#define CH397_MT_DELAY	 				10
#define CH397_WORK_DELAY 				100

#define ETH_HEADER_SIZE 				14 /* size of ethernet header */

#define ETH_MIN_DATA_SIZE   			46 /* minimum eth data size */
#define ETH_MIN_PACKET_SIZE 			(ETH_HEADER_SIZE + ETH_MIN_DATA_SIZE)
			
#define ETH_DEF_DATA_SIZE   			1500 /* default data size */
#define ETH_DEF_PACKET_SIZE 			(ETH_HEADER_SIZE + ETH_DEF_DATA_SIZE)

/* ch397 flags */
enum ch397_flags {
	CH397_SET_RX_MF = 0,
	CH397_SET_RX_MODE,
	CH397_LINK_CHG,
	CH397_CHECK_MAC,
};

struct ch397_int_data {
	u8 link;
	__le16 res1;
	__le16 res2;
	u8 status;
	__le16 res3;
} __packed;

typedef enum {
	STATE_S1 = 0,
	STATE_S2,
	STATE_S3,
} StateType;

struct ch397_chip_info {
	u8 chiptype;
	u8 hwver;
	u8 fwver;
	u8 sta;
	u8 reserved[4];
} __packed;

struct ch397_rx_fixup_info {
	struct sk_buff *ch397_skb;
	u32 header;
	u32 remaining;
	u32 remaining_header;
	u32 remaining_pad;
	u32 size;
	bool split_head;
	StateType state;
};

/* This structure cannot exceed sizeof(unsigned long [5]) AKA 20 bytes */
struct ch397_mac_cfg {
	u32 multi_filter[2];
};

struct ch397_common_private {
	struct usbnet *dev;
	struct delayed_work schedule_work;
	struct workqueue_struct *wq;
	unsigned long flags;
	u16 speed;
	u8 *intr_buff;
	u8 duplex;
	u8 autoneg;
	__le16 crc_num;
	__le32 presvd_mac_mask_set;
	__le32 presvd_mac_mask_clear;
	__le32 presvd_mac_cfg;
	struct ch397_chip_info chip_info;
	struct ch397_mac_cfg mac_cfg;
	struct ch397_rx_fixup_info rx_fixup_info;
};

static int ch397_read(struct usbnet *dev, u8 cmd, u32 reg, u16 length, void *data)
{
	int err, i;
	u16 value = (u16)(reg & 0xFFFF);
	u16 index = (u16)((reg >> 16) & 0xFFFF);

	err = usbnet_read_cmd(dev, cmd, USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE, value, index, data, length);
	if (err != length && err >= 0)
		err = -EINVAL;

	dev_dbg(&dev->intf->dev, "ch397_read() cmd=0x%02x, reg=0x%08x, read=\n", cmd, reg);
	for (i = 0; i < length; i++)
		dev_dbg(&dev->intf->dev, "\t0x%2x\n", *((u8 *)data + i));

	msleep(CH397_USB_DELAY);

	return err;
}

static int ch397_write(struct usbnet *dev, u8 cmd, u32 reg, u16 length, const void *data)
{
	int err, i;
	u16 value = (u16)(reg & 0xFFFF);
	u16 index = (u16)((reg >> 16) & 0xFFFF);

	err = usbnet_write_cmd(dev, cmd, USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE, value, index, data, length);
	if (err >= 0 && err < length)
		err = -EINVAL;

	dev_dbg(&dev->intf->dev, "ch397_write() cmd=0x%02x, reg=0x%08x, write=\n", cmd, reg);
	for (i = 0; i < length; i++)
		dev_dbg(&dev->intf->dev, "\t0x%2x\n", *((u8 *)data + i));

	msleep(CH397_USB_DELAY);

	return err;
}

static int ch397_read_shared_word(struct usbnet *dev, int phy, u8 reg, __le16 *value)
{
	int err;

	err = ch397_read(dev, CH397_USB_RD_PHY, CH397_ETH_BMSR | (reg << 16), 2, value);
	if (err < 0) {
		printk(KERN_ERR "Error reading phy reg, phy: 0x%x, reg: 0x%x.\n", phy, reg);
		return err;
	}

	return err;
}

static int ch397_write_shared_word(struct usbnet *dev, int phy, u8 reg, __le16 *value)
{
	int err;

	err = ch397_write(dev, CH397_USB_WR_PHY, CH397_ETH_BMSR | (reg << 16), 2, value);
	if (err < 0) {
		printk(KERN_ERR "Error writing phy reg, phy: 0x%x, reg: 0x%x.\n", phy, reg);
		return err;
	}

	return err;
}

static int ch397_mdio_read(struct net_device *net, int phy_id, int loc)
{
	struct usbnet *dev = netdev_priv(net);
	__le16 res;

	if (phy_id) {
		netdev_dbg(dev->net, "Only internal phy supported\n");
		return 0;
	}

	ch397_read_shared_word(dev, 1, loc, &res);

	netdev_dbg(dev->net, "ch397_mdio_read() phy_id=0x%02x, loc=0x%02x, returns=0x%04x\n", phy_id, loc,
		   le16_to_cpu(res));

	return le16_to_cpu(res);
}

static void ch397_mdio_write(struct net_device *net, int phy_id, int loc, int val)
{
	struct usbnet *dev = netdev_priv(net);
	struct ch397_common_private *dp = dev->driver_priv;
	__le16 res = cpu_to_le16(val);

	if (phy_id) {
		netdev_dbg(dev->net, "Only internal phy supported\n");
		return;
	}

	if (dp->chip_info.fwver >= 0x37) {
		if ((res & ~(BMCR_SPEED100 | BMCR_ANENABLE | BMCR_ANRESTART | BMCR_FULLDPLX)) == 0x00) {
			if ((res & (BMCR_SPEED100 | BMCR_FULLDPLX)) == (BMCR_SPEED100 | BMCR_FULLDPLX)) {
				return;
			}
		}
	}

	ch397_write_shared_word(dev, 1, loc, &res);

	netdev_dbg(dev->net, "ch397_mdio_write() phy_id=0x%02x, loc=0x%02x, val=0x%04x\n", phy_id, loc, val);
}

static void ch397_get_drvinfo(struct net_device *net, struct ethtool_drvinfo *info)
{
	/* Inherit standard device info */
	usbnet_get_drvinfo(net, info);
}

static u32 ch397_get_link(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);

	return mii_link_ok(&dev->mii);
}

static int ch397_ioctl(struct net_device *net, struct ifreq *rq, int cmd)
{
	struct usbnet *dev = netdev_priv(net);

	return generic_mii_ioctl(&dev->mii, if_mii(rq), cmd, NULL);
}

static int ch397_set_autoneg(struct usbnet *dev, bool autoneg)
{
	int err;

	if (autoneg)
		err = ch397_write(dev, CH397_WR_ETH_AUTONEG, 0xFA00, 0, NULL);
	else
		err = ch397_write(dev, CH397_WR_ETH_AUTONEG, 0xFA01, 0, NULL);

	return err;
}

static int ch397_set_settings(struct net_device *net, const struct ethtool_link_ksettings *cmd)
{
	struct usbnet *dev = netdev_priv(net);
	struct ch397_common_private *dp = dev->driver_priv;
	int ret;

	ret = usbnet_set_link_ksettings_mii(net, cmd);
	if (!ret) {
		dp->autoneg = cmd->base.autoneg;
		dp->speed = cmd->base.speed;
		dp->duplex = cmd->base.duplex;
	}

	if ((dp->chip_info.fwver >= 0x37) && (dp->speed == SPEED_100) && (dp->duplex == DUPLEX_FULL)) {
		if (!dp->autoneg)
			ch397_set_autoneg(dev, false);
		else
			ch397_set_autoneg(dev, true);
	}

	return ret;
}

static const struct ethtool_ops ch397_ethtool_ops = {
	.get_drvinfo = ch397_get_drvinfo,
	.get_link = ch397_get_link,
	.get_msglevel = usbnet_get_msglevel,
	.set_msglevel = usbnet_set_msglevel,
	.nway_reset = usbnet_nway_reset,
	.get_link_ksettings = usbnet_get_link_ksettings_mii,
	.set_link_ksettings = ch397_set_settings,
};

static int ch397_get_mac_address(struct usbnet *dev, void *mac_addr)
{
	int err;
	u8 buf[4];

	memset(buf, 0, sizeof(buf));
	err = ch397_read(dev, CH397_USB_RD_REG, CH397_ETH_MAC_L, 4, buf);
	if (err < 0) {
		netdev_err(dev->net, "Error getting MAC low address.\n");
		return err;
	}
	memcpy(mac_addr, buf, 4);

	memset(buf, 0, sizeof(buf));
	err = ch397_read(dev, CH397_USB_RD_REG, CH397_ETH_MAC_H, 2, buf);
	if (err < 0) {
		netdev_err(dev->net, "Error getting MAC high address.\n");
		return err;
	}

	memcpy(mac_addr + 4, buf, 2);

	return 0;
}

static int ch397_get_info(struct usbnet *dev, void *chip_info)
{
	int err;
	u8 buf[8];

	memset(buf, 0, sizeof(buf));
	err = ch397_read(dev, CH397_USB_GET_INFO, 0x00, 8, buf);
	if (err < 0) {
		netdev_err(dev->net, "Error getting chip info.\n");
		return err;
	}
	memcpy(chip_info, buf, 8);

	return 0;
}

static void __ch397_set_mac_address(struct usbnet *dev)
{
	const u8 *dev_addr = dev->net->dev_addr;

	ch397_write(dev, CH397_USB_WR_REG, CH397_ETH_MAC_L, 4, dev_addr);
	ch397_write(dev, CH397_USB_WR_REG, CH397_ETH_MAC_H, 2, dev_addr + 4);
}

static int ch397_set_mac_address(struct net_device *net, void *p)
{
	struct sockaddr *addr = p;
	struct usbnet *dev = netdev_priv(net);

	if (!is_valid_ether_addr(addr->sa_data)) {
		dev_err(&net->dev, "not setting invalid mac address %pM\n", addr->sa_data);
		return -EINVAL;
	}

	eth_hw_addr_set(net, addr->sa_data);
	__ch397_set_mac_address(dev);

	return 0;
}

static void ch397_set_multicast(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	struct ch397_common_private *dp = dev->driver_priv;
	__le32 val_set = 0, val_clear = 0;
	struct netdev_hw_addr *ha;
	u32 crc_bits;

	if (net->flags & IFF_PROMISC) {
		netdev_dbg(dev->net, "ch397 promiscuous mode enabled\n");
		val_clear |= (CH397_RX_CTRL_PBD | CH397_RX_CTRL_PUF | CH397_RX_CTRL_PAM);
		val_set |= CH397_RX_CTRL_RA;
	} else {
		netdev_dbg(dev->net, "ch397 promiscuous mode disabled\n");
		val_set |= CH397_RX_CTRL_PBD | CH397_RX_CTRL_PUF;
		val_clear |= CH397_RX_CTRL_RA;
	}

	if (net->flags & IFF_ALLMULTI || netdev_mc_count(net) > CH397_MAX_MCAST) {
		/* Too many to filter perfectly -- accept all multicasts. */
		val_set |= CH397_RX_CTRL_PAM;
	}

	dp->mac_cfg.multi_filter[1] = 0xffffffff;
	dp->mac_cfg.multi_filter[0] = 0xffffffff;
	if (net->flags & IFF_PROMISC) {
	} else if (net->flags & IFF_ALLMULTI || netdev_mc_count(net) > CH397_MAX_MCAST) {
	} else if (netdev_mc_empty(net)) {
		/* just broadcast and directed */
	} else {
		/* Build the multicast hash filter. */
		dp->mac_cfg.multi_filter[1] = 0;
		dp->mac_cfg.multi_filter[0] = 0;

		netdev_for_each_mc_addr(ha, net) {
			crc_bits = crc32_le(~0, ha->addr, ETH_ALEN);
			crc_bits = bitrev32((~crc_bits)) >> 26;
			dp->mac_cfg.multi_filter[crc_bits >> 5] |= 1 << (crc_bits & 31);
		}
		val_set |= CH397_RX_CTRL_PLM;
	}

	dp->presvd_mac_mask_set = val_set;
	dp->presvd_mac_mask_clear = val_clear;

	set_bit(CH397_SET_RX_MF, &dp->flags);
	set_bit(CH397_SET_RX_MODE, &dp->flags);
	queue_delayed_work(dp->wq, &dp->schedule_work, msecs_to_jiffies(CH397_MT_DELAY));
}

static const struct net_device_ops ch397_netdev_ops = {
	.ndo_open = usbnet_open,
	.ndo_stop = usbnet_stop,
	.ndo_start_xmit = usbnet_start_xmit,
	.ndo_tx_timeout = usbnet_tx_timeout,
	.ndo_change_mtu = usbnet_change_mtu,
	.ndo_get_stats64 = dev_get_tstats64,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_eth_ioctl = ch397_ioctl,
	.ndo_set_mac_address = ch397_set_mac_address,
	.ndo_set_rx_mode = ch397_set_multicast,
};

static int ch397_set_speed(struct ch397_common_private *dp, u8 autoneg, u16 speed, u8 duplex)
{
	u16 bmcr, anar;
	int ret = 0;

	anar = ch397_mdio_read(dp->dev->net, dp->dev->mii.phy_id, MII_ADVERTISE);
	anar &= ~(ADVERTISE_10HALF | ADVERTISE_10FULL | ADVERTISE_100HALF | ADVERTISE_100FULL);

	if (autoneg == AUTONEG_DISABLE) {
		if (speed == SPEED_10) {
			bmcr = 0;
			anar |= ADVERTISE_10HALF | ADVERTISE_10FULL;
		} else if (speed == SPEED_100) {
			bmcr = BMCR_SPEED100;
			anar |= ADVERTISE_100HALF | ADVERTISE_100FULL;
		} else {
			ret = -EINVAL;
			goto out;
		}
		if (duplex == DUPLEX_FULL)
			bmcr |= BMCR_FULLDPLX;
	} else {
		if (speed == SPEED_10) {
			if (duplex == DUPLEX_FULL)
				anar |= ADVERTISE_10HALF | ADVERTISE_10FULL;
			else
				anar |= ADVERTISE_10HALF;
		} else if (speed == SPEED_100) {
			if (duplex == DUPLEX_FULL) {
				anar |= ADVERTISE_10HALF | ADVERTISE_10FULL;
				anar |= ADVERTISE_100HALF | ADVERTISE_100FULL;
			} else {
				anar |= ADVERTISE_10HALF;
				anar |= ADVERTISE_100HALF;
			}
		} else {
			ret = -EINVAL;
			goto out;
		}
		bmcr = BMCR_ANENABLE | BMCR_ANRESTART;
	}

	ch397_mdio_write(dp->dev->net, dp->dev->mii.phy_id, MII_ADVERTISE, anar);
	ch397_mdio_write(dp->dev->net, dp->dev->mii.phy_id, MII_BMCR, bmcr);

out:
	return ret;
}

static __le32 swap_bytes(__le32 value) {
    __le32 byte1 = (value & 0xFFFF0000) >> 16;
    __le32 byte2 = (value & 0x0000FFFF) << 16;

    return byte1 | byte2;
}

static void work_func(struct work_struct *work)
{
	struct ch397_common_private *dp = container_of(work, struct ch397_common_private, schedule_work.work);
	__le32 value, hashval[2];
	int err;

	if (test_and_clear_bit(CH397_SET_RX_MF, &dp->flags)) {
		hashval[0] = swap_bytes(dp->mac_cfg.multi_filter[1]);
		hashval[1] = swap_bytes(dp->mac_cfg.multi_filter[0]);

		err = ch397_write(dp->dev, CH397_USB_WR_REG, CH397_ETH_MAC_HTHR, 4, &hashval[0]);
		if (err < 0) {
			netdev_err(dp->dev->net, "%s, Error setting mac hash high reg.\n", __func__);
			return;
		}

		err = ch397_write(dp->dev, CH397_USB_WR_REG, CH397_ETH_MAC_HTLR, 4, &hashval[1]);
		if (err < 0) {
			netdev_err(dp->dev->net, "%s, Error setting mac hash low reg.\n", __func__);
			return;
		}
	}

	if (test_and_clear_bit(CH397_SET_RX_MODE, &dp->flags) || test_and_clear_bit(CH397_CHECK_MAC, &dp->flags)) {
		if (ch397_read(dp->dev, CH397_USB_RD_REG, CH397_ETH_MAC_CFG, 4, &value) < 0) {
			netdev_err(dp->dev->net, "%s, Error setting mac configure value.\n", __func__);
			return;
		}
		dp->presvd_mac_cfg = (value | dp->presvd_mac_mask_set) & ~dp->presvd_mac_mask_clear;

		if (test_and_clear_bit(CH397_CHECK_MAC, &dp->flags)) {
			if (!(dp->presvd_mac_cfg & CH397_RX_CTRL_EN) || !(dp->presvd_mac_cfg & CH397_TX_CTRL_EN))
				dp->presvd_mac_cfg |= CH397_RX_CTRL_EN | CH397_TX_CTRL_EN;
		}

		if (dp->chip_info.fwver >= 0x37) {
			err = ch397_write(dp->dev, CH397_WR_ETH_MACCFG, dp->presvd_mac_cfg, 0, NULL);
			if (err < 0) {
				netdev_err(dp->dev->net, "%s, Error setting mac-configure value.\n", __func__);
				return;
			}
		} else {
			err = ch397_write(dp->dev, CH397_USB_WR_REG, CH397_ETH_MAC_CFG, 4, &dp->presvd_mac_cfg);
			if (err < 0) {
				netdev_err(dp->dev->net, "%s, Error setting mac configure value.\n", __func__);
				return;
			}
		}
	}

	if (test_and_clear_bit(CH397_LINK_CHG, &dp->flags)) {
		err = ch397_set_speed(dp, dp->autoneg, dp->speed, dp->duplex);
		if (err)
			netdev_err(dp->dev->net, "ch397_set_speed error, err: %d\n", err);
	}
}

static int ch397_bind(struct usbnet *dev, struct usb_interface *intf)
{
	int ret;
	u8 mac[ETH_ALEN];
	__le32 value;
	struct ch397_common_private *dp;

	ret = usbnet_get_endpoints(dev, intf);
	if (ret)
		goto out;

	msleep(CH397_USB_DELAY);

	/* Get the MAC address */
	if (ch397_get_mac_address(dev, mac) < 0) {
		printk(KERN_ERR "Error reading MAC address\n");
		ret = -ENODEV;
		goto out;
	}

	if (ch397_read(dev, CH397_USB_RD_REG, CH397_ETH_MAC_CFG, 4, &value) < 0) {
		printk(KERN_ERR "Error getting mac configure value.\n");
		ret = -ENODEV;
		goto out;
	}

	/*
	 * Overwrite the auto-generated address only with good ones.
	 */
	if (is_valid_ether_addr(mac)) {
		eth_hw_addr_set(dev->net, mac);
	} else {
		printk(KERN_WARNING "ch397: No valid MAC address in EEPROM, using %pM\n", dev->net->dev_addr);
		__ch397_set_mac_address(dev);
	}

	dev->net->netdev_ops = &ch397_netdev_ops;
	dev->net->ethtool_ops = &ch397_ethtool_ops;
	dev->rx_urb_size = 1024 * 2;

	dev->net->needed_headroom = 8;
	dev->net->needed_tailroom = 4;

	dev->mii.dev = dev->net;
	dev->mii.mdio_read = ch397_mdio_read;
	dev->mii.mdio_write = ch397_mdio_write;
	dev->mii.phy_id_mask = 0x1f;
	dev->mii.reg_num_mask = 0x1f;
	dev->mii.phy_id = 0;

	dev->driver_priv = kzalloc(sizeof(struct ch397_common_private), GFP_KERNEL);
	if (!dev->driver_priv)
		return -ENOMEM;

	dp = dev->driver_priv;
	dp->dev = dev;
	dp->autoneg = AUTONEG_ENABLE;
	dp->speed = SPEED_100;
	dp->duplex = DUPLEX_FULL;
	dp->presvd_mac_cfg = value;

	dp->wq = create_workqueue("ch397_link_workqueue");
	if (!dp->wq) {
		printk(KERN_ERR "ch397 create link workqueue failed!\n");
		return -ENOMEM;
	}

	INIT_DELAYED_WORK(&dp->schedule_work, work_func);

	if (ch397_get_info(dev, &dp->chip_info) < 0) {
		printk(KERN_ERR "Error getting chip info\n");
		ret = -ENODEV;
		goto out;
	}
out:
	return ret;
}

static void ch397_unbind(struct usbnet *dev, struct usb_interface *intf)
{
	struct ch397_common_private *dp = dev->driver_priv;

	cancel_delayed_work(&dp->schedule_work);
	flush_workqueue(dp->wq);
	destroy_workqueue(dp->wq);
	kfree(dev->driver_priv);
}

static void reset_ch397_rx_fixup_info(struct ch397_rx_fixup_info *rx)
{
	if (rx->ch397_skb) {
		/* Discard any incomplete Ethernet frame in the netdev buffer */
		kfree_skb(rx->ch397_skb);
		rx->ch397_skb = NULL;
	}

	memset(rx, 0x00, sizeof(struct ch397_rx_fixup_info));
}

static int ch397_rx_fixup(struct usbnet *dev, struct sk_buff *skb)
{
	struct ch397_common_private *dp = dev->driver_priv;
	struct ch397_rx_fixup_info *rx = &dp->rx_fixup_info;
	u32 offset = 0;
	u32 copy_length = 0;
	void *data_tmp;

	if (rx->state == STATE_S2) {
		u32 header;
		if (rx->remaining_pad && (rx->remaining_pad + sizeof(u32) <= skb->len)) {
			offset = rx->remaining_pad;
			header = get_unaligned_le32(skb->data + offset);
			offset = 0;
			if ((header < ETH_MIN_PACKET_SIZE) || (header > ETH_DEF_PACKET_SIZE + VLAN_HLEN)) {
				netdev_err(dev->net, "%s : Bad Header Length in S2: 0x%x\n", __func__, header);
				reset_ch397_rx_fixup_info(rx);
			}
		}
	}

	while ((offset + sizeof(u32)) <= skb->len) {
		if (!rx->remaining) {
			if (skb->len - offset == sizeof(u32)) {
				rx->header = get_unaligned_le32(skb->data + offset);
				rx->split_head = true;
				offset += sizeof(u32);
				netdev_dbg(dev->net, "%s : ch397 fixup 1, rx->header: 0x%x\n", __func__, rx->header);
				break;
			}
			if (rx->split_head == true) {
				rx->split_head = false;
				offset += sizeof(u32);
			} else {
				rx->header = get_unaligned_le32(skb->data + offset);
				offset += CH397_RX_OVERHEAD;
				netdev_vdbg(dev->net, "%s : ch397 fixup 2, rx->header: 0x%x\n", __func__, rx->header);
			}

			/* get the packet length */
			rx->size = (rx->header + 3) & 0xFFFC;

			if ((rx->header < ETH_MIN_PACKET_SIZE) || (rx->header > ETH_DEF_PACKET_SIZE + VLAN_HLEN)) {
				netdev_err(dev->net, "%s : Bad Header Length: 0x%x\n", __func__, rx->header);
				reset_ch397_rx_fixup_info(rx);
				return 0;
			}

			if (rx->size > ((ETH_DEF_PACKET_SIZE + VLAN_HLEN + 3) & 0xFFFC)) {
				netdev_err(dev->net, "%s : Bad RX Length: 0x%x\n", __func__, rx->size);
				reset_ch397_rx_fixup_info(rx);
				return 0;
			}

			rx->ch397_skb = netdev_alloc_skb_ip_align(dev->net, rx->size);
			if (!rx->ch397_skb)
				return 0;

			rx->remaining = rx->remaining_header = rx->header;
			rx->remaining_pad = rx->size;
			rx->state = STATE_S1;
		}

		if (rx->remaining_pad > skb->len - offset) {
			copy_length = skb->len - offset;
			rx->remaining_header -= copy_length;
			rx->remaining_pad -= copy_length;
			rx->state = STATE_S2;
			netdev_dbg(
				dev->net,
				"%s : ch397 part of frame, copy_length: %d, remain_pad: %d, rx->size: %d, skb->len: %d, offset: %d\n",
				__func__, copy_length, rx->remaining_pad, rx->size, skb->len, offset);
		} else {
			if (rx->state == STATE_S2) {
				copy_length = rx->remaining_header;
				rx->state = STATE_S3;
			} else
				copy_length = rx->remaining;
			rx->remaining = 0;
		}

		if (rx->ch397_skb) {
			data_tmp = skb_put(rx->ch397_skb, copy_length);
			memcpy(data_tmp, skb->data + offset, copy_length);
			if (rx->state != STATE_S2) {
				usbnet_skb_return(dev, rx->ch397_skb);
				rx->ch397_skb = NULL;
			}
		}

		if (rx->state == STATE_S2)
			offset += copy_length;
		else if (rx->state == STATE_S1)
			offset += rx->remaining_pad;
		else
			offset += rx->remaining_pad;
	}

	if (skb->len != offset) {
		netdev_err(dev->net, "%s : Bad SKB Length %d, offset: %d, state: %d\n", __func__, skb->len, offset,
			   rx->state);
		reset_ch397_rx_fixup_info(rx);
		return 0;
	}

	return 1;
}

static struct sk_buff *ch397_tx_fixup(struct usbnet *dev, struct sk_buff *skb, gfp_t flags)
{
	int pad;
	int len = skb->len;
	int packet_len = 0;
	struct ethhdr *eth = eth_hdr(skb);

	if (eth->h_proto == htons(ETH_P_8021Q))
		len = min(len, ETH_DEF_PACKET_SIZE + VLAN_HLEN);
	else
		len = min(len, ETH_DEF_PACKET_SIZE);

	if (len < ETH_MIN_PACKET_SIZE)
		len = ETH_MIN_PACKET_SIZE;

	packet_len = len;
	len += CH397_TX_OVERHEAD;
	len = (len + 3) & 0xFFFFFFFC;

	if (len == 512 || len == 1024) {
		len += 4;
		packet_len += 4;
	}

	len -= CH397_TX_OVERHEAD;
	pad = len - skb->len;

	if (skb_headroom(skb) < CH397_TX_OVERHEAD || skb_tailroom(skb) < pad) {
		struct sk_buff *skb2;

		skb2 = skb_copy_expand(skb, CH397_TX_OVERHEAD, pad, flags);
		dev_kfree_skb_any(skb);
		skb = skb2;
		if (!skb)
			return NULL;
	}

	__skb_push(skb, CH397_TX_OVERHEAD);
	if (pad) {
		memset(skb->data + skb->len, 0, pad);
		__skb_put(skb, pad);
	}

	if (eth->h_proto == htons(ETH_P_8021Q))
		packet_len = min(packet_len, ETH_DEF_PACKET_SIZE + VLAN_HLEN);
	else
		packet_len = min(packet_len, ETH_DEF_PACKET_SIZE);
	memset(skb->data, 0, CH397_TX_OVERHEAD);
	skb->data[0] = packet_len;
	skb->data[1] = packet_len >> 8;
	skb->data[2] = packet_len >> 16;
	skb->data[3] = packet_len >> 24;

	usbnet_set_skb_tx_stats(skb, 1, 0);

	return skb;
}

static void ch397_status(struct usbnet *dev, struct urb *urb)
{
	struct ch397_int_data *event;
	int link;
	struct ch397_common_private *dp = dev->driver_priv;
	__le16 crc_num;

	if (urb->actual_length < 8)
		return;

	crc_num = *(__le16 *)((u8 *)urb->transfer_buffer + 10);
	if (crc_num != dp->crc_num) {
		netdev_dbg(dev->net, "ch397 rx crc: %d\n", crc_num);
		dp->crc_num = crc_num;
	}

	event = urb->transfer_buffer;
	link = !!(event->link & CH397_LINK_STATUS);
	if (netif_carrier_ok(dev->net) != link) {
		usbnet_link_change(dev, link, 1);
		netdev_dbg(dev->net, "Link Status is: %d\n", link);
		if (link == 0) {
			if ((dp->chip_info.fwver < 0x37) || (dp->speed != SPEED_100) || (dp->duplex != DUPLEX_FULL)) {
				if (!dp->autoneg) {
					set_bit(CH397_LINK_CHG, &dp->flags);
					queue_delayed_work(dp->wq, &dp->schedule_work,
							   msecs_to_jiffies(CH397_WORK_DELAY));
				}
			}
		} else {
			set_bit(CH397_CHECK_MAC, &dp->flags);
			queue_delayed_work(dp->wq, &dp->schedule_work,
							   msecs_to_jiffies(CH397_WORK_DELAY));
		}
	}
}

static int ch397_link_reset(struct usbnet *dev)
{
	struct ethtool_cmd ecmd = { .cmd = ETHTOOL_GSET };
	struct ch397_common_private *dp = dev->driver_priv;
	u32 speed;
	__le32 value;
	int err;

	mii_check_media(&dev->mii, 1, 1);
	mii_ethtool_gset(&dev->mii, &ecmd);
	speed = ethtool_cmd_speed(&ecmd);

	err = ch397_read(dev, CH397_USB_RD_REG, CH397_ETH_MAC_CFG, 4, &value);
	if (err < 0) {
		netdev_err(dev->net, "Error getting mac configure value.\n");
		return err;
	}

	if (speed == SPEED_100)
		value |= CH397_MEDIUM_PS;
	else
		value &= ~CH397_MEDIUM_PS;

	if (ecmd.duplex == DUPLEX_FULL)
		value |= CH397_MEDIUM_FD;
	else
		value &= ~CH397_MEDIUM_FD;

	value |= CH397_FLOWCTRL_EN;

	if (dp->chip_info.fwver >= 0x37) {
		err = ch397_write(dp->dev, CH397_WR_ETH_MACCFG, value, 0, NULL);
		if (err < 0) {
			netdev_err(dp->dev->net, "%s, Error setting mac-configure value.\n", __func__);
			return err;
		}
	} else {
		err = ch397_write(dp->dev, CH397_USB_WR_REG, CH397_ETH_MAC_CFG, 4, &value);
		if (err < 0) {
			netdev_err(dp->dev->net, "%s, Error setting mac configure value.\n", __func__);
			return err;
		}
	}

	dp->presvd_mac_cfg = value;

	netdev_dbg(dev->net, "link_reset() speed: %u duplex: %d\n", ethtool_cmd_speed(&ecmd), ecmd.duplex);

	return 0;
}

static const struct driver_info ch397_info = {
	.description = "WCH CH397 USB2.0 Ethernet",
	.flags = FLAG_ETHER | FLAG_LINK_INTR | FLAG_MULTI_PACKET,
	.bind = ch397_bind,
	.unbind = ch397_unbind,
	.rx_fixup = ch397_rx_fixup,
	.tx_fixup = ch397_tx_fixup,
	.status = ch397_status,
	.link_reset = ch397_link_reset,
	.reset = ch397_link_reset,
};

static int ch397_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);

	if (udev->actconfig->desc.bConfigurationValue != 1) {
		usb_driver_set_configuration(udev, 1);
		return -ENODEV;
	}

	printk(KERN_INFO "ch397 device probe, driver version: %s\n", VERSION_DESC);

	return usbnet_probe(intf, id);
}

static const struct usb_device_id ch397_ids[] = {
	{
		USB_DEVICE_INTERFACE_CLASS(0x1a86, 0x5397, USB_CLASS_VENDOR_SPEC), /* ch397 chip */
		.driver_info = (unsigned long)&ch397_info,
	},

	{
		USB_DEVICE(0x1a86, 0x5397), /* ch397 chip */
		.driver_info = (unsigned long)&ch397_info,
	},

	{
		USB_DEVICE_INTERFACE_CLASS(0x1a86, 0x5396, USB_CLASS_VENDOR_SPEC), /* ch396 chip */
		.driver_info = (unsigned long)&ch397_info,
	},

	{
		USB_DEVICE(0x1a86, 0x5396), /* ch396 chip */
		.driver_info = (unsigned long)&ch397_info,
	},

	{
		USB_DEVICE_INTERFACE_CLASS(0x1a86, 0x5395, USB_CLASS_VENDOR_SPEC), /* ch339 chip */
		.driver_info = (unsigned long)&ch397_info,
	},

	{
		USB_DEVICE(0x1a86, 0x5395), /* ch339 chip */
		.driver_info = (unsigned long)&ch397_info,
	},

	{
		USB_DEVICE_INTERFACE_CLASS(0x1a86, 0x5394, USB_CLASS_VENDOR_SPEC), /* ch336 chip */
		.driver_info = (unsigned long)&ch397_info,
	},

	{
		USB_DEVICE(0x1a86, 0x5394), /* ch336 chip */
		.driver_info = (unsigned long)&ch397_info,
	},

	{},
};

MODULE_DEVICE_TABLE(usb, ch397_ids);

static struct usb_driver ch397_driver = {
	.name = "ch397",
	.id_table = ch397_ids,
	.probe = ch397_probe,
	.disconnect = usbnet_disconnect,
	.suspend = usbnet_suspend,
	.resume = usbnet_resume,
	.disable_hub_initiated_lpm = 1,
};

module_usb_driver(ch397_driver);

MODULE_AUTHOR("Sergey Kharenko <skharenko@hust.edu.cn>");
MODULE_DESCRIPTION("USB ethernet driver for ch336/ch339/ch396/ch397, etc.");
MODULE_VERSION("1.5.0");
MODULE_LICENSE("GPL");

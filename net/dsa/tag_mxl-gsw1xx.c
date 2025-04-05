// SPDX-License-Identifier: GPL-2.0+
/*
 * net/dsa/tag_mxl-gsw1xx.c - DSA driver Special Tag support for MaxLinear GSW1xx switch chips
 *
 * Copyright (C) 2023 - 2024 MaxLinear Inc.
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
 */

#include <linux/bitops.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <net/dsa.h>

#include "dsa_priv.h"

#ifndef LINUX_VERSION_CODE
#include <linux/version.h>
#else
#define KERNEL_VERSION(a, b, c) (((a) << 16) + ((b) << 8) + (c))
#endif

/* To define the outgoing port and to discover the incoming port a special
 * tag is used by the GSW1xx.
 *
 *       Dest MAC       Src MAC    special TAG        EtherType
 * ...| 1 2 3 4 5 6 | 1 2 3 4 5 6 | 1 2 3 4 5 6 7 8 | 1 2 |...
 *                                |<--------------->|
 */


/* special tag in TX path header */
#define GSW1XX_TX_HEADER_LEN	8

/* Byte 0 = Ethertype byte 1 -> 0x88 */
/* Byte 1 = Ethertype byte 2 -> 0xC3*/

/* Byte 2 */
#define GSW1XX_TX_PORT_MAP_EN		BIT(7)
#define GSW1XX_TX_CLASS_EN		BIT(6)
#define GSW1XX_TX_TIME_STAMP_EN		BIT(5)
#define GSW1XX_TX_LRN_DIS		BIT(4)
#define GSW1XX_TX_CLASS_SHIFT		0
#define GSW1XX_TX_CLASS_MASK		GENMASK(3, 0)

/* Byte 3 */
#define GSW1XX_TX_PORT_MAP_LOW_SHIFT	0
#define GSW1XX_TX_PORT_MAP_LOW_MASK	GENMASK(7, 0)

/* Byte 4 */
#define GSW1XX_TX_PORT_MAP_HIGH_SHIFT	0
#define GSW1XX_TX_PORT_MAP_HIGH_MASK	GENMASK(7, 0)

#define GSW1XX_RX_HEADER_LEN		8

/* special tag in RX path header */
/* Byte 4 */
#define GSW1XX_RX_PORT_MAP_LOW_SHIFT	0
#define GSW1XX_RX_PORT_MAP_LOW_MASK	GENMASK(7, 0)

/* Byte 5 */
#define GSW1XX_RX_PORT_MAP_HIGH_SHIFT	0
#define GSW1XX_RX_PORT_MAP_HIGH_MASK	GENMASK(7, 0)


static struct sk_buff *gsw1xx_tag_xmit(struct sk_buff *skb,
				       struct net_device *dev)
{
	struct dsa_port *dp = dsa_slave_to_port(dev);
	u8 *gsw1xx_tag;

	if (skb == NULL)
		return skb;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0))
	err = skb_cow_head(skb, GSW1XX_TX_HEADER_LEN);
	if (err)
		return NULL;
#endif

	/* provide additional space 'GSW1XX_TX_HEADER_LEN' bytes */
	skb_push(skb, GSW1XX_TX_HEADER_LEN);

	/* add space between MAC address and Ethertype */
	memmove(skb->data, skb->data + GSW1XX_TX_HEADER_LEN, 2 * ETH_ALEN);

	/* special tag ingress */
	gsw1xx_tag = skb->data + 2 * ETH_ALEN;
	gsw1xx_tag[0] = 0x88;
	gsw1xx_tag[1] = 0xc3;
	gsw1xx_tag[2] = GSW1XX_TX_PORT_MAP_EN | GSW1XX_TX_LRN_DIS;
	gsw1xx_tag[3] = BIT(dp->index + GSW1XX_TX_PORT_MAP_LOW_SHIFT) & GSW1XX_TX_PORT_MAP_LOW_MASK;
	gsw1xx_tag[4] = 0;
	gsw1xx_tag[5] = 0;
	gsw1xx_tag[6] = 0;
	gsw1xx_tag[7] = 0;

	return skb;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 15, 0))
static struct sk_buff *gsw1xx_tag_rcv(struct sk_buff *skb,
				      struct net_device *dev,
				      struct packet_type *pt)
#else
static struct sk_buff *gsw1xx_tag_rcv(struct sk_buff *skb,
				      struct net_device *dev)
#endif
{
	int port;
	u8 *gsw1xx_tag;

	if (unlikely(!pskb_may_pull(skb, GSW1XX_RX_HEADER_LEN))) {
		dev_warn_ratelimited(&dev->dev, "Dropping packet, cannot pull SKB\n");
		return NULL;
	}

	gsw1xx_tag = skb->data - 2;

	if ((gsw1xx_tag[0] != 0x88) && (gsw1xx_tag[1] != 0xc3)) {
		dev_warn_ratelimited(&dev->dev, "Dropping packet due to invalid special tag marker\n");
		dev_warn_ratelimited(&dev->dev, "Rx Packet Tag: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", gsw1xx_tag[0], gsw1xx_tag[1], gsw1xx_tag[2], gsw1xx_tag[3], gsw1xx_tag[4], gsw1xx_tag[5], gsw1xx_tag[6], gsw1xx_tag[7]);
		return NULL;
	}

	/* Get source port information */
	port = (gsw1xx_tag[2] & GSW1XX_RX_PORT_MAP_LOW_MASK) >> GSW1XX_RX_PORT_MAP_LOW_SHIFT;
	skb->dev = dsa_master_find_slave(dev, 0, port);
	if (!skb->dev) {
		dev_warn_ratelimited(&dev->dev, "Dropping packet due to invalid source port\n");
		dev_warn_ratelimited(&dev->dev, "Rx Packet Tag: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", gsw1xx_tag[0], gsw1xx_tag[1], gsw1xx_tag[2], gsw1xx_tag[3], gsw1xx_tag[4], gsw1xx_tag[5], gsw1xx_tag[6], gsw1xx_tag[7]);
		return NULL;
	}

	/* remove the GSW1xx special tag between MAC addresses and the current ethertype field. */
	skb_pull_rcsum(skb, GSW1XX_RX_HEADER_LEN);
	memmove(skb->data - ETH_HLEN, skb->data - (ETH_HLEN + GSW1XX_RX_HEADER_LEN), 2 * ETH_ALEN);

	return skb;
}

static const struct dsa_device_ops gsw1xx_netdev_ops = {
	.name = "gsw1xx",
	.proto	= DSA_TAG_PROTO_MXL_GSW1XX,
	.xmit = gsw1xx_tag_xmit,
	.rcv = gsw1xx_tag_rcv,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 14, 0))
	.overhead = GSW1XX_RX_HEADER_LEN,
#else
	.needed_headroom = GSW1XX_RX_HEADER_LEN,
#endif
};

MODULE_LICENSE("GPL");
MODULE_ALIAS_DSA_TAG_DRIVER(DSA_TAG_PROTO_MXL_GSW1XX);

module_dsa_tag_driver(gsw1xx_netdev_ops);

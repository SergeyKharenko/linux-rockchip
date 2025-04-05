/* SPDX-License-Identifier: GPL-2.0 */
/*
 * drivers/net/dsa/mxl-gsw1xx.h - Header file for driver for MaxLinear GSW1xx switch chips
 *
 * Copyright (C) 2023 -2024 MaxLinear Inc.
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
 */

#define GSW1xx_PORTS			6

/* Device Addresses */
#define DEVADDR_PHY(p)			(p)

#define PHY_STAT_REG			1
#define   PHY_STAT_LINK			2
#define MMDIO_STAT0				0xf416
#define   MMDIO_STAT_LINK		(1<<5)
#define   MMDIO_STAT_SPEED		(3<<3)
#define     SPEED_10M			(0<<3)
#define     SPEED_100M			(1<<3)
#define     SPEED_1G			(2<<3)
#define   MMDIO_STAT_FDUP		(1<<2)

/* Port used for RGMII or optional RMII */
#define GSW1XX_MII_PORT			0x5
/* Port used for SGMII */
#define GSW1XX_SGMII_PORT		0x4

/* PDI Registers */
#define MMDIO_BASE			0xF400
#define OP_CODE_RD			(2 << 10)
#define OP_CODE_WR			(1 << 10)
#define PHY_ADDR(p)			(((p) & 0x1f)<<5)
#define PHY_REG_MASK			0x1f

/* GSW1XX MDIO Registers */
#define GSW1XX_MDIO_GLOB		0x00
#define  GSW1XX_MDIO_GLOB_ENABLE	BIT(15)
#define GSW1XX_MDIO_CTRL		0x08
#define  GSW1XX_MDIO_CTRL_BUSY		BIT(12)
#define  GSW1XX_MDIO_CTRL_RD		BIT(11)
#define  GSW1XX_MDIO_CTRL_WR		BIT(10)
#define  GSW1XX_MDIO_CTRL_PHYAD_MASK	0x1f
#define  GSW1XX_MDIO_CTRL_PHYAD_SHIFT	5
#define  GSW1XX_MDIO_CTRL_REGAD_MASK	0x1f
#define GSW1XX_MDIO_READ		0x09
#define GSW1XX_MDIO_WRITE		0x0A
#define GSW1XX_MDIO_MDC_CFG0		0x0B
#define GSW1XX_MDIO_MDC_CFG1		0x0C
#define GSW1XX_MDIO_PHYp(p)		(0x15 - (p))
#define  GSW1XX_MDIO_PHY_LINK_MASK	0x6000
#define  GSW1XX_MDIO_PHY_LINK_DOWN	0x4000
#define  GSW1XX_MDIO_PHY_LINK_UP	0x2000
#define  GSW1XX_MDIO_PHY_SPEED_MASK	0x1800
#define  GSW1XX_MDIO_PHY_SPEED_M10	0x0000
#define  GSW1XX_MDIO_PHY_SPEED_M100	0x0800
#define  GSW1XX_MDIO_PHY_SPEED_G1	0x1000
#define  GSW1XX_MDIO_PHY_FDUP_MASK	0x0600
#define  GSW1XX_MDIO_PHY_FDUP_EN	0x0200
#define  GSW1XX_MDIO_PHY_FDUP_DIS	0x0600
#define  GSW1XX_MDIO_PHY_FCONTX_MASK	0x0180
#define  GSW1XX_MDIO_PHY_FCONTX_EN	0x0080
#define  GSW1XX_MDIO_PHY_FCONTX_DIS	0x0180
#define  GSW1XX_MDIO_PHY_FCONRX_MASK	0x0060
#define  GSW1XX_MDIO_PHY_FCONRX_EN	0x0020
#define  GSW1XX_MDIO_PHY_FCONRX_DIS	0x0060
#define  GSW1XX_MDIO_PHY_ADDR_MASK	0x001f

#define SWITCH_BASE			0xe000

/* GSW1XX GPIO Registers */
#define GPIO_BASE			0xF300
#define GPIO_ALTSEL0			0x83
#define GPIO_ALTSEL0_EXTPHY_MUX_VAL	0x03C3
#define GPIO_ALTSEL1			0x84
#define GPIO_ALTSEL1_EXTPHY_MUX_VAL	0x003F

/* GSW1XX MII Registers */
#define RGMII_BASE			0xF100
#define   MII_CFG5_OFFS			0
#define   MII_PCDU5_OFFS		1
#define     PCDU5_RXDLY_NS(t)		((t*2)<<7)
#define     PCDU5_TXDLY_NS(t)		((t*2)<<0)
#define GSW1XX_MII_CFG			0x0

#define  GSW1XX_MII_CFG_RESET		BIT(15)
#define  GSW1XX_MII_CFG_EN		BIT(14)
#define  GSW1XX_MII_CFG_ISOLATE	BIT(13)
#define  GSW1XX_MII_CFG_LDCLKDIS	BIT(12)
#define  GSW1XX_MII_CFG_RGMII_IBS	BIT(8)
#define  GSW1XX_MII_CFG_RMII_CLK	BIT(7)
#define  GSW1XX_MII_CFG_MODE_RMIIM	0x3
#define  GSW1XX_MII_CFG_MODE_RGMII	0x4
#define  GSW1XX_MII_CFG_MODE_MASK	0xf
#define  GSW1XX_MII_CFG_RATE_M2P5	0x00
#define  GSW1XX_MII_CFG_RATE_M25	0x10
#define  GSW1XX_MII_CFG_RATE_M125	0x20
#define  GSW1XX_MII_CFG_RATE_M50	0x30
#define  GSW1XX_MII_CFG_RATE_AUTO	0x40
#define  GSW1XX_MII_CFG_RATE_MASK	0x70
#define GSW1XX_MII_PCDU0		0x01
#define GSW1XX_MII_PCDU1		0x03
#define GSW1XX_MII_PCDU5		0x05
#define  GSW1XX_MII_PCDU_TXDLY_SHIFT	0
#define  GSW1XX_MII_PCDU_RXDLY_SHIFT	7
#define  GSW1XX_MII_PCDU_TXDLY_MASK	GENMASK(2, GSW1XX_MII_PCDU_TXDLY_SHIFT)
#define  GSW1XX_MII_PCDU_RXDLY_MASK	GENMASK(9, GSW1XX_MII_PCDU_RXDLY_SHIFT)
/* RGMII PAD Slew Control Register */
#define RGMII_SLEW_CFG 0xFA78
#define RGMII_SLEW_CFG_RX_2_5_V BIT(4)
#define RGMII_SLEW_CFG_TX_2_5_V BIT(5)

/* GSW1XX Core Registers */
#define GSW1XX_SWRES			0x000
#define  GSW1XX_SWRES_R1		BIT(1)  /* GSW1XX Software reset */
#define  GSW1XX_SWRES_R0		BIT(0)  /* GSW1XX Hardware reset */
#define GSW1XX_VERSION			0x013

/* GSW1XX PNUM Registers */
#define GSW1XX_PNUM_ID			0xA11
#define  GSW1XX_PNUM_ID_VER_MASK	GENMASK(15, 12)

#define GSW1XX_BM_RAM_VAL(x)		(0x043 - (x))
#define GSW1XX_BM_RAM_ADDR		0x044
#define GSW1XX_BM_RAM_CTRL		0x045
#define  GSW1XX_BM_RAM_CTRL_BAS		BIT(15)
#define  GSW1XX_BM_RAM_CTRL_OPMOD	BIT(5)
#define  GSW1XX_BM_RAM_CTRL_ADDR_MASK	GENMASK(4, 0)
#define GSW1XX_BM_QUEUE_GCTRL		0x04A
#define  GSW1XX_BM_QUEUE_GCTRL_GL_MOD	BIT(10)  /* WRED Mode LOC/GLOB */
/* buffer management Port Configuration Register */
#define GSW1XX_BM_PCFGp(p)		(0x080 + ((p) * 2))
#define  GSW1XX_BM_PCFG_CNTEN		BIT(0)  /* RMON Counter Enable */
#define  GSW1XX_BM_PCFG_IGCNT		BIT(1)  /* Ingres Special Tag RMON count */

/* PCE */
#define GSW1XX_PCE_TBL_KEY(x)			(0x447 - (x))
#define GSW1XX_PCE_TBL_MASK			0x448
#define GSW1XX_PCE_TBL_VAL(x)			(0x44D - (x))
#define GSW1XX_PCE_TBL_ADDR			0x44E
#define GSW1XX_PCE_TBL_CTRL			0x44F
#define  GSW1XX_PCE_TBL_CTRL_BAS		BIT(15)
#define  GSW1XX_PCE_TBL_CTRL_TYPE		BIT(13)
#define  GSW1XX_PCE_TBL_CTRL_VLD		BIT(12)
#define  GSW1XX_PCE_TBL_CTRL_KEYFORM		BIT(11)
#define  GSW1XX_PCE_TBL_CTRL_GMAP_MASK		GENMASK(10, 7)
#define  GSW1XX_PCE_TBL_CTRL_OPMOD_MASK		GENMASK(6, 5)
#define  GSW1XX_PCE_TBL_CTRL_OPMOD_ADRD		0x00
#define  GSW1XX_PCE_TBL_CTRL_OPMOD_ADWR		0x20
#define  GSW1XX_PCE_TBL_CTRL_OPMOD_KSRD		0x40
#define  GSW1XX_PCE_TBL_CTRL_OPMOD_KSWR		0x60
#define  GSW1XX_PCE_TBL_CTRL_ADDR_MASK		GENMASK(4, 0)
#define GSW1XX_PCE_PMAP1			0x453	/* Monitoring port map */
#define GSW1XX_PCE_PMAP2			0x454	/* Default Multicast port map */
#define GSW1XX_PCE_PMAP3			0x455	/* Default Unknown Unicast port map */
#define GSW1XX_PCE_GCTRL_0			0x456
#define  GSW1XX_PCE_GCTRL_0_MTFL		BIT(0)  /* MAC Table Flushing */
#define  GSW1XX_PCE_GCTRL_0_MC_VALID		BIT(3)
#define  GSW1XX_PCE_GCTRL_0_VLAN		BIT(14) /* VLAN aware Switching */
#define GSW1XX_PCE_GCTRL_1			0x457
#define  GSW1XX_PCE_GCTRL_1_MAC_GLOCK		BIT(2)	/* MAC Address table lock */
#define  GSW1XX_PCE_GCTRL_1_MAC_GLOCK_MOD	BIT(3) /* Mac address table lock forwarding mode */
#define  GSW1XX_PCE_GCTRL_1_VLANMD		BIT(9)	/* VLAN MODE */
#define GSW1XX_PCE_PCTRL_0p(p)			(0x480 + ((p) * 0xA))
#define  GSW1XX_PCE_GCTRL_1_LRNMOD		BIT(0)	/* MAC Address Learning Mode */
#define  GSW1XX_PCE_PCTRL_0_TVM			BIT(5)	/* Transparent VLAN mode */
#define  GSW1XX_PCE_PCTRL_0_VREP		BIT(6)	/* VLAN Replace Mode */
#define  GSW1XX_PCE_PCTRL_0_INGRESS		BIT(11)	/* Accept special tag in ingress */
#define  GSW1XX_PCE_PCTRL_0_PSTATE_LISTEN	0x0
#define  GSW1XX_PCE_PCTRL_0_PSTATE_RX		0x1
#define  GSW1XX_PCE_PCTRL_0_PSTATE_TX		0x2
#define  GSW1XX_PCE_PCTRL_0_PSTATE_LEARNING	0x3
#define  GSW1XX_PCE_PCTRL_0_PSTATE_FORWARDING	0x7
#define  GSW1XX_PCE_PCTRL_0_PSTATE_MASK		GENMASK(2, 0)
#define GSW1XX_PCE_VCTRL(p)			(0x485 + ((p) * 0xA))
#define  GSW1XX_PCE_VCTRL_UVR			BIT(0)	/* Unknown VLAN Rule */
#define  GSW1XX_PCE_VCTRL_VINR_MASK		GENMASK(2, 1)	/* CTAG VLAN Ingress rule */
#define  GSW1XX_PCE_VCTRL_VINR_ALL		0 /* CTAG VLAN Ingress rule: admit all frames */
/* CTAG VLAN Ingress rule: admit only VLAN-tagged frames, discard priority-tagged/untagged frames */
#define  GSW1XX_PCE_VCTRL_VINR_VTAG		BIT(1)
/* CTAG VLAN Ingress rule: aadmit untagged/priority-tagged frames only, discard VLAN tagged frames */
#define  GSW1XX_PCE_VCTRL_VINR_UNTAG		BIT(2)
#define  GSW1XX_PCE_VCTRL_VIMR			BIT(3)	/* VLAN Ingress Member violation rule */
#define  GSW1XX_PCE_VCTRL_VEMR			BIT(4)	/* VLAN Egress Member violation rule */
#define  GSW1XX_PCE_VCTRL_VSR			BIT(5)	/* VLAN Security */
#define  GSW1XX_PCE_VCTRL_VID0			BIT(6)	/* Priority Tagged Rule */
#define GSW1XX_PCE_DEFPVID(p)			(0x486 + ((p) * 0xA))

#define GSW1XX_MAC_FLEN				0x8C5
#define GSW1XX_MAC_CTRL_0p(p)			(0x903 + ((p) * 0xC))
#define  GSW1XX_MAC_CTRL_0_PADEN		BIT(8)
#define  GSW1XX_MAC_CTRL_0_FCS_EN		BIT(7)
#define  GSW1XX_MAC_CTRL_0_FCON_MASK		0x0070
#define  GSW1XX_MAC_CTRL_0_FCON_AUTO		0x0000
#define  GSW1XX_MAC_CTRL_0_FCON_RX		0x0010
#define  GSW1XX_MAC_CTRL_0_FCON_TX		0x0020
#define  GSW1XX_MAC_CTRL_0_FCON_RXTX		0x0030
#define  GSW1XX_MAC_CTRL_0_FCON_NONE		0x0040
#define  GSW1XX_MAC_CTRL_0_FDUP_MASK		0x000C
#define  GSW1XX_MAC_CTRL_0_FDUP_AUTO		0x0000
#define  GSW1XX_MAC_CTRL_0_FDUP_EN		0x0004
#define  GSW1XX_MAC_CTRL_0_FDUP_DIS		0x000C
#define  GSW1XX_MAC_CTRL_0_GMII_MASK		0x0003
#define  GSW1XX_MAC_CTRL_0_GMII_AUTO		0x0000
#define  GSW1XX_MAC_CTRL_0_GMII_MII		0x0001
#define  GSW1XX_MAC_CTRL_0_GMII_RGMII		0x0002
#define GSW1XX_MAC_CTRL_2p(p)			(0x905 + ((p) * 0xC))
#define  GSW1XX_MAC_CTRL_2_MLEN			BIT(3) /* Maximum Untagged Frame Length */
#define GSW1XX_MAC_CTRL_4p(p)			(0x907 + ((p) * 0xC))
#define  GSW1XX_MAC_CTRL_4_LPIEN		BIT(7) /* LPI Mode Enable */
#define  GSW1XX_MAC_CTRL_4_GWAIT_MASK		GENMASK(14, 8) /* LPI Wait Time 1G */
#define  GSW1XX_MAC_CTRL_4_GWAIT		8 /* LPI Wait Time 1G */
#define  GSW1XX_MAC_CTRL_4_WAIT_MASK		GENMASK(6, 0) /* LPI Wait Time 100M */
#define  GSW1XX_MAC_CTRL_4_WAIT			0 /* LPI Wait Time 100M */

/* Ethernet Switch Fetch DMA Port Control Register */
#define GSW1XX_FDMA_PCTRLp(p)			(0xA80 + ((p) * 0x6))
#define  GSW1XX_FDMA_PCTRL_EN			BIT(0)  /* FDMA Port Enable */
#define  GSW1XX_FDMA_PCTRL_STEN			BIT(1)  /* Special Tag Insertion Enable */
#define  GSW1XX_FDMA_PCTRL_VLANMOD_MASK		GENMASK(4, 3)   /* VLAN Modification Control */
#define  GSW1XX_FDMA_PCTRL_VLANMOD_SHIFT	3   /* VLAN Modification Control */
#define  GSW1XX_FDMA_PCTRL_VLANMOD_DIS		(0x0 << GSW1XX_FDMA_PCTRL_VLANMOD_SHIFT)
#define  GSW1XX_FDMA_PCTRL_VLANMOD_PRIO		(0x1 << GSW1XX_FDMA_PCTRL_VLANMOD_SHIFT)
#define  GSW1XX_FDMA_PCTRL_VLANMOD_ID		(0x2 << GSW1XX_FDMA_PCTRL_VLANMOD_SHIFT)
#define  GSW1XX_FDMA_PCTRL_VLANMOD_BOTH		(0x3 << GSW1XX_FDMA_PCTRL_VLANMOD_SHIFT)

/* Ethernet Switch Store DMA Port Control Register */
#define GSW1XX_SDMA_PCTRLp(p)			(0xBC0 + ((p) * 0x6))
#define  GSW1XX_SDMA_PCTRL_EN			BIT(0)  /* SDMA Port Enable */
#define  GSW1XX_SDMA_PCTRL_FCEN			BIT(1)  /* Flow Control Enable */
#define  GSW1XX_SDMA_PCTRL_PAUFWD		BIT(3)  /* Pause Frame Forwarding */

#define GSW1XX_TABLE_ACTIVE_VLAN		0x01
#define GSW1XX_TABLE_VLAN_MAPPING		0x02
#define GSW1XX_TABLE_MAC_BRIDGE			0x0b
#define  GSW1XX_TABLE_MAC_BRIDGE_STATIC	0x01    /* Static not, aging entry */
#define  GSW1XX_TABLE_MAC_BRIDGE_STATIC_VALID	0x02    /* Static not, aging entry, valid bit */

/* Ethernet Switch PCE Port Control Register 3 */
#define GSW1XX_PCE_PCTRL_3p(p)			(0x483 + ((p) * 0xA))
#define  GSW1XX_PCE_PCTRL_3_LNDIS		BIT(15)  /* Learning Disable */

/* SGMII configuration */
#define GSW1XX_RST_REQ				0xfa01
#define GSW1XX_SGMII_PHY_HWBU_CTRL		0xd009
#define GSW1XX_SGMII_TBI_TXANEGH		0xd300
#define GSW1XX_SGMII_TBI_TXANEGL		0xd301
#define GSW1XX_SGMII_TBI_ANEGCTL		0xd304
#define GSW1XX_SGMII_TBI_TBICTL			0xd305
#define GSW1XX_SGMII_TBI_LPSTAT			0xd30A
#define GSW1XX_SGMII_PHY_D			0xd100
#define GSW1XX_SGMII_PHY_A			0xd101
#define GSW1XX_SGMII_PHY_C			0xd102
#define GSW1XX_SGMII_PCS_RXB_CTL		0xd401
#define GSW1XX_SGMII_PCS_TXB_CTL		0xd404
#define GSW1XX_SGMII_PHY_RX0_CFG2		0xd004

/* Number of entries in the MAC Address Learning table */
#define GSW1XX_MAC_LEARN_TABLE_SIZE		2048

/* Maximum packet size supported by the switch. */
#define GSW1XX_MAX_PACKET_LENGTH		9600

/* SGMII related settings */
#define GSW1XX_SGMII_PORT_NOT_USED		0
#define GSW1XX_SGMII_WITH_EXTERNAL_PHY		1
#define GSW1XX_SGMII_IS_CPU_PORT		2

#define GSW1XX_NCO_CTRL				0xF968
#define GSW1XX_SGMII_HSP_MASK			GENMASK(3, 2)
#define GSW1XX_SGMII_SEL			BIT(1)
#define GSW1XX_SGMII_1G				0x0
#define GSW1XX_SGMII_2G5			0xC
#define GSW1XX_SGMII_1G_NCO1			0x0
#define GSW1XX_SGMII_2G5_NCO2			0x2

#define GSW1XX_LEDXH_CFG_pin(pin)		(0x1f01e2 + ((pin) * 0x2))
#define GSW1XX_LEDXL_CFG_pin(pin)		(0x1f01e3 + ((pin) * 0x2))

#define GSW1XX_LEDXH_CFG_CON_MASK		GENMASK(7, 4)
#define GSW1XX_LEDXH_CFG_BLINKF_MASK		GENMASK(3, 0)
#define GSW1XX_LEDXL_CFG_BLINKS_MASK		GENMASK(7, 4)
#define GSW1XX_LEDXL_CFG_PULSE_MASK		GENMASK(3, 0)

#define GSW1XX_LEDXH_CFG_CON_NONE		(0x0 << 4)
#define GSW1XX_LEDXH_CFG_CON_LINK10		(0x1 << 4)
#define GSW1XX_LEDXH_CFG_CON_LINK100		(0x2 << 4)
#define GSW1XX_LEDXH_CFG_CON_LINK10X		(0x3 << 4)
#define GSW1XX_LEDXH_CFG_CON_LINK1000		(0x4 << 4)
#define GSW1XX_LEDXH_CFG_CON_LINK10_0		(0x5 << 4)
#define GSW1XX_LEDXH_CFG_CON_LINK100X		(0x6 << 4)
#define GSW1XX_LEDXH_CFG_CON_LINK10XX		(0x7 << 4)
#define GSW1XX_LEDXH_CFG_CON_PDOWN		(0x8 << 4)
#define GSW1XX_LEDXH_CFG_CON_EEE		(0x9 << 4)
#define GSW1XX_LEDXH_CFG_CON_ANEG		(0xA << 4)
#define GSW1XX_LEDXH_CFG_CON_ABIST		(0xB << 4)
#define GSW1XX_LEDXH_CFG_CON_CDIAG		(0xC << 4)
#define GSW1XX_LEDXH_CFG_CON_FIBER		(0xD << 4)

#define GSW1XX_LEDXH_CFG_BLINKF_NONE		0x0
#define GSW1XX_LEDXH_CFG_BLINKF_LINK10		0x1
#define GSW1XX_LEDXH_CFG_BLINKF_LINK100		0x2
#define GSW1XX_LEDXH_CFG_BLINKF_LINK10X		0x3
#define GSW1XX_LEDXH_CFG_BLINKF_LINK1000	0x4
#define GSW1XX_LEDXH_CFG_BLINKF_LINK10_0	0x5
#define GSW1XX_LEDXH_CFG_BLINKF_LINK100X	0x6
#define GSW1XX_LEDXH_CFG_BLINKF_LINK10XX	0x7
#define GSW1XX_LEDXH_CFG_BLINKF_PDOWN		0x8
#define GSW1XX_LEDXH_CFG_BLINKF_EEE		0x9
#define GSW1XX_LEDXH_CFG_BLINKF_ANEG		0xA
#define GSW1XX_LEDXH_CFG_BLINKF_BIST		0xB
#define GSW1XX_LEDXH_CFG_BLINKF_CDIAG		0xC

#define GSW1XX_LEDXL_CFG_BLINKS_NONE		(0x0 << 4)
#define GSW1XX_LEDXL_CFG_BLINKS_LINK10		(0x1 << 4)
#define GSW1XX_LEDXL_CFG_BLINKS_LINK100		(0x2 << 4)
#define GSW1XX_LEDXL_CFG_BLINKS_LINK10X		(0x3 << 4)
#define GSW1XX_LEDXL_CFG_BLINKS_LINK1000	(0x4 << 4)
#define GSW1XX_LEDXL_CFG_BLINKS_LINK10_0	(0x5 << 4)
#define GSW1XX_LEDXL_CFG_BLINKS_LINK100X	(0x6 << 4)
#define GSW1XX_LEDXL_CFG_BLINKS_LINK10XX	(0x7 << 4)
#define GSW1XX_LEDXL_CFG_BLINKS_PDOWN		(0x8 << 4)
#define GSW1XX_LEDXL_CFG_BLINKS_EEE		(0x9 << 4)
#define GSW1XX_LEDXL_CFG_BLINKS_ANEG		(0xA << 4)
#define GSW1XX_LEDXL_CFG_BLINKS_BIST		(0xB << 4)
#define GSW1XX_LEDXL_CFG_BLINKS_CDIAG		(0xC << 4)

#define GSW1XX_LEDXL_CFG_PULSE_NONE		0x0
#define GSW1XX_LEDXL_CFG_PULSE_TXACT		0x1
#define GSW1XX_LEDXL_CFG_PULSE_RXACT		0x2
#define GSW1XX_LEDXL_CFG_PULSE_COL		0x4
#define GSW1XX_LEDXL_CFG_PULSE_PULSEM		0x8

#define GSW1XX_MMDDATA				0xE
#define GSW1XX_MMDCTRL				0xD

#define GSW1XX_FID_WIDTH			0xD
#define GSW1XX_LINUX_BRIDGE_DEFAULT_PVID	1

#define GSW1XX_CHECK_PHYRXERR			1

/* SGMII MODE USER configuration */
#define GSW1XX_SGMII_MAC			0
#define GSW1XX_SGMII_PHY			1
#define GSW1XX_SGMII_1000BX			2

/* SGMII RX Data signal Invert Control - 1 - default */
#define GSW1XX_SGMII_RX_SIGNAL_INV		1

/* ****************************************************** */
/* Customer specific configuration START		  */
/* Adapt here if other than default values are required!  */
/* ****************************************************** */

/* adjust the SMDIO Base address to the related pinstrapping settings */
#define SMDIO_BADR				0x1f

/* RGMII customer delays */
	#define GSW1XX_MII_PCDU_RXDLY_CUSTOM	4
	#define GSW1XX_MII_PCDU_TXDLY_CUSTOM	4

/* RMII mode only*/
/* boards which use RMII might have a dedicated oscillator on board
 * for 50 MHz RMII reference block -> conflict when GSW1xx recflk is output
 * 0 - refclk input, 1 - refclk output
 */
#define GSW1XX_MII_CFG_RMII_CLK_CUSTOM	0

/* Automatic clock disable in case of link down for RGMII/RMII port */
#define GSW1XX_MII_CFG_AUTO_LDCLK_DISABLE	0

/* RGMII TX PAD Voltage Supply Level - 0: 3.3V, 1: 2.5V */
	#define RGMII_SLEW_RX_TX_VOLT 0

/* SGMII configuration */
/* Indicate if SGMII will be used to set_max_ports accordingly
 * GSW1XX_SGMII_PORT_NOT_USED - SGMII not used
 * GSW1XX_SGMII_WITH_EXTERNAL_PHY - SGMII used to connect external PHY
 * GSW1XX_SGMII_IS_CPU_PORT - SGMII used as cpu port
 */
#define GSW1XX_USE_SGMII	GSW1XX_SGMII_PORT_NOT_USED

/* GSW1XX_SGMII_MAC - SGMII is operated in MAC mode (e.g. to connect an external PHY)
 * GSW1XX_SGMII_PHY - SGMII is operated in PHY mode (e.g. to connect to a MAC)
 * GSW1XX_1000Base-X - SGMII is operated in 1000Base-X mode
 */
#define GSW1XX_SGMII_MODE	GSW1XX_SGMII_MAC

/* 1 - SGMII shall use Autonegotiation,
 * 0 - SGMII shall work in forced mode
 */
#define GSW1XX_SGMII_AUTO_NEG_ENABLED	1

/* Adjust default LED settings for internal PHY ports.
 * might be overwritten e.g. by intel_xway.c phy driver
 */
#define GSW1XX_CONFIG_CUSTOM_LED_SETTINGS	1

#define GSW1XX_LED0H_CFG_CUSTOM		(GSW1XX_LEDXH_CFG_CON_LINK1000 | \
					GSW1XX_LEDXH_CFG_BLINKF_NONE)
#define GSW1XX_LED0L_CFG_CUSTOM		(GSW1XX_LEDXL_CFG_PULSE_TXACT | \
					GSW1XX_LEDXL_CFG_PULSE_RXACT | \
					GSW1XX_LEDXL_CFG_PULSE_PULSEM)
#define GSW1XX_LED1H_CFG_CUSTOM		(GSW1XX_LEDXH_CFG_CON_LINK10X | \
					GSW1XX_LEDXH_CFG_BLINKF_NONE)
#define GSW1XX_LED1L_CFG_CUSTOM		(GSW1XX_LEDXL_CFG_PULSE_TXACT | \
					GSW1XX_LEDXL_CFG_PULSE_RXACT | \
					GSW1XX_LEDXL_CFG_PULSE_PULSEM)
#define GSW1XX_LED2H_CFG_CUSTOM		(GSW1XX_LEDXH_CFG_BLINKF_LINK1000 | \
					GSW1XX_LEDXH_CFG_BLINKF_NONE)
#define GSW1XX_LED2L_CFG_CUSTOM		(GSW1XX_LEDXL_CFG_BLINKS_NONE | \
					GSW1XX_LEDXL_CFG_PULSE_NONE)
/* Driver uses transparent port based VLANs to group bridges and realize
 * isolation between standalone ports and bridges. The PVIDs used for
 * this purpose can be adapted here.
 */
#define GSW1XX_STANDALONE_PORT_PVID	0
#define GSW1XX_BRIDGE_PVID_START	1
#define GSW1XX_BRIDGE_PVID_END		(GSW1XX_BRIDGE_PVID_START + GSW1xx_PORTS)

/* Can be disabled if BRIDGE and VLAN_8021Q is enabled in kernel config.
 * In this case, a VLANID 0 will be configured for each standalone port automatically.
 */
#define GSW1XX_STANDALONE_SINGLE_PORT_BR	1
/* **************************************************** */
/* Customer specific configuration END			*/
/* **************************************************** */

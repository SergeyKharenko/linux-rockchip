/* SPDX-License-Identifier: GPL-2.0 */
/*
 * drivers/net/dsa/mxl-gsw1xx_pce.c - PCE microcode code update for driver for MaxLinear GSW1xx switch chips

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
 */

/**************************************************************************/
/*      DEFINES:                                                          */
/**************************************************************************/

#define INSTR 0
#define IPV6 1
#define LENACCU 2

/* GSWIP_2.X */
enum {
	GOUT_MAC0 = 0,
	GOUT_MAC1,
	GOUT_MAC2,
	GOUT_MAC3,
	GOUT_MAC4,
	GOUT_MAC5,
	GOUT_ETHTYP,
	GOUT_VTAG0,
	GOUT_VTAG1,
	GOUT_ITAG0,
	GOUT_ITAG1,	/*10 */
	GOUT_ITAG2,
	GOUT_ITAG3,
	GOUT_IP0,
	GOUT_IP1,
	GOUT_IP2,
	GOUT_IP3,
	GOUT_SIP0,
	GOUT_SIP1,
	GOUT_SIP2,
	GOUT_SIP3,	/*20*/
	GOUT_SIP4,
	GOUT_SIP5,
	GOUT_SIP6,
	GOUT_SIP7,
	GOUT_DIP0,
	GOUT_DIP1,
	GOUT_DIP2,
	GOUT_DIP3,
	GOUT_DIP4,
	GOUT_DIP5,	/*30*/
	GOUT_DIP6,
	GOUT_DIP7,
	GOUT_SESID,
	GOUT_PROT,
	GOUT_APP0,
	GOUT_APP1,
	GOUT_IGMP0,
	GOUT_IGMP1,
	GOUT_STAG0 = 61,
	GOUT_STAG1 = 62,
	GOUT_NONE = 63,
};

/* parser's microcode flag type */
enum {
	GFLAG_ITAG = 0,
	GFLAG_VLAN,
	GFLAG_SNAP,
	GFLAG_PPPOE,
	GFLAG_IPV6,
	GFLAG_IPV6FL,
	GFLAG_IPV4,
	GFLAG_IGMP,
	GFLAG_TU,
	GFLAG_HOP,
	GFLAG_NN1,	/*10*/
	GFLAG_NN2,
	GFLAG_END,
	GFLAG_NO,	/*13*/
	GFLAG_SVLAN,	/*14 */
};

struct gsw1xx_pce_microcode {
	u16 val_3;
	u16 val_2;
	u16 val_1;
	u16 val_0;
};

#define PCE_MC_M(val, msk, ns, out, len, type, flags, ipv4_len) \
	{ (val), (msk), ((ns) << 10 | (out) << 4 | (len) >> 1),\
	 ((len) & 1) << 15 | (type) << 13 | (flags) << 9 | (ipv4_len) << 8 }

static const struct gsw1xx_pce_microcode gsw1xx_pce_microcode[] = {
	/*-----------------------------------------------------------------*/
	/**   value    mask   ns  out_fields   L  type   flags   ipv4_len **/
	/*-----------------------------------------------------------------*/
	/* V22_2X (IPv6 issue fixed) */
	PCE_MC_M(0x88c3, 0xFFFF, 1,  GOUT_ITAG0,  4, INSTR,   GFLAG_ITAG,  0),
	PCE_MC_M(0x8100, 0xFFFF, 4,  GOUT_STAG0,  2, INSTR,   GFLAG_SVLAN, 0),
	PCE_MC_M(0x88A8, 0xFFFF, 4,  GOUT_STAG0,  2, INSTR,   GFLAG_SVLAN, 0),
	PCE_MC_M(0x9100, 0xFFFF, 4,  GOUT_STAG0,  2, INSTR,   GFLAG_SVLAN, 0),
	PCE_MC_M(0x8100, 0xFFFF, 5,  GOUT_VTAG0,  2, INSTR,   GFLAG_VLAN,  0),
	PCE_MC_M(0x88A8, 0xFFFF, 6,  GOUT_VTAG0,  2, INSTR,   GFLAG_VLAN,  0),
	PCE_MC_M(0x9100, 0xFFFF, 4,  GOUT_VTAG0,  2, INSTR,   GFLAG_VLAN,  0),
	PCE_MC_M(0x8864, 0xFFFF, 20, GOUT_ETHTYP, 1, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x0800, 0xFFFF, 24, GOUT_ETHTYP, 1, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x86DD, 0xFFFF, 25, GOUT_ETHTYP, 1, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x8863, 0xFFFF, 19, GOUT_ETHTYP, 1, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x0000, 0xF800, 13, GOUT_NONE,   0, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x0000, 0x0000, 44, GOUT_ETHTYP, 1, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x0600, 0x0600, 44, GOUT_ETHTYP, 1, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x0000, 0x0000, 15, GOUT_NONE,   1, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0xAAAA, 0xFFFF, 17, GOUT_NONE,   1, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x0300, 0xFF00, 45, GOUT_NONE,   0, INSTR,   GFLAG_SNAP,  0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_DIP7,   3, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x0000, 0x0000, 21, GOUT_DIP7,   3, INSTR,   GFLAG_PPPOE, 0),
	PCE_MC_M(0x0021, 0xFFFF, 24, GOUT_NONE,   1, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x0057, 0xFFFF, 25, GOUT_NONE,   1, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x0000, 0x0000, 44, GOUT_NONE,   0, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x4000, 0xF000, 27, GOUT_IP0,    4, INSTR,   GFLAG_IPV4,  1),
	PCE_MC_M(0x6000, 0xF000, 30, GOUT_IP0,    3, INSTR,   GFLAG_IPV6,  0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x0000, 0x0000, 28, GOUT_IP3,    2, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x0000, 0x0000, 29, GOUT_SIP0,   4, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x0000, 0x0000, 44, GOUT_NONE,   0, LENACCU, GFLAG_NO,    0),
	PCE_MC_M(0x1100, 0xFF00, 43, GOUT_PROT,   1, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x0600, 0xFF00, 43, GOUT_PROT,   1, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x0000, 0xFF00, 36, GOUT_IP3,   17, INSTR,   GFLAG_HOP,   0),
	PCE_MC_M(0x2B00, 0xFF00, 36, GOUT_IP3,   17, INSTR,   GFLAG_NN1,   0),
	PCE_MC_M(0x3C00, 0xFF00, 36, GOUT_IP3,   17, INSTR,   GFLAG_NN2,   0),
	PCE_MC_M(0x0000, 0x0000, 43, GOUT_PROT,   1, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x0000, 0x00F0, 38, GOUT_NONE,   0, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x0000, 0x0000, 44, GOUT_NONE,   0, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x0000, 0xFF00, 36, GOUT_NONE,   0, IPV6,    GFLAG_HOP,   0),
	PCE_MC_M(0x2B00, 0xFF00, 36, GOUT_NONE,   0, IPV6,    GFLAG_NN1,   0),
	PCE_MC_M(0x3C00, 0xFF00, 36, GOUT_NONE,   0, IPV6,    GFLAG_NN2,   0),
	PCE_MC_M(0x0000, 0x00FC, 44, GOUT_PROT,   0, IPV6,    GFLAG_NO,    0),
	PCE_MC_M(0x0000, 0x0000, 44, GOUT_NONE,   0, IPV6,    GFLAG_NO,    0),
	PCE_MC_M(0x0000, 0x0000, 44, GOUT_SIP0,  16, INSTR,   GFLAG_NO,    0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_APP0,   4, INSTR,   GFLAG_IGMP,  0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_END,   0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_END,   0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_END,   0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_END,   0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_END,   0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_END,   0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_END,   0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_END,   0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_END,   0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_END,   0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_END,   0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_END,   0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_END,   0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_END,   0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_END,   0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_END,   0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_END,   0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_END,   0),
	PCE_MC_M(0x0000, 0x0000, 45, GOUT_NONE,   0, INSTR,   GFLAG_END,   0),
};

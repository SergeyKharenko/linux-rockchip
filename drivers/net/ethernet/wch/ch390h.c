/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * SPI ethernet driver for SPI to 100Mbps ethernet chip ch390.
 *
 * Copyright (C) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * 				http://wch.cn
 * Author:   	WCH <tech@wch.cn>
 * Contributor: Sergey Kharenko <skharenko@hust.edu.cn>
 */

#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/crc32.h>
#include <linux/version.h>

#include "ch390.h"

#define DRVNAME_CH390H "ch390h"

#ifdef CONFIG_WCH_CH390_DEBUG
struct reg_label {
	char *name;
	unsigned int reg;
};

#define REG_LABEL(REG) { #REG, REG }

static struct reg_label reg_labels[] = {
	REG_LABEL(CH390_NCR),
	REG_LABEL(CH390_NSR),
	REG_LABEL(CH390_TCR),
	REG_LABEL(CH390_TSRA),
	REG_LABEL(CH390_TSRB),
	REG_LABEL(CH390_RCR),
	REG_LABEL(CH390_RSR),
	REG_LABEL(CH390_ROCR),
	REG_LABEL(CH390_BPTR),
	REG_LABEL(CH390_FCTR),
	REG_LABEL(CH390_FCR),
	REG_LABEL(CH390_GPR),
	REG_LABEL(CH390_ATCR),
	REG_LABEL(CH390_RCSCSR),
	REG_LABEL(CH390_INTCR),
	REG_LABEL(CH390_ALNCR),
	REG_LABEL(CH390_ISR),
	REG_LABEL(CH390_IMR),
};
#endif

/*
 * struct board_info - maintain the saved data
 * @spidev: spi device structure
 * @ndev: net device structure
 * @mdiobus: mii bus structure
 * @phydev: phy device structure
 * @txq: tx queue structure
 * @rxctrl_work: Work queue for updating RX mode and multicast lists
 * @tx_work: Work queue for tx packets
 * @pause: ethtool pause parameter structure
 * @spi_lockm: between threads lock structure
 * @reg_mutex: reg write/read lock structure
 * @bc: rx control statistics structure
 * @rxhdr: rx header structure
 * @rctl: rx control setting structure
 * @msg_enable: message level value
 * @hash_table: to store operating hash table
 * @imr_all: to store operating imr value for register ch390_IMR
 * @lcr_all: to store operating lcmr value for register ch390_LMCR
 * @rcr_all: to store operating rcr value for register ch390_RCR
 * The saved data variables, keep up to date for retrieval back to use
 */
struct board_info {
	u32 msg_enable;
	struct spi_device *spidev;
	struct net_device *ndev;
	struct mii_bus *mdiobus;
	struct phy_device *phydev;
	struct sk_buff_head txq;
	struct work_struct rxctrl_work;
	struct work_struct tx_work;
	struct ethtool_pauseparam pause;
	struct mutex spi_lockm;
	struct mutex reg_mutex;
	struct ch390_op_stats bc;
	struct ch390_rxhdr rxhdr;
	u8 hash_table[8];
	u8 imr_all;
	u8 lcr_all;
	u8 rcr_all;
	bool has_eeprom;
};

static inline int ch390h_set_reg(struct board_info *db, u8 reg, u8 val)
{
	int ret;
	
	reg |= OPC_REG_W;
	struct spi_transfer trans[] = {
		{
			.tx_buf = &reg,
			.len = 1,
			.cs_change = 0
		},
		{
			.tx_buf = &val,
			.len = 1,
			.cs_change = 1
		}
	};

	mutex_lock(&db->reg_mutex);
	ret = spi_sync_transfer(db->spidev,trans,2);
	mutex_unlock(&db->reg_mutex);

	return ret;
}

static inline int ch390h_get_reg(struct board_info *db, u8 reg, void *val)
{
	int ret;

	reg |= OPC_REG_R;
	struct spi_transfer trans[] = {
		{
			.tx_buf = &reg,
			.len = 1,
			.cs_change = 0
		},
		{
			.rx_buf = val,
			.len = 1,
			.cs_change = 1
		}
	};
	
	mutex_lock(&db->reg_mutex);
	ret = spi_sync_transfer(db->spidev,trans,2);
	mutex_unlock(&db->reg_mutex);

	return ret;
}

static inline int ch390h_write_mem(struct board_info *db, const void *buff, size_t len)
{
	int ret;
	u8 reg = OPC_MEM_WRITE;
	struct spi_transfer trans[] = {
		{
			.tx_buf = &reg,
			.len = 1,
			.cs_change = 0
		},
		{
			.tx_buf = buff,
			.len = len,
			.cs_change = 1
		}
	};

	mutex_lock(&db->reg_mutex);
	ret = spi_sync_transfer(db->spidev,trans,2);
	mutex_unlock(&db->reg_mutex);

	return ret;
}

static inline int ch390h_read_mem(struct board_info *db, void *buff, size_t len)
{
	int ret;
	u8 reg = OPC_MEM_READ;
	struct spi_transfer trans[] = {
		{
			.tx_buf = &reg,
			.len = 1,
			.cs_change = 0
		},
		{
			.rx_buf = buff,
			.len = len,
			.cs_change = 1
		}
	};

	mutex_lock(&db->reg_mutex);
	ret = spi_sync_transfer(db->spidev,trans,2);
	mutex_unlock(&db->reg_mutex);

	return ret;
}

static int ch390h_drop_frame(struct board_info *db, size_t count)
{
	int ret;
	u8 mrrh, mrrl;

	CH390_RETURN_ON_ERROR(ch390h_get_reg(db, CH390_MRRH, &mrrh), "read MRRH failed\n");
	CH390_RETURN_ON_ERROR(ch390h_get_reg(db, CH390_MRRL, &mrrl), "read MRRL failed\n");
	
	u16 addr = mrrh << 8 | mrrl;
	addr = le16_to_cpu(addr);
	addr += count;
	addr = addr < 0x4000 ? addr : addr - 0x3400;
	addr = cpu_to_le16(addr);

	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_MRRH, addr >>8 ), "write MRRH failed\n");
	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_MRRL, addr & 0xFF ), "write MRRL failed\n");
	return ret;
}

static int ch390h_epcr_poll(struct board_info *db)
{
	int ret;
	u8 cnt = 0;
	u8 epcr;

	while (cnt < 100) {
		CH390_RETURN_ON_ERROR(ch390h_get_reg(db, CH390_EPCR, &epcr),"read EPCR failed\n");

		if (!(epcr & EPCR_ERRE))
			return 0;
		usleep_range(50, 100);
		cnt ++;
	}

	netdev_err(db->ndev, "eeprom/phy in processing get timeout\n");
	return -ETIMEDOUT;
}

static int ch390h_irq_flag(struct board_info *db)
{
	struct spi_device *spi = db->spidev;
	int irq_type = irq_get_trigger_type(spi->irq);

	if (irq_type)
		return irq_type;

	return IRQF_TRIGGER_HIGH;
}

static unsigned int ch390h_intcr_value(struct board_info *db)
{
	return (ch390h_irq_flag(db) == IRQF_TRIGGER_LOW) ? INCR_POL_L : INCR_POL_H;
}

static int ch390h_set_recv(struct board_info *db)
{
	int ret;

	for(int i = 0; i<8 ; i++){
		CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_MAR + i, db->hash_table[i]), "write PAR failed\n");
	}
	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_RCR, db->rcr_all),"write RCR failed\n");

	return ret;
}

static int ch390h_core_reset(struct board_info *db)
{
	int ret;

	db->bc.fifo_rst_cnt++;

	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_NCR, NCR_RST), "write NCR failed\n"); /* NCR reset */
	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_MLEDCR, db->lcr_all), "write MLEDCR failed\n"); /* LEDMode1 */
	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_INTCR, ch390h_intcr_value(db)), "write INTCR failed\n");

	return ret;
}

static int ch390h_update_fcr(struct board_info *db)
{
	int ret;
	u8 fcr;

	CH390_RETURN_ON_ERROR(ch390h_get_reg(db, CH390_FCR, &fcr), "read FCR failed\n");

	if (db->pause.rx_pause)
		fcr |= FCR_BKPM | FCR_FLCE;
	else
	 	fcr &= ~(FCR_BKPM | FCR_FLCE);

	if (db->pause.tx_pause)
		fcr |= FCR_TXPEN;
	else
	 	fcr &= ~FCR_TXPEN;

	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_FCR, fcr),"write FCR failed\n");

	return ret;
}

static int ch390h_disable_interrupt(struct board_info *db)
{
	int ret;
	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_IMR, IMR_PAR), "write IMR failed\n");
	return ret;
}

static int ch390h_enable_interrupt(struct board_info *db)
{
	int ret;
	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_IMR, IMR_ALL), "write IMR failed\n");
	return ret;
}

static int ch390h_clear_interrupt(struct board_info *db)
{
	int ret;
	u8 status;

	CH390_RETURN_ON_ERROR(ch390h_get_reg(db, CH390_ISR, &status),"read ISR failed\n");
	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_ISR, status),"write ISR failed\n");

	return ret;
}

static int ch390h_eeprom_read(struct board_info *db, int offset, u16 *data)
{
	int ret;
	u8 epdrl, epdrh;

	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_EPAR, offset),"write EPAR failed\n");
	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_EPCR, EPCR_ERPRR),"write EPCR failed\n");
	
	ret = ch390h_epcr_poll(db);
	if (ret)
		return ret;

	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_EPCR, 0),"write EPCR failed\n");
	CH390_RETURN_ON_ERROR(ch390h_get_reg(db, CH390_EPDRL, &epdrl), "read EPDRL failed\n");
	CH390_RETURN_ON_ERROR(ch390h_get_reg(db, CH390_EPDRH, &epdrh), "read EPDRH failed\n");

	*data=le16_to_cpu(epdrh<<8|epdrl);
	return ret;
}

static int ch390h_eeprom_write(struct board_info *db, int offset, u16 data)
{
	int ret;
	
	data = cpu_to_le16(data);

	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_EPAR, offset),"write EPAR failed\n");
	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_EPDRL, data & 0xFF), "write EPDRL failed\n");
	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_EPDRH, data >> 8), "write EPDRH failed\n");

	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_EPCR, EPCR_WEP | EPCR_ERPRW), "write EPCR failed\n");

	ret = ch390h_epcr_poll(db);
	if (ret)
		return ret;

	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_EPCR, 0), "write EPCR failed\n");

	return ret;
}

static int ch390h_phyread(void *context, u8 reg, u16 *data)
{
	struct board_info *db = context;
	int ret;
	u8 epdrl, epdrh;

	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_EPAR, CH390_PHY | reg), "write EPAR failed\n");
	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_EPCR, EPCR_ERPRR | EPCR_EPOS), "write EPCR failed\n");

	ret = ch390h_epcr_poll(db);
	if (ret)
		return ret;

	*data = 0;

	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_EPCR, 0), "write EPCR failed\n");
	CH390_RETURN_ON_ERROR(ch390h_get_reg(db, CH390_EPDRL, &epdrl), "read EPDRL failed\n");
	CH390_RETURN_ON_ERROR(ch390h_get_reg(db, CH390_EPDRH, &epdrh), "read EPDRH failed\n");

	*data=le16_to_cpu(epdrh<<8|epdrl);

	return ret;
}

static int ch390h_phywrite(void *context, u8 reg, u16 data)
{
	struct board_info *db = context;
	int ret;

	data = cpu_to_le16(data);
	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_EPAR, CH390_PHY | reg), "write EPAR failed\n");
	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_EPDRL, data & 0xFF), "write EPDRL failed\n");
	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_EPDRH, data >> 8), "write EPDRH failed\n");

	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_EPCR, EPCR_EPOS | EPCR_ERPRW),"write EPCR failed\n");

	ret = ch390h_epcr_poll(db);
	if (ret)
		return ret;

	CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_EPCR, 0),"write EPCR failed\n");
	return ret;
}

static int ch390h_mdio_read(struct mii_bus *bus, int addr, int regnum)
{
	struct board_info *db = bus->priv;
	int ret;
	u16 val;

	if (addr == CH390_PHY_ADDR) 
		CH390_RETURN_ON_ERROR(ch390h_phyread(db, regnum, &val),"read phy failed\n");

	return (int)val;
}

static int ch390h_mdio_write(struct mii_bus *bus, int addr, int regnum, u16 val)
{
	struct board_info *db = bus->priv;
	int ret;

	if (addr == CH390_PHY_ADDR)
		CH390_RETURN_ON_ERROR(ch390h_phywrite(db, regnum, val),"write phy failed\n");

	return -ENODEV;
}

static int ch390h_verify_id(struct board_info *db)
{
	struct device *dev = &db->spidev->dev;
	int ret;
	u8 id[2];
	u16 pid, vid;

	CH390_RETURN_ON_ERROR(ch390h_get_reg(db, CH390_VIDL, &id[0]), "read VIDL failed\n");
	CH390_RETURN_ON_ERROR(ch390h_get_reg(db, CH390_VIDH, &id[1]), "read VIDH failed\n");
	vid = le16_to_cpu(id[1]<<8|id[0]);
	if(vid!=CH390_VID) {
		dev_err(dev, "dev vid error as %04x !\n", vid);
		return -ENODEV;
	}
	
	CH390_RETURN_ON_ERROR(ch390h_get_reg(db, CH390_PIDL, &id[0]), "read PIDL failed\n");
	CH390_RETURN_ON_ERROR(ch390h_get_reg(db, CH390_PIDH, &id[1]), "read PIDH failed\n");
	pid = le16_to_cpu(id[1]<<8|id[0]);
	if(pid!=CH390H_PID) {
		dev_err(dev, "dev pid error as %04x !\n", pid);
		return -ENODEV;
	}

	dev_info(dev, "chip %04x found\n", pid);
	return 0;
}

/* 
 * Read CH390_PAR registers which is the mac address loaded from EEPROM while power-on
 */
static int ch390h_map_etherdev_par(struct net_device *ndev, struct board_info *db)
{
	u8 addr[ETH_ALEN];
	int ret;

	for(int i=0;i<ETH_ALEN;i++){
		CH390_RETURN_ON_ERROR(ch390h_get_reg(db, CH390_PAR +i,addr+i), "read PAR failed\n");
	}

	if (!is_valid_ether_addr(addr)) {
		eth_hw_addr_random(ndev);

		for(int i=0;i<ETH_ALEN;i++){
			CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_PAR +i,(ndev->dev_addr)[i]), "write PAR failed\n");
		}

		dev_dbg(&db->spidev->dev, "Use random MAC address\n");
	}
	else
		eth_hw_addr_set(ndev, addr);

	return 0;
}

/*
 * ethtool-ops
 */
static void ch390h_get_drvinfo(struct net_device *ndev, struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, DRVNAME_CH390H, sizeof(info->driver));
}

static void ch390h_set_msglevel(struct net_device *ndev, u32 value)
{
	struct board_info *db = to_ch390_board(ndev);

	db->msg_enable = value;
}

static u32 ch390h_get_msglevel(struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);

	return db->msg_enable;
}

static int ch390h_get_eeprom_len(struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);
	if(db->has_eeprom)
		return 128;
	else
	 	return 0;
}

static int ch390h_get_eeprom(struct net_device *ndev, struct ethtool_eeprom *ee, u8 *data)
{
	struct board_info *db = to_ch390_board(ndev);
	if(!db->has_eeprom)
		return -ENXIO;

	int offset = ee->offset;
	int len = ee->len;
	int ret;

	if ((len | offset) & 1)
		return -EINVAL;

	ee->magic = CH390_EEPROM_MAGIC;

	while(len>0){
		CH390_RETURN_ON_ERROR(ch390h_eeprom_read(db, offset / 2, (u16 *)data), "read eeprom failed\n");
		data += 2;
		offset += 2;
		len -= 2;
	}
	return ret;
}

static int ch390h_set_eeprom(struct net_device *ndev, struct ethtool_eeprom *ee, u8 *data)
{
	struct board_info *db = to_ch390_board(ndev);
	if(!db->has_eeprom)
		return -ENXIO;

	int offset = ee->offset;
	int len = ee->len;
	int i, ret;

	if ((len | offset) & 1)
		return -EINVAL;

	if (ee->magic != CH390_EEPROM_MAGIC)
		return -EINVAL;

	while(len>0){
		CH390_RETURN_ON_ERROR(ch390h_eeprom_write(db, offset / 2, *(u16 *)data), "write eeprom failed\n");
		data += 2;
		offset += 2;
		len -= 2;
		if(len==1){
			CH390_RETURN_ON_ERROR(ch390h_eeprom_write(db, offset / 2, (u16)(*data)), "write eeprom failed\n");
			break;
		}
	}

	return ret;
}

static void ch390h_get_pauseparam(struct net_device *ndev, struct ethtool_pauseparam *pause)
{
	struct board_info *db = to_ch390_board(ndev);

	*pause = db->pause;
}

static int ch390h_set_pauseparam(struct net_device *ndev, struct ethtool_pauseparam *pause)
{
	struct board_info *db = to_ch390_board(ndev);
	int advertise = 0;

	db->pause = *pause;

	if (pause->autoneg == AUTONEG_DISABLE)
		return ch390h_update_fcr(db);

	(void)advertise;
	phy_set_sym_pause(db->phydev, pause->rx_pause, pause->tx_pause, pause->autoneg);
	phy_start_aneg(db->phydev);

	return 0;
}

static const struct ethtool_ops ch390h_ethtool_ops = {
	.get_drvinfo = ch390h_get_drvinfo,
	.get_link_ksettings = phy_ethtool_get_link_ksettings,
	.set_link_ksettings = phy_ethtool_set_link_ksettings,
	.get_msglevel = ch390h_get_msglevel,
	.set_msglevel = ch390h_set_msglevel,
	.nway_reset = phy_ethtool_nway_reset,
	.get_link = ethtool_op_get_link,
	.get_eeprom_len = ch390h_get_eeprom_len,
	.get_eeprom = ch390h_get_eeprom,
	.set_eeprom = ch390h_set_eeprom,
	.get_pauseparam = ch390h_get_pauseparam,
	.set_pauseparam = ch390h_set_pauseparam,
};

static int ch390h_all_start(struct board_info *db)
{
	int ret;

	ret = ch390h_core_reset(db);
	if (ret)
		return ret;

	/* After ch390h_core_reset phy must be reopen */
	ret = ch390h_set_reg(db, CH390_GPR, 0);
	if (ret)
		return ret;

	msleep(1);

	return ch390h_enable_interrupt(db);
}

static int ch390h_all_stop(struct board_info *db)
{
	int ret;

	/*
	 * GPR power off of the internal phy,
	 * the internal phy still could be accessed after this GPR power off control
	 */
	ret = ch390h_set_reg(db, CH390_GPR, GPR_PHYPD);
	if (ret)
		return ret;

	return ch390h_set_reg(db, CH390_RCR, RCR_DIS_CRC);
}

/*
 * read packets from the fifo memory
 * return value:
 *  > 0 - read packet number, caller can repeat the rx operation
 *    0 - no error, caller need stop further rx operation
 *  -EBUSY - read data error, caller escape from rx operation
 */
static int ch390h_loop_rx(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	u8 rxbyte;
	int ret, rxlen;
	struct sk_buff *skb;
	u8 *rdptr;
	int scanrr = 0;

	do {		
		ret = ch390h_get_reg(db, OPC_MEM_DMY_R, &rxbyte);
		if (ret)
			return ret;

		ret = ch390h_get_reg(db, OPC_MEM_DMY_R, &rxbyte);
		if (ret)
			return ret;

		if (rxbyte & CH390_PKT_ERR) {
			ret = ch390h_set_reg(db, CH390_RCR, 0);
			if (ret)
				return ret;

        	ret = ch390h_set_reg(db, CH390_MPTRCR, 0x01);
			if (ret)
				return ret;

        	ret = ch390h_set_reg(db, CH390_MRRH, 0x0c);
			if (ret)
				return ret;

			msleep(1);
        	ret = ch390h_set_reg(db, CH390_RCR, RCR_RXEN);
			if (ret)
				return ret;

			return scanrr; /* packet-erro */
		}

		if (rxbyte != CH390_PKT_RDY)
			break; /* exhaust-empty */

		ret = ch390h_read_mem(db, &db->rxhdr, sizeof(struct ch390_rxhdr));
		if (ret)
			return ret;

		rxlen = le16_to_cpu(db->rxhdr.rxlen);

		skb = dev_alloc_skb(rxlen);
		if (!skb) {
			ret = ch390h_drop_frame(db, rxlen);
			if (ret)
				return ret;
			return scanrr;
		}

		rdptr = skb_put(skb, rxlen - ETH_FCS_LEN);

		if (rxlen <= CH390_PKT_MAX) {
			ret = ch390h_read_mem(db, rdptr, rxlen);
			if (ret) {
				db->bc.rx_err_cnt++;
				dev_kfree_skb(skb);
				return ret;
			}
		}

		if (db->rxhdr.status & RSR_ERR_BITS || rxlen > CH390_PKT_MAX) {
			netdev_dbg(ndev, "rxhdr-byte (%02x)\n",
				   db->rxhdr.headbyte);

			if (db->rxhdr.status & RSR_ERR_BITS) {
				db->bc.status_err_cnt++;
				netdev_dbg(ndev, "check rxstatus-error (%02x)\n",
					   db->rxhdr.status);
			} else {
				db->bc.large_err_cnt++;
				netdev_dbg(ndev, "check rxlen large-error (%d > %d)\n",
					   rxlen, CH390_PKT_MAX);
			}

			return scanrr;
		}

		skb->protocol = eth_type_trans(skb, db->ndev);
		if (db->ndev->features & NETIF_F_RXCSUM)
			skb_checksum_none_assert(skb);
		netif_rx(skb);
		db->ndev->stats.rx_bytes += rxlen;
		db->ndev->stats.rx_packets++;
		scanrr++;
	} while (!ret);

	return scanrr;
}

/* 
 * transmit a packet
 * return value:
 *   0 - succeed
 *  -ETIMEDOUT - timeout error
 */
static int ch390h_single_tx(struct board_info *db, u8 *buff, unsigned int len)
{
	int ret;
	unsigned int temp_low = len & 0xff;
	unsigned int temp_high = (len >> 8) & 0xff;
	u8 val, temp;

	ret = ch390h_write_mem(db, buff, len);
	if (ret)
		return ret;

	do {
		ret = ch390h_get_reg(db, CH390_TCR, &temp);
		if (ret)
			return ret;
	} while (temp & TCR_TXREQ);

	ret = ch390h_set_reg(db, CH390_TXPLL, temp_low);
	if (ret < 0)
		return ret;
	
	ret = ch390h_set_reg(db, CH390_TXPLH, temp_high);
	if (ret < 0)
		return ret;

	ret = ch390h_get_reg(db, CH390_TCR, &val);
	if (ret < 0)
		return ret;

	return ch390h_set_reg(db, CH390_TCR, val | TCR_TXREQ);
}

static int ch390h_loop_tx(struct board_info *db)
{
    struct net_device *ndev = db->ndev;
    int ntx = 0;
    int ret;

    while (!skb_queue_empty(&db->txq)) {
        struct sk_buff *skb;
        unsigned int len;
        skb = skb_dequeue(&db->txq);

		if (skb) {
            ntx++;
            ret = ch390h_single_tx(db, skb->data, skb->len);
            len = skb->len;
            dev_kfree_skb(skb);

            if (ret < 0) {
                db->bc.tx_err_cnt++;
                return 0;
            }

            ndev->stats.tx_bytes += len;
            ndev->stats.tx_packets++;
        }

        if (netif_queue_stopped(ndev) && (skb_queue_len(&db->txq) < CH390_TX_QUE_LO_WATER))
            netif_wake_queue(ndev);
    }

    return ntx;
}

static irqreturn_t ch390h_rx_threaded_irq(int irq, void *pw)
{
	struct board_info *db = pw;
	int result, result_tx;

	mutex_lock(&db->spi_lockm);

	result = ch390h_disable_interrupt(db);
	if (result)
		goto out_unlock;

	result = ch390h_clear_interrupt(db);
	if (result)
		goto out_unlock;

	do {
		result = ch390h_loop_rx(db); /* threaded irq rx */
		if (result < 0)
			goto out_unlock;

		result_tx = ch390h_loop_tx(db); /* more tx better performance */
		if (result_tx < 0)
			goto out_unlock;
	} while (result > 0);

	ch390h_enable_interrupt(db);

	/*
	 * To exit and has mutex unlock while rx or tx error
	 */
out_unlock:
	mutex_unlock(&db->spi_lockm);

	return IRQ_HANDLED;
}

static void ch390h_tx_delay(struct work_struct *work)
{
	struct board_info *db = container_of(work, struct board_info, tx_work);
	int result;

	mutex_lock(&db->spi_lockm);

	result = ch390h_loop_tx(db);
	if (result < 0)
		netdev_err(db->ndev, "transmit packet error\n");

	mutex_unlock(&db->spi_lockm);
}

static void ch390h_rxctl_delay(struct work_struct *work)
{
	struct board_info *db = container_of(work, struct board_info, rxctrl_work);
	struct net_device *ndev = db->ndev;
	int ret;

	mutex_lock(&db->spi_lockm);

	for(int i=0;i<ETH_ALEN;i++){
		CH390_GOTO_ON_ERROR(ch390h_set_reg(db, CH390_PAR +i,(ndev->dev_addr)[i]), err, "write PAR failed\n");
	}

	ch390h_set_recv(db);

err:
	mutex_unlock(&db->spi_lockm);
}

/* 
 * Open network device
 * Called when the network device is marked active, such as a user executing
 * 'ifconfig up' on the device
 */
static int ch390h_open(struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);
	int ret;

	db->imr_all = IMR_PAR | IMR_PRI;
	db->lcr_all = MLEDCR_LED_MOD1;
	db->rcr_all = RCR_DIS_CRC | RCR_RXEN;
	memset(db->hash_table, 0, sizeof(db->hash_table));

	phy_support_sym_pause(db->phydev);
	phy_start(db->phydev);

	/* flow control parameters init */
	db->pause.rx_pause = true;
	db->pause.tx_pause = true;
	db->pause.autoneg = AUTONEG_DISABLE;

	if (db->phydev->autoneg)
		db->pause.autoneg = AUTONEG_ENABLE;

	ret = ch390h_all_start(db);
	if (ret) {
		phy_stop(db->phydev);
		return ret;
	}

	netif_wake_queue(ndev);

	return 0;
}

/*
 * Close network device
 * Called to close down a network device which has been active. Cancel any
 * work, shutdown the RX and TX process and then place the chip into a low
 * power state while it is not being used
 */
static int ch390h_stop(struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);
	int ret;

	ret = ch390h_all_stop(db);
	if (ret)
		return ret;

	flush_work(&db->tx_work);
	flush_work(&db->rxctrl_work);

	phy_stop(db->phydev);

	netif_stop_queue(ndev);

	skb_queue_purge(&db->txq);

	return 0;
}

/*
 * event: play a schedule starter in condition
 */
static netdev_tx_t ch390h_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);

	skb_queue_tail(&db->txq, skb);
	if (skb_queue_len(&db->txq) > CH390_TX_QUE_HI_WATER)
		netif_stop_queue(ndev); /* enforce limit queue size */

	schedule_work(&db->tx_work);

	return NETDEV_TX_OK;
}

/*
 * event: play with a schedule starter
 */
static void ch390h_set_rx_mode(struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);
	struct netdev_hw_addr *ha;
	u8 rcr = RCR_DIS_CRC | RCR_RXEN;
	u32 hash_val;
	u8 hash_table[8];

	/* rx control */
	if (ndev->flags & IFF_PROMISC) {
		rcr |= RCR_PRMSC;
		netdev_dbg(ndev, "set_multicast rcr |= RCR_PRMSC, rcr= %02x\n", rcr);
	}

	if (ndev->flags & IFF_ALLMULTI) {
		rcr |= RCR_ALL;
		netdev_dbg(ndev, "set_multicast rcr |= RCR_ALLMULTI, rcr= %02x\n", rcr);
	}

	/* broadcast address */
	hash_table[0] = 0;
	hash_table[1] = 0;
	hash_table[2] = 0;
	hash_table[3] = 0x8000;

	/* the multicast address in Hash Table : 64 bits */
	netdev_for_each_mc_addr(ha, ndev) {
		hash_val = crc32_le(~0, ha->addr, ETH_ALEN) & GENMASK(5, 0);
		hash_table[hash_val / 8] |= BIT(hash_val % 8);
	}

	/* schedule work to do the actual set of the data if needed */
	if (memcmp(db->hash_table, hash_table, sizeof(hash_table))|| db->rcr_all!=rcr) {
		memcpy(db->hash_table, hash_table, sizeof(hash_table));
		db->rcr_all = rcr;
		schedule_work(&db->rxctrl_work);
	}
}

/*
 * event: write into the mac registers and eeprom directly
 */
static int ch390h_set_mac_address(struct net_device *ndev, void *p)
{
	struct board_info *db = to_ch390_board(ndev);
	struct sockaddr *addr = p;
	int ret;

	if (!(ndev->priv_flags & IFF_LIVE_ADDR_CHANGE) && netif_running(ndev))
			return -EBUSY;
	if (!is_valid_ether_addr(addr->sa_data))
			return -EADDRNOTAVAIL;

	eth_commit_mac_addr_change(ndev, p);

	for(int i=0;i<ETH_ALEN;i++){
		CH390_RETURN_ON_ERROR(ch390h_set_reg(db, CH390_PAR +i,(ndev->dev_addr)[i]), "write PAR failed\n");
	}
	return ret;
}

static const struct net_device_ops ch390h_netdev_ops = {
	.ndo_open = ch390h_open,
	.ndo_stop = ch390h_stop,
	.ndo_start_xmit = ch390h_start_xmit,
	.ndo_set_rx_mode = ch390h_set_rx_mode,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_set_mac_address = ch390h_set_mac_address,
};

static void ch390h_operation_clear(struct board_info *db)
{
	db->bc.status_err_cnt = 0;
	db->bc.large_err_cnt = 0;
	db->bc.rx_err_cnt = 0;
	db->bc.tx_err_cnt = 0;
	db->bc.fifo_rst_cnt = 0;
}

static int ch390h_mdio_register(struct board_info *db)
{
	struct spi_device *spi = db->spidev;
	int ret;

	db->mdiobus = mdiobus_alloc();
	if (!db->mdiobus)
		return -ENOMEM;

	db->mdiobus->priv = db;
	db->mdiobus->read = ch390h_mdio_read;
	db->mdiobus->write = ch390h_mdio_write;
	db->mdiobus->name = "ch390h-mdiobus";
	db->mdiobus->phy_mask = (u32)~BIT(1);
	db->mdiobus->parent = &spi->dev;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 8, 0))
	snprintf(db->mdiobus->id, MII_BUS_ID_SIZE, "ch390-%s.%u", dev_name(&spi->dev), spi_get_chipselect(spi, 0));
#else
	snprintf(db->mdiobus->id, MII_BUS_ID_SIZE, "ch390-%s.%u", dev_name(&spi->dev), spi->chip_select);
#endif

	ret = mdiobus_register(db->mdiobus);
	if (ret) {
		netdev_err(db->ndev, "can't register MDIO bus\n");
		goto out;
	}

	return 0;
out:
	mdiobus_free(db->mdiobus);
	return ret;
}

static void ch390h_mdio_unregister(struct board_info *db)
{
	mdiobus_unregister(db->mdiobus);
	mdiobus_free(db->mdiobus);
}

static void ch390h_handle_link_change(struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);

	phy_print_status(db->phydev);

	/*
	 * only write pause settings to mac. since mac and phy are integrated
	 * together, such as link state, speed and duplex are sync already
	 */
	if (db->phydev->link) {
		if (db->phydev->pause) {
			db->pause.rx_pause = true;
			db->pause.tx_pause = true;
		}
		ch390h_update_fcr(db);
	}
}

/*
 * phy connect as poll mode
 */
static int ch390h_phy_connect(struct board_info *db)
{
	char phy_id[MII_BUS_ID_SIZE + 3];

	snprintf(phy_id, sizeof(phy_id), PHY_ID_FMT, db->mdiobus->id, CH390_PHY_ADDR);

	db->phydev = phy_connect(db->ndev, phy_id, ch390h_handle_link_change, PHY_INTERFACE_MODE_MII);
	if (IS_ERR(db->phydev))
		return PTR_ERR(db->phydev);
	return 0;
}

static int ch390h_request_irq(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	struct spi_device *spi = db->spidev;
	int ret;

	ndev->irq = spi->irq;

	ret = request_threaded_irq(spi->irq, NULL, ch390h_rx_threaded_irq, 
			ch390h_irq_flag(db) | IRQF_ONESHOT, ndev->name, db);
	if (ret < 0) {
		netdev_err(ndev, "failed to get irq!\n");
		goto out;
	}

out:
	return ret;
}

#ifdef CONFIG_WCH_CH390_DEBUG
static ssize_t reg_dump_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct board_info *db;
    int i, len = 0;
    u8 val;

    dev_info(dev, "reg_dump_show");
	if (!ndev) {
        dev_err(dev, "net_device is NULL\n");
        return -EINVAL;
    }

    db = netdev_priv(ndev);

    if (!db) {
        dev_err(dev, "board_info is NULL\n");
        return -EINVAL;
    }

    for (i = 0; i < sizeof(reg_labels) / sizeof(reg_labels[0]); i++) {
        if (ch390h_get_reg(db, reg_labels[i].reg, &val) != 0) {
            dev_err(dev, "Failed to read register %s\n", reg_labels[i].name);
            return -EIO;
        }
        len += sprintf(buf + len, "%s: 0x%02x\n", reg_labels[i].name, val);
    }

    return len;
}

static ssize_t reg_dump_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct board_info *db;
	unsigned int reg;
	u8 val;
	char reg_name[32];

	dev_info(dev, "reg_dump_store\n");
	if (!ndev) {
        dev_err(dev, "net_device is NULL\n");
        return -EINVAL;
    }

	db = netdev_priv(ndev);

	if (sscanf(buf, "%31s %02hhx", reg_name, &val) == 2) {
		int i;

		for (i = 0; i < sizeof(reg_labels) / sizeof(reg_labels[0]); i++) {
			if (strcmp(reg_labels[i].name, reg_name) == 0) {
				reg = reg_labels[i].reg;
				if (ch390h_set_reg(db, reg, val) < 0)
					dev_info(dev, "set reg: 0x%02x - value: 0x%02x filed!\n", reg, val);
				else
					dev_info(dev, "set reg: 0x%02x - value: 0x%02x success!\n", reg, val);
				break;
			}
		}
	}

	return count;
}

static DEVICE_ATTR(reg_dump, S_IRUGO | S_IWUSR, reg_dump_show, reg_dump_store);

static struct attribute *ch390h_attributes[] = { &dev_attr_reg_dump.attr, NULL };

static struct attribute_group ch390h_attribute_group = { .attrs = ch390h_attributes };

int ch390h_create_sysfs(struct spi_device *spi)
{
	int err;

	err = sysfs_create_group(&spi->dev.kobj, &ch390h_attribute_group);
	if (err != 0) {
		dev_err(&spi->dev, "sysfs_create_group() failed!!");
		sysfs_remove_group(&spi->dev.kobj, &ch390h_attribute_group);
		return -EIO;
	}

	err = sysfs_create_link(NULL, &spi->dev.kobj, "ch390h");
	if (err < 0) {
		dev_err(&spi->dev, "Failed to create link!");
		return -EIO;
	}

	dev_info(&spi->dev, "sysfs_create_group() succeeded!!");

	return err;
}
#endif

static int ch390h_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct net_device *ndev;
	struct board_info *db;
	int ret = 0;

	ndev = alloc_etherdev(sizeof(*db));
	if (!ndev)
		return -ENOMEM;

	SET_NETDEV_DEV(ndev, dev);
	dev_set_drvdata(dev, ndev);

	db = netdev_priv(ndev);

	db->msg_enable = 0;
	db->spidev = spi;
	db->ndev = ndev;

	ndev->netdev_ops = &ch390h_netdev_ops;
	ndev->ethtool_ops = &ch390h_ethtool_ops;

	mutex_init(&db->spi_lockm);
	mutex_init(&db->reg_mutex);

	INIT_WORK(&db->rxctrl_work, ch390h_rxctl_delay);
	INIT_WORK(&db->tx_work, ch390h_tx_delay);

	if (of_find_property(dev->of_node, "wch,eeprom", NULL))
		db->has_eeprom = true;
	else
	 	db->has_eeprom = false;

	ret = ch390h_verify_id(db);
	if (ret)
		goto err_map;

	ret = ch390h_map_etherdev_par(ndev, db);
	if (ret < 0)
		goto err_map;

	ret = ch390h_mdio_register(db);
	if (ret)
		goto err_reg_mdio;

	ret = ch390h_phy_connect(db);
	if (ret)
		goto err_phy_conn;

	ch390h_operation_clear(db);
	skb_queue_head_init(&db->txq);

	ret = register_netdev(ndev);
	if (ret) {
		dev_err(dev, "device register failed: %d\n", ret);
    	goto err_reg_nd;
	}

	ret = ch390h_request_irq(db);
	if (ret < 0) {
		dev_err(dev, "device request irq failed: %d\n", ret);
		goto err_req_irq;
	}

#ifdef CONFIG_WCH_CH390_DEBUG
	ch390h_create_sysfs(spi);
#endif

	return 0;

err_reg_nd:
	phy_disconnect(db->phydev);
err_phy_conn:
err_req_irq:
	ch390h_mdio_unregister(db);
err_map:
err_reg_mdio:
	free_netdev(ndev);
	return ret;
}

static void ch390h_drv_remove(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct net_device *ndev = dev_get_drvdata(dev);
	struct board_info *db = to_ch390_board(ndev);

	phy_disconnect(db->phydev);
	unregister_netdev (ndev);
	ch390h_mdio_unregister(db);
	free_netdev(ndev);
	free_irq(db->spidev->irq, db);

#ifdef CONFIG_WCH_CH390_DEBUG
	sysfs_remove_group(&spi->dev.kobj, &ch390h_attribute_group);
	sysfs_remove_link(NULL, "ch390");
#endif
}

static const struct of_device_id ch390h_match_table[] = { 
	{ .compatible = "wch, ch390h" }, 
	{ .compatible = "wch, ch390d" },
	{} 
};

static const struct spi_device_id ch390h_id_table[] = {
	{ "ch390h", 0 },
	{ "ch390d", 1 },
	{}
};

static struct spi_driver ch390h_driver = {
	.driver = {
		.name = DRVNAME_CH390H,
		.of_match_table = ch390h_match_table,
	},
	.probe = ch390h_probe,
	.remove = ch390h_drv_remove,
	.id_table = ch390h_id_table,
};
module_spi_driver(ch390h_driver);

MODULE_AUTHOR("Sergey Kharenko <skharenko@hust.edu.cn>");
MODULE_DESCRIPTION("SPI ethernet driver for CH390H/D");
MODULE_VERSION("1.2.0");
MODULE_LICENSE("GPL");

/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * CH390H/D SPI Ethernet driver
 *
 * Driver for the CH390H/D SPI to 100Mbps Ethernet controller.
 *
 * Copyright (C) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Author:       WCH <tech@wch.cn>
 * Maintainer:   Sergey Kharenko <skharenko@hust.edu.cn>
 *
 * This driver provides support for the CH390H/D Ethernet controller
 * connected via SPI. It integrates with the Linux networking stack
 * through the standard net_device interface.
 */

#include <linux/crc32.h>
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
#include <linux/version.h>

#include "ch390.h"

#define DRVNAME_CH390H "ch390h"

#ifdef CONFIG_WCH_CH390_DEBUG
static struct ch390_reg_label reg_labels[] = {
	CH390_DEBUG_REG_LIST(CH390_REG_LABEL_GEN)
};
#endif

/**
 * struct board_info - CH390H private network device data structure
 * @spidev:          Pointer to the SPI device structure.
 * @ndev:            Pointer to the network device structure.
 * @mdiobus:         Pointer to the MII bus for PHY communication.
 * @phydev:          Pointer to the PHY device structure.
 * @txq:             Queue for outgoing network packets (sk_buffs).
 * @async_tx_work:   Work queue item for asynchronous packet transmission.
 * @async_rx_work:   Work queue item for asynchronous packet reception.
 * @async_rx_mode_work: Work queue item for applying new receiver mode settings.
 * @pause:           Stores the current ethtool pause (flow control) settings.
 * @spi_lockm:       Mutex to protect against concurrent access to the SPI bus.
 * @reg_mutex:       Mutex to protect against concurrent access to CH390H registers.
 * @stats:           Network statistics (64-bit).
 * @syncp:           Synchronization for safely updating 64-bit statistics.
 * @hash_table:      A cache for the 64-bit multicast hash table.
 * @imr_all:         A cache for the Interrupt Mask Register (IMR) value.
 * @lcr_all:         A cache for the LED Control Register (MLEDCR) value.
 * @rcr_all:         A cache for the Receive Control Register (RCR) value.
 * @has_eeprom:      Flag indicating if an EEPROM is present on the board.
 * @irq_posedge:     Flag indicating if the interrupt is rising-edge triggered.
 */
struct board_info {
	u32 msg_enable;
	struct spi_device *spidev;
	struct net_device *ndev;
	struct mii_bus *mdiobus;
	struct phy_device *phydev;
	struct sk_buff_head txq;
	struct work_struct async_tx_work;
	struct work_struct async_rx_work;
	struct work_struct async_rx_mode_work;
	struct ethtool_pauseparam pause;
	struct mutex spi_lockm;
	struct mutex reg_mutex;
	struct rtnl_link_stats64 stats;
	struct u64_stats_sync syncp;
	u8 hash_table[8];
	u8 imr_all;
	u8 lcr_all;
	u8 rcr_all;
	bool has_eeprom;
	bool irq_posedge;
	bool rxcsum;
};

/**
 * ch390h_io_register_write - Write a byte to a CH390H register.
 * @db:  Pointer to the driver's private data structure.
 * @reg: The address of the register to write.
 * @val: The value to write to the register.
 *
 * This is a low-level function for writing to an internal register of the
 * CH390H via the SPI interface. Access to the register is protected by a mutex.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static inline int ch390h_io_register_write(struct board_info *db, u8 reg, u8 val)
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

/**
 * ch390h_io_register_read - Read a byte from a CH390H register.
 * @db:  Pointer to the driver's private data structure.
 * @reg: The address of the register to read.
 * @val: Pointer to a u8 to store the read value.
 *
 * This is a low-level function for reading from an internal register of the
 * CH390H via the SPI interface. Access to the register is protected by a mutex.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static inline int ch390h_io_register_read(struct board_info *db, u8 reg, void *val)
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

/**
 * ch390h_io_memory_write - Write a block of data to CH390H's internal memory.
 * @db:   Pointer to the driver's private data structure.
 * @buff: Pointer to the data buffer to write.
 * @len:  The number of bytes to write.
 *
 * Writes data to the CH390H's internal RAM, typically for a transmit packet.
 * Access is protected by a mutex.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static inline int ch390h_io_memory_write(struct board_info *db, const void *buff, size_t len)
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

/**
 * ch390h_io_memory_read - Read a block of data from CH390H's internal memory.
 * @db:   Pointer to the driver's private data structure.
 * @buff: Pointer to a buffer to store the read data.
 * @len:  The number of bytes to read.
 *
 * Reads data from the CH390H's internal RAM, typically for a received packet.
 * Access is protected by a mutex.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static inline int ch390h_io_memory_read(struct board_info *db, void *buff, size_t len)
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

/**
 * ch390h_epcr_busy_wait - Wait for an EEPROM or PHY operation to complete.
 * @db: Pointer to the driver's private data structure.
 *
 * Polls the EPCR_ERRE bit in the EPCR register to wait for the completion of an
 * EEPROM or PHY access operation.
 *
 * Return: 0 on success, or -ETIMEDOUT on timeout.
 */
static inline int ch390h_epcr_busy_wait(struct board_info *db)
{
	int ret;
	u8 cnt = 0;
	u8 epcr;

	while (cnt < 100) {
		CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_EPCR, &epcr),"read EPCR failed\n");
		if (!(epcr & EPCR_ERRE))
			return 0;
		usleep_range(50, 100);
		cnt ++;
	}
	netdev_err(db->ndev, "eeprom/phy in processing get timeout\n");
	return -ETIMEDOUT;
}

/**
 * ch390h_eeprom_read - Read a 16-bit word from the EEPROM.
 * @db:     Pointer to the driver's private data structure.
 * @offset: The word offset (not byte) in the EEPROM to read from.
 * @data:   Pointer to a u16 to store the read data.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390h_eeprom_read(struct board_info *db, int offset, u16 *data)
{
	int ret;
	u8 epdrl, epdrh;

	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_EPAR, offset),"write EPAR failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_EPCR, EPCR_ERPRR),"write EPCR failed\n");
	
	CH390_RETURN_ON_ERROR(ch390h_epcr_busy_wait(db),"read eeprom failed\n");

	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_EPCR, 0),"write EPCR failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_EPDRL, &epdrl), "read EPDRL failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_EPDRH, &epdrh), "read EPDRH failed\n");

	*data=le16_to_cpu(epdrh<<8|epdrl);
	return ret;
}

/**
 * ch390h_eeprom_write - Write a 16-bit word to the EEPROM.
 * @db:     Pointer to the driver's private data structure.
 * @offset: The word offset (not byte) in the EEPROM to write to.
 * @data:   The 16-bit data to write.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390h_eeprom_write(struct board_info *db, int offset, u16 data)
{
	int ret;
	
	data = cpu_to_le16(data);

	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_EPAR, offset),"write EPAR failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_EPDRL, data & 0xFF), "write EPDRL failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_EPDRH, data >> 8), "write EPDRH failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_EPCR, EPCR_WEP | EPCR_ERPRW), "write EPCR failed\n");

	CH390_RETURN_ON_ERROR(ch390h_epcr_busy_wait(db),"write eeprom failed\n");

	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_EPCR, 0), "write EPCR failed\n");
	return ret;
}

/**
 * ch390h_phyread - Read a register from the internal PHY.
 * @context: Pointer to the driver's private data structure.
 * @reg:     The PHY register address to read.
 * @data:    Pointer to a u16 to store the read value.
 *
 * This function is used as a backend for the MDIO read operation.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390h_phyread(void *context, u8 reg, u16 *data)
{
	struct board_info *db = context;
	int ret;
	u8 epdrl, epdrh;

	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_EPAR, CH390_PHY | reg), "write EPAR failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_EPCR, EPCR_ERPRR | EPCR_EPOS), "write EPCR failed\n");

	CH390_RETURN_ON_ERROR(ch390h_epcr_busy_wait(db),"read phy failed\n");

	*data = 0;

	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_EPCR, 0), "write EPCR failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_EPDRL, &epdrl), "read EPDRL failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_EPDRH, &epdrh), "read EPDRH failed\n");

	*data=le16_to_cpu(epdrh<<8|epdrl);

	return ret;
}

/**
 * ch390h_phywrite - Write a register to the internal PHY.
 * @context: Pointer to the driver's private data structure.
 * @reg:     The PHY register address to write.
 * @data:    The 16-bit value to write.
 *
 * This function is used as a backend for the MDIO write operation.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390h_phywrite(void *context, u8 reg, u16 data)
{
	struct board_info *db = context;
	int ret;

	data = cpu_to_le16(data);
	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_EPAR, CH390_PHY | reg), "write EPAR failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_EPDRL, data & 0xFF), "write EPDRL failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_EPDRH, data >> 8), "write EPDRH failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_EPCR, EPCR_EPOS | EPCR_ERPRW),"write EPCR failed\n");

	CH390_RETURN_ON_ERROR(ch390h_epcr_busy_wait(db),"write phy failed\n");

	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_EPCR, 0),"write EPCR failed\n");
	return ret;
}

/**
 * ch390h_mdio_read - MDIO bus read callback.
 * @bus:    Pointer to the mii_bus structure.
 * @addr:   The PHY address on the MDIO bus.
 * @regnum: The register offset to read.
 *
 * This function is registered with the MDIO subsystem to handle read requests.
 * It only responds to reads for this driver's internal PHY.
 *
 * Return: The 16-bit value read from the PHY register, or 0xFFFF on error.
 */
static int ch390h_mdio_read(struct mii_bus *bus, int addr, int regnum)
{
	struct board_info *db = bus->priv;
	int ret;
	u16 val = 0xFFFF;

	if (addr == CH390_PHY_ADDR) 
		CH390_RETURN_ON_ERROR(ch390h_phyread(db, regnum, &val),"read phy failed\n");
	return (int)val;
}

/**
 * ch390h_mdio_write - MDIO bus write callback.
 * @bus:    Pointer to the mii_bus structure.
 * @addr:   The PHY address on the MDIO bus.
 * @regnum: The register offset to write.
 * @val:    The 16-bit value to write.
 *
 * This function is registered with the MDIO subsystem to handle write requests.
 * It only responds to writes for this driver's internal PHY.
 *
 * Return: 0 on success, or -ENODEV if the PHY address is incorrect.
 */
static int ch390h_mdio_write(struct mii_bus *bus, int addr, int regnum, u16 val)
{
	struct board_info *db = bus->priv;
	int ret;

	if (addr == CH390_PHY_ADDR)
		CH390_RETURN_ON_ERROR(ch390h_phywrite(db, regnum, val),"write phy failed\n");

	return -ENODEV;
}

/**
 * ch390h_drop_frame - Discard the current received frame from RX buffer.
 * @db:  Pointer to the driver's private data structure.
 * @len: Length of the frame to discard.
 *
 * Advances the RX memory read pointer past the current frame, effectively
 * dropping it. Used when a received frame has errors or cannot be processed.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390h_drop_frame(struct board_info *db, size_t len)
{
	int ret;
	u8 mrrh, mrrl;

	CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_MRRH, &mrrh), "read MRRH failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_MRRL, &mrrl), "read MRRL failed\n");
	
	u16 addr = mrrh << 8 | mrrl;
	addr = le16_to_cpu(addr);
	addr += len;
	addr = addr < 0x4000 ? addr : addr - 0x3400;
	addr = cpu_to_le16(addr);

	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_MRRH, addr >>8 ), "write MRRH failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_MRRL, addr & 0xFF ), "write MRRL failed\n");
	return 0;
}

/**
 * ch390h_update_fcr - Update the Flow Control Register.
 * @db: Pointer to the driver's private data structure.
 *
 * Configures the hardware flow control settings in the FCR register based on
 * the cached values in `db->pause`.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390h_update_fcr(struct board_info *db)
{
	int ret;
	u8 fcr;

	CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_FCR, &fcr), "read FCR failed\n");
	if (db->pause.rx_pause)
		fcr |= FCR_BKPM | FCR_FLCE;
	else
	 	fcr &= ~(FCR_BKPM | FCR_FLCE);

	if (db->pause.tx_pause)
		fcr |= FCR_TXPEN;
	else
	 	fcr &= ~FCR_TXPEN;
	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_FCR, fcr),"write FCR failed\n");
	return 0;
}

/**
 * ch390h_verify_id - Verify the Chip's Vendor and Product ID.
 * @db: Pointer to the driver's private data structure.
 *
 * Reads the VID and PID registers to ensure that the correct chip is present.
 *
 * Return: 0 if IDs match, or -ENODEV if they do not.
 */
static int ch390h_verify_id(struct board_info *db)
{
	struct device *dev = &db->spidev->dev;
	int ret;
	u8 id[2];
	u8 chipr;
	u16 pid, vid;

	CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_VIDL, &id[0]), "read VIDL failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_VIDH, &id[1]), "read VIDH failed\n");
	vid = le16_to_cpu(id[1]<<8|id[0]);
	if(vid!=CH390_VID) {
		dev_err(dev, "dev vid error as %04x !\n", vid);
		return -ENODEV;
	}
	
	CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_PIDL, &id[0]), "read PIDL failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_PIDH, &id[1]), "read PIDH failed\n");
	pid = le16_to_cpu(id[1]<<8|id[0]);
	if(pid!=CH390H_PID) {
		dev_err(dev, "dev pid error as %04x !\n", pid);
		return -ENODEV;
	}

	CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_CHIPR, &chipr), "read CHIPR failed\n");
	dev_info(dev, "chip %02x found\n", chipr);
	return 0;
}

/**
 * ch390h_init_mac_addr - Initialize the device MAC address.
 * @ndev: Pointer to the network device structure.
 * @db:   Pointer to the driver's private data structure.
 *
 * Reads the MAC address from the hardware's PAR registers. If the address is
 * invalid, a random MAC address is generated and written back to the hardware.
 * The MAC address is then set for the net device.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390h_init_mac_addr(struct net_device *ndev, struct board_info *db)
{
	u8 addr[ETH_ALEN];
	int ret;

	for(int i=0;i<ETH_ALEN;i++){
		CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_PAR +i,addr+i), "read PAR failed\n");
	}

	if (!is_valid_ether_addr(addr)) {
		eth_hw_addr_random(ndev);

		for(int i=0;i<ETH_ALEN;i++){
			CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_PAR +i,(ndev->dev_addr)[i]), "write PAR failed\n");
		}
		dev_dbg(&db->spidev->dev, "Use random MAC address\n");
	}
	else
		eth_hw_addr_set(ndev, addr);
	return 0;
}

static int ch390h_init_hw_offload(struct board_info *db) 
{
	int ret;

	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_TCSCR, TCSCR_ALL), 
						"write TCSCR failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_RCSCSR, RCSCSR_RCSEN | RCSCSR_DCSE), 
						"write RCSCSR failed\n");
	return 0;
} 

/*
 * Ethtool operations
 */

/**
 * ch390h_get_drvinfo - Implementation of ethtool get_drvinfo.
 * @ndev: Pointer to the network device structure.
 * @info: Pointer to the ethtool_drvinfo structure to be filled.
 */
static void ch390h_get_drvinfo(struct net_device *ndev, struct ethtool_drvinfo *info)
{
	strlcpy(info->driver, DRVNAME_CH390H, sizeof(info->driver));
}

/**
 * ch390h_set_msglevel - Implementation of ethtool set_msglevel.
 * @ndev:  Pointer to the network device structure.
 * @value: The new message level.
 */
static void ch390h_set_msglevel(struct net_device *ndev, u32 value)
{
	struct board_info *db = to_ch390_board(ndev);

	db->msg_enable = value;
}

/**
 * ch390h_get_msglevel - Implementation of ethtool get_msglevel.
 * @ndev: Pointer to the network device structure.
 *
 * Return: The current message level.
 */
static u32 ch390h_get_msglevel(struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);

	return db->msg_enable;
}

/**
 * ch390h_get_eeprom_len - Implementation of ethtool get_eeprom_len.
 * @ndev: Pointer to the network device structure.
 *
 * Return: The EEPROM size (128 bytes) if present, otherwise 0.
 */
static int ch390h_get_eeprom_len(struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);
	if(db->has_eeprom)
		return 128;
	else
	 	return 0;
}

/**
 * ch390h_get_eeprom - Implementation of ethtool get_eeprom.
 * @ndev: Pointer to the network device structure.
 * @ee:   Pointer to the ethtool_eeprom structure with read parameters.
 * @data: Buffer to store the read EEPROM data.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
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
	return 0;
}

/**
 * ch390h_set_eeprom - Implementation of ethtool set_eeprom.
 * @ndev: Pointer to the network device structure.
 * @ee:   Pointer to the ethtool_eeprom structure with write parameters.
 * @data: Buffer containing the data to write to the EEPROM.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390h_set_eeprom(struct net_device *ndev, struct ethtool_eeprom *ee, u8 *data)
{
	struct board_info *db = to_ch390_board(ndev);
	if(!db->has_eeprom)
		return -ENXIO;

	int offset = ee->offset;
	int len = ee->len;
	int ret;

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
	return 0;
}

/**
 * ch390h_get_pauseparam - Implementation of ethtool get_pauseparam.
 * @ndev:  Pointer to the network device structure.
 * @pause: Pointer to the ethtool_pauseparam structure to be filled.
 */
static void ch390h_get_pauseparam(struct net_device *ndev, struct ethtool_pauseparam *pause)
{
	struct board_info *db = to_ch390_board(ndev);

	*pause = db->pause;
}

/**
 * ch390h_set_pauseparam - Implementation of ethtool set_pauseparam.
 * @ndev:  Pointer to the network device structure.
 * @pause: Pointer to the ethtool_pauseparam structure with new settings.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390h_set_pauseparam(struct net_device *ndev, struct ethtool_pauseparam *pause)
{
	struct board_info *db = to_ch390_board(ndev);

	db->pause = *pause;

	if (pause->autoneg == AUTONEG_DISABLE)
		return ch390h_update_fcr(db);

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

/**
 * ch390h_reset - Perform a software reset of the CH390H chip.
 * @db: Pointer to the driver's private data structure.
 *
 * This function initiates a software reset and waits for it to complete.
 *
 * Return: 0 on success, or -ETIMEDOUT on timeout.
 */
static int ch390h_reset(struct board_info *db)
{
	int ret;
	u8 cnt = 0;
	u8 ncr;

	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_NCR, NCR_RST), "write NCR failed\n"); /* NCR reset */

	while(cnt < 100){
		CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_NCR, &ncr), "read NCR failed\n");
		if(!(ncr & NCR_RST)){
			return 0;
		}
		cnt++;
		msleep(1);
	}
	return -ETIMEDOUT;
}

/**
 * ch390h_start - Initialize and enable the CH390H chip for operation.
 * @db: Pointer to the driver's private data structure.
 *
 * This function is called from ndo_open. It configures the essential
 * registers, enables the receiver, and sets up interrupts.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390h_start(struct board_info *db)
{
	int ret;
	u8 rcr;

	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_GPR, 0x00),"write GPR failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_MLEDCR, db->lcr_all), "write MLEDCR failed\n");
	msleep(1);

	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db,CH390_MPTRCR, MPTRCR_RST_RX), "write MPTRCR failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_ISR, ISR_CLR_INT), "write ISR failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_IMR, IMR_PAR | IMR_PRI), "write IMR failed\n");

    CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_RCR, &rcr), "read RCR failed\n");
    rcr |= RCR_RXEN;
    CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_RCR, rcr), "write RCR failed\n");
	return 0;
}

/**
 * ch390h_stop - Disable the CH390H chip.
 * @db: Pointer to the driver's private data structure.
 *
 * This function is called from ndo_stop. It disables interrupts, powers down
 * the PHY, and disables the receiver.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390h_stop(struct board_info *db)
{
	int ret;
	u8 rcr;

	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_IMR, 0x00), "write IMR failed\n");
	/*
	 * GPR power off of the internal phy,
	 * the internal phy still could be accessed after this GPR power off control
	 */
	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_GPR, GPR_PHYPD), "power off phy failed\n");
    CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_RCR, &rcr), "read RCR failed\n");
    rcr &= ~RCR_RXEN;
    CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_RCR, rcr), "write RCR failed\n");
	return 0;
}

/**
 * ch390h_transmit - Transmit a single network packet.
 * @db:   Pointer to the driver's private data structure.
 * @buff: Pointer to the packet data.
 * @len:  Length of the packet data.
 *
 * Writes the packet data to the CH390H's transmit buffer, sets the length
 * registers, and issues the transmit command.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390h_transmit(struct board_info *db, u8 *buff, unsigned int len)
{
	int ret;

	len = cpu_to_le32(len);
	unsigned int temp_low = len & 0xff;
	unsigned int temp_high = (len >> 8) & 0xff;
	u8 val, temp;

	CH390_RETURN_ON_ERROR(ch390h_io_memory_write(db, buff, len), "write tx mem failed\n");

	do {
		CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_TCR, &temp), "read TCR failed\n");
	} while (temp & TCR_TXREQ);

	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_TXPLL, temp_low),"write TXPLL failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_TXPLH, temp_high),"write TXPLH failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_TCR, &val),"read TCR failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_TCR, val | TCR_TXREQ),"write TCR failed\n");
	return 0;
}

/**
 * ch390h_receive - Receive a single network packet.
 * @db:  Pointer to the driver's private data structure.
 * @skb: Pointer to an sk_buff pointer.
 *
 * Checks if a packet is ready. If so, it reads the header, checks for errors,
 * allocates an sk_buff, and reads the packet data into it.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390h_receive(struct board_info *db, struct sk_buff *skb)
{
	int ret;
	u16 len;
	u8 ready;

	CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_MRCMDX, &ready), "read MRCMDX failed\n");
	CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_MRCMDX, &ready), "read MRCMDX failed\n");

	if ((!db->rxcsum && (ready & CH390_PKT_ERR)) || 
		(db->rxcsum && (ready & CH390_PKT_ERR_WITH_RCSEN))) {
		CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_RCR, 0),"write RCR failed\n");
		CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_MPTRCR, MPTRCR_RST_RX),"write MPTRCR failed\n");
		CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_MRRH, 0x0C),"write MRRH failed\n");
		msleep(1);
		CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_RCR, RCR_RXEN),"write RCR failed\n");
		return -EIO;
	}
	else {
		struct ch390_rxhdr rx_header;
		if(ready & CH390_PKT_RDY) {
			CH390_RETURN_ON_ERROR(ch390h_io_memory_read(db, (u8 *)& rx_header, sizeof(rx_header)), 
								"peek rx header failed\n");
			len=le16_to_cpu(rx_header.rxlen);
			if(rx_header.status & RSR_ERR_BITS) {
				u64_stats_update_begin(&db->syncp);
				db->stats.rx_dropped++;
				db->stats.rx_errors++;
				u64_stats_update_end(&db->syncp);
				ch390h_drop_frame(db, len);
				return -EIO;
			}
			else if(len > CH390_PKT_MAX) {
				u64_stats_update_begin(&db->syncp);
				db->stats.rx_length_errors++;
				db->stats.rx_errors++;
				u64_stats_update_end(&db->syncp);
				CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_MPTRCR, MPTRCR_RST_RX), 
									"reset rx pointer failed\n");
				return -EIO;
			}
			else {
				skb = dev_alloc_skb(len);
				if(!skb){
					u64_stats_update_begin(&db->syncp);
					db->stats.rx_dropped++;
					db->stats.rx_errors++;
					u64_stats_update_end(&db->syncp);
					ch390h_drop_frame(db, len);
					return -ENOMEM;
				}
				void *ptr = skb_put(skb, len - ETH_FCS_LEN);
				CH390_GOTO_ON_ERROR(ch390h_io_memory_read(db, ptr, len), err, "read rx data failed\n");
				u64_stats_update_begin(&db->syncp);
				db->stats.rx_packets++;
				db->stats.rx_bytes+= len;
				u64_stats_update_end(&db->syncp);
			}
		}
		else 
			skb = NULL;
	}
	return 0;

err:
	u64_stats_update_begin(&db->syncp);
	db->stats.rx_errors++;
	u64_stats_update_end(&db->syncp);
	dev_kfree_skb(skb);
	return ret;
}

/**
 * ch390h_irq_handler - The interrupt handler (top half).
 * @irq: The interrupt number.
 * @pw:  Pointer to the driver's private data structure.
 *
 * This function is the primary interrupt handler. It simply schedules the
 * receive work queue to process the interrupt in a bottom-half context.
 *
 * Return: IRQ_HANDLED.
 */
static irqreturn_t ch390h_irq_handler(int irq, void *pw)
{
	struct board_info *db = pw;
	schedule_work(&db->async_rx_work);
	return IRQ_HANDLED;
}

/**
 * ch390h_async_transmit - Asynchronous transmit work function (bottom half).
 * @work: Pointer to the work_struct.
 *
 * Dequeues packets from the transmit queue (`txq`) and sends them to the
 * hardware. This function runs in a workqueue context.
 */
static void ch390h_async_transmit(struct work_struct *work)
{
	struct board_info *db = container_of(work, struct board_info, async_tx_work);
	struct net_device *ndev = db->ndev;
	int ret;

	mutex_lock(&db->spi_lockm);

    while (!skb_queue_empty(&db->txq)) {
        struct sk_buff *skb;
        unsigned int len;
        skb = skb_dequeue(&db->txq);

		if (skb) {
            ret = ch390h_transmit(db, skb->data, skb->len);
            len = skb->len;
            dev_kfree_skb(skb);

            if (ret < 0) {
				u64_stats_update_begin(&db->syncp);
				db->stats.tx_dropped++;
				db->stats.tx_errors++;
				u64_stats_update_end(&db->syncp);
                goto err;
            }
			u64_stats_update_begin(&db->syncp);
			db->stats.tx_packets++;
			db->stats.tx_bytes += len;
			u64_stats_update_end(&db->syncp);
        }

        if (netif_queue_stopped(ndev) && (skb_queue_len(&db->txq) < CH390_TX_QUE_LO_WATER))
            netif_wake_queue(ndev);
    }

	mutex_unlock(&db->spi_lockm);
	return;

err:
	netdev_err(db->ndev, "transmit packet error\n");
	mutex_unlock(&db->spi_lockm);
}

/**
 * ch390h_async_receive - Asynchronous receive work function (bottom half).
 * @work: Pointer to the work_struct.
 *
 * Reads the interrupt status register and processes received packets. This
 * function runs in a workqueue context, scheduled by the IRQ handler.
 */
static void ch390h_async_receive(struct work_struct *work)
{
	struct board_info *db = container_of(work, struct board_info, async_tx_work);
	int ret;
	u8 status;
	struct sk_buff *skb = NULL;

	mutex_lock(&db->spi_lockm);

	CH390_GOTO_ON_ERROR(ch390h_io_register_read(db, CH390_ISR, &status), err, "read ISR failed\n");
	CH390_GOTO_ON_ERROR(ch390h_io_register_write(db, CH390_ISR, status), err, "write ISR failed\n");

	if (status & ISR_PR) {
		while(1){
			CH390_GOTO_ON_ERROR(ch390h_receive(db, skb), err, "frame read from module failed\n");
			if(!skb)
				break;
			skb->protocol = eth_type_trans(skb, db->ndev);
			if (db->ndev->features & NETIF_F_RXCSUM)
				skb_checksum_none_assert(skb);
			netif_rx(skb);
		}
	}

err:
	mutex_unlock(&db->spi_lockm);
}

/**
 * ch390h_async_apply_rx_mode - Apply new RX mode settings (bottom half).
 * @work: Pointer to the work_struct.
 *
 * Writes the cached MAC address, multicast hash table, and receive control
 * register values to the hardware. This is done in a workqueue to avoid
 * sleeping in the ndo_set_rx_mode callback.
 */
static void ch390h_async_apply_rx_mode(struct work_struct *work)
{
	struct board_info *db = container_of(work, struct board_info, async_rx_mode_work);
	struct net_device *ndev = db->ndev;
	int ret;

	mutex_lock(&db->spi_lockm);

	for(int i=0;i<ETH_ALEN;i++){
		CH390_GOTO_ON_ERROR(ch390h_io_register_write(db, CH390_PAR +i,(ndev->dev_addr)[i]), err, "write PAR failed\n");
	}

	for(int i = 0; i<8 ; i++){
		CH390_GOTO_ON_ERROR(ch390h_io_register_write(db, CH390_MAR + i, db->hash_table[i]), err, "write MAR failed\n");
	}
	CH390_GOTO_ON_ERROR(ch390h_io_register_write(db, CH390_RCR, db->rcr_all), err, "write RCR failed\n");

err:
	mutex_unlock(&db->spi_lockm);
}



/**
 * ch390h_open - Open the network device (ndo_open).
 * @ndev: Pointer to the network device structure.
 *
 * Called when the network device is brought up (e.g., via "ifconfig up").
 * It initializes hardware, starts the PHY, and enables the transmit queue.
 *
 * Return: 0 on success, or a negative error code on failure.
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

	ret = ch390h_start(db);
	if (ret) {
		phy_stop(db->phydev);
		return ret;
	}

	netif_wake_queue(ndev);
	return 0;
}

/**
 * ch390h_close - Close the network device (ndo_stop).
 * @ndev: Pointer to the network device structure.
 *
 * Called when the network device is brought down (e.g., via "ifconfig down").
 * It stops the hardware, stops the PHY, and cleans up running tasks.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390h_close(struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);
	int ret;

	ret = ch390h_stop(db);
	if (ret)
		return ret;

	flush_work(&db->async_tx_work);
	flush_work(&db->async_rx_work);
	flush_work(&db->async_rx_mode_work);

	phy_stop(db->phydev);

	netif_stop_queue(ndev);

	skb_queue_purge(&db->txq);

	return 0;
}

/**
 * ch390h_start_xmit - Transmit a packet (ndo_start_xmit).
 * @skb:  The socket buffer containing the packet to transmit.
 * @ndev: Pointer to the network device structure.
 *
 * This function is called by the network subsystem to send a packet. It queues
 * the packet and schedules the transmit workqueue to perform the actual I/O.
 *
 * Return: NETDEV_TX_OK.
 */
static netdev_tx_t ch390h_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct board_info *db = to_ch390_board(ndev);

	skb_queue_tail(&db->txq, skb);
	if (skb_queue_len(&db->txq) > CH390_TX_QUE_HI_WATER)
		netif_stop_queue(ndev); /* enforce limit queue size */
	schedule_work(&db->async_tx_work);
	return NETDEV_TX_OK;
}

/**
 * ch390h_set_rx_mode - Configure the packet reception mode (ndo_set_rx_mode).
 * @ndev: Pointer to the network device structure.
 *
 * Called by the network subsystem to change the receiver's filter settings,
 * such as enabling promiscuous mode or updating the multicast address list.
 * It calculates the new settings and schedules a work item to apply them.
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
	hash_table[3] = 0;
	hash_table[4] = 0;
	hash_table[5] = 0;
	hash_table[6] = 0;
	hash_table[7] = 0x80;

	/* the multicast address in Hash Table : 64 bits */
	netdev_for_each_mc_addr(ha, ndev) {
		hash_val = crc32_le(~0, ha->addr, ETH_ALEN) & GENMASK(5, 0);
		hash_table[hash_val / 8] |= BIT(hash_val % 8);
	}

	/* schedule work to do the actual set of the data if needed */
	if (memcmp(db->hash_table, hash_table, sizeof(hash_table))|| db->rcr_all!=rcr) {
		memcpy(db->hash_table, hash_table, sizeof(hash_table));
		db->rcr_all = rcr;
		schedule_work(&db->async_rx_mode_work);
	}
}

/**
 * ch390h_set_mac_address - Set the MAC address (ndo_set_mac_address).
 * @ndev: Pointer to the network device structure.
 * @p:    Pointer to a sockaddr containing the new MAC address.
 *
 * This function allows changing the device's hardware MAC address.
 *
 * Return: 0 on success, or a negative error code on failure.
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
		CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_PAR +i,(ndev->dev_addr)[i]), "write PAR failed\n");
	}
	return ret;
}

/**
 * ch390h_get_stats - Get network statistics (ndo_get_stats64).
 * @ndev:    Pointer to the network device structure.
 * @storage: Pointer to rtnl_link_stats64 to store the statistics.
 *
 * Provides the kernel with the driver's collected network statistics.
 */
static void ch390h_get_stats(struct net_device *ndev, struct rtnl_link_stats64 *storage){
	struct board_info *db = to_ch390_board(ndev);
	uint start;
	do {
		start = u64_stats_fetch_begin(&db->syncp);
		*storage = db->stats;
	} while (u64_stats_fetch_retry(&db->syncp, start));
}

static int ch390h_set_features(struct net_device *dev, netdev_features_t features) {
	struct board_info *db = to_ch390_board(ndev);
	int ret = 0;
	int ncr;
	int tcscr;
	int rcscsr;

	if(features & NETIF_F_LOOPBACK){
		CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_NCR, &ncr), "read NCR failed\n");
		ncr |= NCR_LBK_MAC;
		CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_NCR, ncr), "write NCR failed\n");
	}
	else {
		CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_NCR, &ncr), "read NCR failed\n");
		ncr &= ~NCR_LBK_MAC;
		CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_NCR, ncr), "write NCR failed\n");
	}

	if(features & NETIF_F_HW_CSUM) {
		CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_TCSCR, &tcscr), "read TCSCR failed\n");
		tcscr |= TCSCR_ALL;
		CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_TCSCR, tcscr), "write TCSCR failed\n");
	}
	else {
		CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_TCSCR, &tcscr), "read TCSCR failed\n");
		tcscr &= ~TCSCR_ALL;
		CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_TCSCR, tcscr), "write TCSCR failed\n");
	}

	if(features & NETIF_F_RXCSUM) {
		CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_RCSCSR, &rcscsr), "read RCSCSR failed\n");
		rcscsr |= (RCSCSR_RCSEN | RCSCSR_DCSE);
		CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_RCSCSR, rcscsr), "write RCSCSR failed\n");
		db->rxcsum = true;
	}
	else {
		CH390_RETURN_ON_ERROR(ch390h_io_register_read(db, CH390_RCSCSR, &rcscsr), "read RCSCSR failed\n");
		rcscsr &= ~(RCSCSR_RCSEN | RCSCSR_DCSE);
		CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_RCSCSR, rcscsr), "write RCSCSR failed\n");
		db->rxcsum = false;
	}
}

static const struct net_device_ops ch390h_netdev_ops = {
	.ndo_open = ch390h_open,
	.ndo_stop = ch390h_close,
	.ndo_start_xmit = ch390h_start_xmit,
	.ndo_set_rx_mode = ch390h_set_rx_mode,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_set_mac_address = ch390h_set_mac_address,
	.ndo_get_stats64 = ch390h_get_stats,
	.ndo_set_features = ch390h_set_features
};

/**
 * ch390h_mdio_register - Allocate and register the MDIO bus.
 * @db: Pointer to the driver's private data structure.
 *
 * Sets up the MII bus structure and registers it with the kernel's MDIO
 * subsystem, allowing communication with the internal PHY.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
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
		goto err;
	}

	return 0;
err:
	mdiobus_free(db->mdiobus);
	return ret;
}

/**
 * ch390h_mdio_unregister - Unregister and free the MDIO bus.
 * @db: Pointer to the driver's private data structure.
 */
static void ch390h_mdio_unregister(struct board_info *db)
{
	mdiobus_unregister(db->mdiobus);
	mdiobus_free(db->mdiobus);
}

/**
 * ch390h_handle_link_change - PHY link state change handler.
 * @ndev: Pointer to the network device structure.
 *
 * This function is a callback that is invoked by the PHY library when the
 * link status changes (e.g., cable connected/disconnected). It updates flow
 * control settings accordingly.
 */
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

/**
 * ch390h_phy_connect - Connect the driver to the PHY device.
 * @db: Pointer to the driver's private data structure.
 *
 * Uses the registered MDIO bus to find and connect to the internal PHY.
 *
 * Return: 0 on success, or a negative error code on failure.
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

/**
 * ch390h_request_irq - Request and configure the interrupt line.
 * @db: Pointer to the driver's private data structure.
 *
 * Requests the IRQ from the kernel and configures the hardware interrupt
 * polarity based on the device tree settings.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
static int ch390h_request_irq(struct board_info *db)
{
	struct net_device *ndev = db->ndev;
	struct spi_device *spi = db->spidev;
	int ret;

	ndev->irq = spi->irq;
	if(db->irq_posedge){
		CH390_GOTO_ON_ERROR(request_threaded_irq(spi->irq, NULL, ch390h_irq_handler, 
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, ndev->name, db), err, "fail to request irq\n");
		CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_INTCR, INCR_POL_H), "write INTCR failed\n");
	}
	else {
		CH390_GOTO_ON_ERROR(request_threaded_irq(spi->irq, NULL, ch390h_irq_handler, 
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, ndev->name, db), err, "fail to request irq\n");
		CH390_RETURN_ON_ERROR(ch390h_io_register_write(db, CH390_INTCR, INCR_POL_L), "write INTCR failed\n");
	}
	return ret;

err:
	netdev_err(ndev, "failed to request irq!\n");
	return ret;
}

#ifdef CONFIG_WCH_CH390_DEBUG
static ssize_t ch390h_reg_dump_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct board_info *db;
    int len = 0;
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

    for (int i = 0; i < ARRAY_SIZE(reg_labels); i++) {
        if (ch390h_io_register_read(db, reg_labels[i].reg, &val) != 0) {
            dev_err(dev, "Failed to read register %s\n", reg_labels[i].name);
            return -EIO;
        }
        len += sprintf(buf + len, "%s: 0x%02x\n", reg_labels[i].name, val);
    }

    return len;
}

static ssize_t ch390h_reg_dump_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
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

		for (i = 0; i < ARRAY_SIZE(reg_labels); i++) {
			if (strcmp(reg_labels[i].name, reg_name) == 0) {
				reg = reg_labels[i].reg;
				if (ch390h_io_register_write(db, reg, val) < 0)
					dev_info(dev, "set reg: 0x%02x - value: 0x%02x filed!\n", reg, val);
				else
					dev_info(dev, "set reg: 0x%02x - value: 0x%02x success!\n", reg, val);
				break;
			}
		}
	}
	return count;
}

static DEVICE_ATTR_RW(ch390h_reg_dump);

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

/**
 * ch390h_probe - Probe function for the SPI device.
 * @spi: Pointer to the SPI device structure.
 *
 * This is the main entry point for the driver. It's called by the SPI subsystem
 * when a device matching this driver is found. It handles memory allocation,
 * hardware reset and verification, MAC address initialization, MDIO and PHY
 * setup, and registration of the network device.
 *
 * Return: 0 on success, or a negative error code on failure.
 */
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
	db->rxcsum = true;

	ndev->netdev_ops = &ch390h_netdev_ops;
	ndev->ethtool_ops = &ch390h_ethtool_ops;
	ndev->features = NETIF_F_HW_CSUM | NETIF_F_RXCSUM;
	ndev->hw_features =  | NETIF_F_LOOPBACK;

	mutex_init(&db->spi_lockm);
	mutex_init(&db->reg_mutex);

	INIT_WORK(&db->async_tx_work, ch390h_async_transmit);
	INIT_WORK(&db->async_rx_work, ch390h_async_receive);
	INIT_WORK(&db->async_rx_mode_work, ch390h_async_apply_rx_mode);

	if (of_find_property(dev->of_node, "wch,eeprom", NULL))
		db->has_eeprom = true;
	else
	 	db->has_eeprom = false;

	if(irq_get_trigger_type(spi->irq) == IRQF_TRIGGER_RISING)
		db->irq_posedge = true;
	else
	 	db->irq_posedge = false;

	CH390_GOTO_ON_ERROR(ch390h_reset(db), err_nd, "reset hardware failed\n");
	CH390_GOTO_ON_ERROR(ch390h_verify_id(db), err_nd, "verify hardware failed\n");
	CH390_GOTO_ON_ERROR(ch390h_init_mac_addr(ndev, db), err_nd, "init mac address failed\n");
	CH390_GOTO_ON_ERROR(ch390h_mdio_register(db), err_nd, "register mdio failed\n");
	CH390_GOTO_ON_ERROR(ch390h_phy_connect(db), err_mdio, "connect phy failed\n");

	memset(&db->stats,0,sizeof(struct rtnl_link_stats64));

	skb_queue_head_init(&db->txq);

	CH390_GOTO_ON_ERROR(register_netdev(ndev),err_phy, "register netdev failed\n");
	CH390_GOTO_ON_ERROR(ch390h_request_irq(db),err_mdio,"request irq failed\n");

#ifdef CONFIG_WCH_CH390_DEBUG
	ch390h_create_sysfs(spi);
#endif
	return 0;

err_phy:
	phy_disconnect(db->phydev);
err_mdio:
	ch390h_mdio_unregister(db);
err_nd:
	free_netdev(ndev);
	return ret;
}

/**
 * ch390h_remove - Remove function for the SPI device.
 * @spi: Pointer to the SPI device structure.
 *
 * This function is called when the device is removed from the system. It
 * unregisters the network device, disconnects the PHY, frees the IRQ, and
 * releases all allocated resources.
 */
static void ch390h_remove(struct spi_device *spi)
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
	{ .compatible = "wch,ch390h" }, 
	{ .compatible = "wch,ch390d" },
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
	.remove = ch390h_remove,
	.id_table = ch390h_id_table,
};
module_spi_driver(ch390h_driver);

MODULE_AUTHOR("Sergey Kharenko <skharenko@hust.edu.cn>");
MODULE_DESCRIPTION("SPI ethernet driver for CH390H/D");
MODULE_VERSION("1.2.0");
MODULE_LICENSE("GPL");
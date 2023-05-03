// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2022 The SecondHand Authors.
 *
 * Patched for DM9621 
 */

#include <common.h>
#include <dm.h>
#include <log.h>
#include <net.h>
#include <usb.h>
#include <malloc.h>
#include <memalign.h>
#include <linux/delay.h>
#include <linux/mii.h>
#include "usb_ether.h"
#include <linux/bitrev.h>
#include <linux/crc32.h>

/* control requests */
#define DM_READ_REGS			0x00
#define DM_WRITE_REGS			0x01
#define DM_READ_MEMS			0x02
#define DM_WRITE_REG			0x03
#define DM_WRITE_MEMS			0x05
#define DM_WRITE_MEM			0x07

/* registers */
#define DM_NET_CTRL				0x00
#define DM_RX_CTRL				0x05
#define DM_SHARED_CTRL			0x0b
#define DM_SHARED_ADDR			0x0c
#define DM_SHARED_DATA			0x0d	/* low + high */
#define DM_PHY_ADDR				0x10	/* 6 bytes */
#define DM_MCAST_ADDR			0x16	/* 8 bytes */
#define DM_GPR_CTRL				0x1e
#define DM_GPR_DATA				0x1f
#define DM_CHIP_ID				0x2c
#define DM_MODE_CTRL			0x91	/* only on dm9620 */

/* chip id values */
#define ID_DM9601				0
#define ID_DM9620				1

#define DM9621_BASE_NAME "dm621"

#define DM_MAX_MCAST			64
#define DM_MCAST_SIZE			8
#define DM_EEPROM_LEN			256
#define DM_TX_OVERHEAD			2	/* 2 byte header */
#define DM_RX_OVERHEAD			7	/* 3 byte header + 4 byte crc tail */
#define DM_TIMEOUT				1000

#define USB_CTRL_SET_TIMEOUT  	5000
#define USB_CTRL_GET_TIMEOUT  	5000
#define USB_BULK_SEND_TIMEOUT 	5000
#define USB_BULK_RECV_TIMEOUT 	5000

#define DM9621_RX_URB_SIZE  	2048
#define PHY_CONNECT_TIMEOUT 	5000

/* driver private */
struct dm9621_private {
	int flags;
	struct ueth_data ueth;
};

static int dm_read(struct ueth_data *ueth, u8 reg, u16 length, void *data)
{
	int err;
	err = usb_control_msg(
			ueth->pusb_dev,
			usb_rcvctrlpipe(ueth->pusb_dev, 0),
			DM_READ_REGS,
			USB_DIR_IN | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0,
			reg,
			data,
			length,
			USB_CTRL_GET_TIMEOUT);

	if(err >= 0 && err != length)
		err = -EINVAL;
	return err;
}

static int dm_read_reg(struct ueth_data *ueth, u8 reg, u8 *value)
{
	int ret;
	ALLOC_CACHE_ALIGN_BUFFER(u8, buf, 1);
	
	ret = dm_read(ueth, reg, 1, buf);
	*value = *buf;
	return ret;
}

static int dm_write(struct ueth_data *ueth, u8 reg, u16 length, void *data)
{
	int err;
	err = usb_control_msg(
			ueth->pusb_dev,
			usb_sndctrlpipe(ueth->pusb_dev, 0),
			DM_WRITE_REGS,
			USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			0,
			reg,
			data,
			length,
			USB_CTRL_SET_TIMEOUT);

	if (err >= 0 && err < length)
		err = -EINVAL;
	return err;
}

static int dm_write_reg(struct ueth_data *ueth, u8 reg, u8 value)
{
	return usb_control_msg(
			ueth->pusb_dev,
			usb_sndctrlpipe(ueth->pusb_dev, 0),
			DM_WRITE_REG,
			USB_DIR_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
			value,
			reg,
			NULL,
			0,
			USB_CTRL_SET_TIMEOUT);
}

static int dm_read_shared_word(struct ueth_data *ueth, int phy, u8 reg, __le16 *value)
{
	int ret, i;

	ret = dm_write_reg(ueth, DM_SHARED_ADDR, phy ? (reg | 0x40) : reg);
	if (ret < 0)
		goto out;
	ret = dm_write_reg(ueth, DM_SHARED_CTRL, phy ? 0xc : 0x4);
	if (ret < 0)
		goto out;

	for (i = 0; i < DM_TIMEOUT; i++) {
		u8 tmp = 0;

		udelay(1);
		ret = dm_read_reg(ueth, DM_SHARED_CTRL, &tmp);
		if (ret < 0)
			goto out;

		/* ready */
		if ((tmp & 1) == 0)
			break;
	}

	if (i == DM_TIMEOUT) {
		printf("%s read timed out!\n", phy ? "phy" : "eeprom");
		ret = -EIO;
		goto out;
	}

	ret = dm_write_reg(ueth, DM_SHARED_CTRL, 0x0);
	if (ret < 0)
		goto out;
	ret = dm_read(ueth, DM_SHARED_DATA, 2, value);
	if (ret < 0)
		goto out;

 out:
	if(ret < 0)
		printf("dm_read_shared_word err = %d\n", ret);
	return ret;
}

static int dm_write_shared_word(struct ueth_data *ueth, int phy, u8 reg, __le16 *value)
{
	int ret, i;

	ret = dm_write(ueth, DM_SHARED_DATA, 2, value);
	if (ret < 0)
		goto out;

	ret = dm_write_reg(ueth, DM_SHARED_ADDR, phy ? (reg | 0x40) : reg);
	if (ret < 0)
		goto out;
	ret = dm_write_reg(ueth, DM_SHARED_CTRL, phy ? 0x1a : 0x12);
	if (ret < 0)
		goto out;

	for (i = 0; i < DM_TIMEOUT; i++) {
		u8 tmp = 0;

		udelay(1);
		ret = dm_read_reg(ueth, DM_SHARED_CTRL, &tmp);
		if (ret < 0)
			goto out;

		/* ready */
		if ((tmp & 1) == 0)
			break;
	}

	if (i == DM_TIMEOUT) {
		printf("%s write timed out!\n", phy ? "phy" : "eeprom");
		ret = -EIO;
		goto out;
	}

	ret = dm_write_reg(ueth, DM_SHARED_CTRL, 0x0);
	if (ret < 0)
		goto out;

out:
	if(ret < 0)
		printf("dm_write_shared_word err = %d\n", ret);
	return ret;
}

static int dm9621_mdio_read(struct ueth_data *ueth, int phy_id, int loc)
{
	int ret = 0;
	ALLOC_CACHE_ALIGN_BUFFER(__le16, res, 1);

	if (phy_id) {
		printf("Only internal phy supported\n");
		return 0;
	}

	ret = dm_read_shared_word(ueth, 1, loc, res);
	if(ret < 0)
		return ret;

	return le16_to_cpu(*res);
}

static int dm9621_mdio_write(struct ueth_data *ueth, int phy_id, int loc, int val)
{
	ALLOC_CACHE_ALIGN_BUFFER(__le16, res, 1);
	*res = cpu_to_le16(val);
	
	if (phy_id) {
		printf("Only internal phy supported\n");
		return -1;
	}

	return dm_write_shared_word(ueth, 1, loc, res);
}

static int mii_nway_restart(struct ueth_data *ueth)
{
	int bmcr;
	int ret = -EINVAL;

	/* if autoneg is off, it's an error */
	bmcr = dm9621_mdio_read(ueth, ueth->phy_id, MII_BMCR);

	if (bmcr & BMCR_ANENABLE) {
		bmcr |= BMCR_ANRESTART;
		ret = dm9621_mdio_write(ueth, ueth->phy_id, MII_BMCR, bmcr);
	}

	return ret;
}

static int dm9621_set_multicast(struct ueth_data *ueth, u8 *enetaddr)
{
	int ret = 0;
	u8 rx_ctl = 0x31;
	ALLOC_CACHE_ALIGN_BUFFER(u8, hashes, 20);
	
	memset(hashes, 0x00, DM_MCAST_SIZE);
	hashes[DM_MCAST_SIZE - 1] |= 0x80;	/* broadcast address */

	u32 crc = (bitrev32(crc32_le(~0, enetaddr, ETH_ALEN))) >> 26;
	hashes[crc >> 3] |= 1 << (crc & 0x7);

	ret = dm_write(ueth, DM_MCAST_ADDR, DM_MCAST_SIZE, hashes);
	if (ret < 0)
		return ret;
	ret = dm_write_reg(ueth, DM_RX_CTRL, rx_ctl);
	if (ret < 0)
		return ret;

	return 0;	
}

static int dm9621_init_common(struct ueth_data *ueth)
{
	int timeout = 0;
#define TIMEOUT_RESOLUTION 50	/* ms */
	int link_detected;

	do {
		link_detected = dm9621_mdio_read(ueth, ueth->phy_id, MII_BMSR) & BMSR_LSTATUS;
		if (!link_detected) {
			if (timeout == 0)
				printf("Waiting for Ethernet connection... ");
			udelay(TIMEOUT_RESOLUTION * 1000);
			timeout += TIMEOUT_RESOLUTION;
		}
	} while (!link_detected && timeout < PHY_CONNECT_TIMEOUT);
	if (link_detected) {
		if (timeout != 0)
			printf("done.\n");
	} else {
		printf("unable to connect.\n");
		goto out_err;
	}

	/*
	 * Wait some more to avoid timeout on first transfer
	 * (e.g. EHCI timed out on TD - token=0x8008d80)
	 */
	mdelay(50);

	return 0;

out_err:
	return -1;
}

static int dm9621_send_common(struct ueth_data *ueth, void *packet, int length)
{
	int err;
	u16 packet_len;
	int actual_len;
	ALLOC_CACHE_ALIGN_BUFFER(u8, msg, PKTSIZE + DM_TX_OVERHEAD);	
	
	/* format:
	 * b1: packet length low
	 * b2: packet length high
	 * b3..n: packet data
	 */

	packet_len = length;
	cpu_to_le16s(&packet_len);

	memcpy(msg, &packet_len, DM_TX_OVERHEAD);
	memcpy(msg + DM_TX_OVERHEAD, (void *)packet, length);

	err = usb_bulk_msg(ueth->pusb_dev,
				usb_sndbulkpipe(ueth->pusb_dev, ueth->ep_out),
				(void *)msg,
				length + sizeof(packet_len),
				&actual_len,
				USB_BULK_SEND_TIMEOUT);
	debug("Tx: len = %zu, actual = %u, err = %d\n",
			length + sizeof(packet_len), actual_len, err);

	return err;
}

static int dm9621_write_hwaddr_common(struct ueth_data *ueth, u8 *enetaddr)
{
	ALLOC_CACHE_ALIGN_BUFFER(u8, buff, ETH_ALEN);

	if (!is_valid_ethaddr(enetaddr)) {
		printf("not setting invalid mac address %pM\n", enetaddr);
		return -EINVAL;
	}

	memcpy(buff, enetaddr, ETH_ALEN);
	return dm_write(ueth, DM_PHY_ADDR, ETH_ALEN, buff);
}

static int dm9621_basic_reset(struct ueth_data *ueth, u8 *enetaddr)
{
	u8 id = 0;
	int ret = 0;
	ALLOC_CACHE_ALIGN_BUFFER(u8, buff, ETH_ALEN);
	
	/* reset */
	if (dm_write_reg(ueth, DM_NET_CTRL, 1) < 0) {
		printf("Error reset net\n");
		return -ENODEV;
	}
	udelay(20);

	/* read MAC */
	if (dm_read(ueth, DM_PHY_ADDR, ETH_ALEN, buff) < 0) {
		printf("Error reading MAC address\n");
		return -ENODEV;
	}
	
	memcpy(enetaddr, buff, ETH_ALEN);

	if (dm_read_reg(ueth, DM_CHIP_ID, &id) < 0) {
		printf("Error reading chip ID\n");
		return -ENODEV;
	}

	/* put dm9620 devices in dm9601 mode 3byte RX header */
	if (id == ID_DM9620) {
		u8 mode;

		if (dm_read_reg(ueth, DM_MODE_CTRL, &mode) < 0) {
			printf("Error reading MODE_CTRL\n");
			return -ENODEV;
		}
		ret = dm_write_reg(ueth, DM_MODE_CTRL, mode&0x7f);
		if (ret < 0)
			return ret;
	}

	/* power up phy */
	ret = dm_write_reg(ueth, DM_GPR_CTRL, 1);
	if (ret < 0)
		return ret;
	ret = dm_write_reg(ueth, DM_GPR_DATA, 0);
	if (ret < 0)
		return ret;

	/* receive broadcast packets */
	ret = dm9621_set_multicast(ueth, enetaddr);
	if (ret < 0)
		return ret;

	/* reset interval phy */
	ret = dm9621_mdio_write(ueth, ueth->phy_id, MII_BMCR, BMCR_RESET);
	if (ret < 0)
		return ret;
	ret = dm9621_mdio_write(ueth, ueth->phy_id, MII_ADVERTISE,
			  ADVERTISE_ALL | ADVERTISE_CSMA | ADVERTISE_PAUSE_CAP);
	if (ret < 0)
		return ret;
				  
	if (mii_nway_restart(ueth))
	{
		printf("Auto-negotiation not enable\n");
		return -ENODEV;
	}

	return 0;
}

static int dm9621_eth_start(struct udevice *udev)
{
	struct dm9621_private *priv = dev_get_priv(udev);

	return dm9621_init_common(&priv->ueth);
}

static int dm9621_eth_send(struct udevice *udev, void *packet, int length)
{
	struct dm9621_private *priv = dev_get_priv(udev);

	return dm9621_send_common(&priv->ueth, packet, length);
}

static int dm9621_eth_recv(struct udevice *udev, int flags, uchar **packetp)
{
	u8 *ptr;
	u8 status;
	int ret, len;
	u32 packet_len;
	struct dm9621_private *priv = dev_get_priv(udev);
	ALLOC_CACHE_ALIGN_BUFFER(u8, pkt, PKTSIZE);

	len = usb_ether_get_rx_bytes(&priv->ueth, &ptr);
	debug("%s: first try, len=%d\n", __func__, len);
	if (!len) {
		if (!(flags & ETH_RECV_CHECK_DEVICE))
			return -EAGAIN;
		ret = usb_ether_receive(&priv->ueth, DM9621_RX_URB_SIZE);
		if (ret == -EAGAIN)
			return ret;

		len = usb_ether_get_rx_bytes(&priv->ueth, &ptr);
		debug("%s: second try, len=%d\n", __func__, len);
	}

	/* format:
	 * b1: rx status
	 * b2: packet length (incl crc) low
	 * b3: packet length (incl crc) high
	 * b4..n-4: packet data
	 * bn-3..bn: ethernet crc
	 */

	if (len < DM_RX_OVERHEAD) {
		debug("Rx: incomplete packet length\n");
		goto err;
	}

	status = ptr[0];
	if (unlikely(status & 0xbf)) {
		printf("Rx: packet status failure: %d\n", status);
		goto err;
	}

	/* sub crc data */ 
	packet_len = (ptr[1] | (ptr[2]<<8)) - 4;
	if (packet_len > len - DM_RX_OVERHEAD) {
		debug("Rx: too large packet: %d\n", packet_len);
		goto err;
	}

	memcpy(pkt, ptr + 3, packet_len);   /* 3 bytes header */
	memcpy(ptr, pkt, packet_len);       /* copy to dev->rxbuf */

	*packetp = ptr;
	return packet_len;

err:
	usb_ether_advance_rxbuf(&priv->ueth, -1);
	return -EINVAL;
}

static int dm9621_free_pkt(struct udevice *udev, uchar *packet, int packet_len)
{
	struct dm9621_private *priv = dev_get_priv(udev);

	if (packet_len & 1)
		packet_len++;
	usb_ether_advance_rxbuf(&priv->ueth, DM_RX_OVERHEAD + packet_len);

	return 0;
}

static void dm9621_eth_stop(struct udevice *udev)
{
	debug("** %s()\n", __func__);
}

static int dm9621_write_hwaddr(struct udevice *udev)
{
	struct eth_pdata *pdata = dev_get_plat(udev);
	struct dm9621_private *priv = dev_get_priv(udev);

	if (dm9621_write_hwaddr_common(&priv->ueth, pdata->enetaddr) < 0)
		return -1;
	else
		return 0;	
}

static int dm9621_eth_probe(struct udevice *udev)
{
	int ret;
	struct eth_pdata *pdata = dev_get_plat(udev);
	struct dm9621_private *priv = dev_get_priv(udev);

	/* register net device */
	priv->flags = udev->driver_data;
	ret = usb_ether_register(udev, &priv->ueth, DM9621_RX_URB_SIZE);
	if (ret)
		return ret;

	/* reset net device */
	ret = dm9621_basic_reset(&priv->ueth, pdata->enetaddr);
	if (ret)
		goto err;

	return 0;

err:
	return usb_ether_deregister(&priv->ueth);
}

static const struct eth_ops dm9621_eth_ops = {
	.start				= dm9621_eth_start,
	.send				= dm9621_eth_send,
	.recv				= dm9621_eth_recv,
	.free_pkt			= dm9621_free_pkt,
	.stop				= dm9621_eth_stop,
	.write_hwaddr		= dm9621_write_hwaddr,
};

U_BOOT_DRIVER(dm9621_eth) = {
	.name = "dm9621_eth",
	.id = UCLASS_ETH,
	.probe = dm9621_eth_probe,
	.ops = &dm9621_eth_ops,
	.priv_auto = sizeof(struct dm9621_private),
	.plat_auto = sizeof(struct eth_pdata),
};

static const struct usb_device_id dm9621_eth_id_table[] = {
	{ USB_DEVICE(0x07aa, 0x9601), .driver_info = 0},	/* Corega FEther USB-TXC */
	{ USB_DEVICE(0x0a46, 0x9601), .driver_info = 0},	/* Davicom USB-100 */
	{ USB_DEVICE(0x0a46, 0x6688), .driver_info = 0},	/* ZT6688 USB NIC */
	{ USB_DEVICE(0x0a46, 0x0268), .driver_info = 0},	/* ShanTou ST268 USB NIC */
	{ USB_DEVICE(0x0a46, 0x8515), .driver_info = 0},	/* ADMtek ADM8515 USB NIC */
	{ USB_DEVICE(0x0a47, 0x9601), .driver_info = 0},	/* Hirose USB-100 */
	{ USB_DEVICE(0x0fe6, 0x8101), .driver_info = 0},	/* DM9601 USB to Fast Ethernet Adapter */
	{ USB_DEVICE(0x0fe6, 0x9700), .driver_info = 0},	/* DM9601 USB to Fast Ethernet Adapter */
	{ USB_DEVICE(0x0a46, 0x9000), .driver_info = 0},	/* DM9000E */
	{ USB_DEVICE(0x0a46, 0x9620), .driver_info = 0},	/* DM9620 USB to Fast Ethernet Adapter */
	{ USB_DEVICE(0x0a46, 0x9621), .driver_info = 0},	/* DM9621A USB to Fast Ethernet Adapter */
	{ USB_DEVICE(0x0a46, 0x9622), .driver_info = 0},	/* DM9622 USB to Fast Ethernet Adapter */
	{ USB_DEVICE(0x0a46, 0x0269), .driver_info = 0},	/* DM962OA USB to Fast Ethernet Adapter */
	{ USB_DEVICE(0x0a46, 0x1269), .driver_info = 0},	/* DM9621A USB to Fast Ethernet Adapter */
	{}, // END
};

U_BOOT_USB_DEVICE(dm9621_eth, dm9621_eth_id_table);

/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2011 Samsung Electronics
 *
 * Configuration settings for the SAMSUNG ITOP4412 (EXYNOS4412) board.
 */

#ifndef __CONFIG_ITOP4412_H
#define __CONFIG_ITOP4412_H

#include <configs/exynos4-common.h>

/* High Level Configuration Options */
#define CONFIG_USB_ETHER_DM9621
#define CONFIG_CLK_1000_400_200
#ifndef CONFIG_SYS_L2CACHE_OFF
#define CONFIG_SYS_L2_PL310
#define CFG_SYS_PL310_BASE	0x10502000
#endif

/* ITOP has 4 bank of DRAM */
#define CFG_SYS_SDRAM_BASE		0x40000000
#define PHYS_SDRAM_1			CFG_SYS_SDRAM_BASE
#define SDRAM_BANK_SIZE			(256 << 20)	/* 256 MB */
#define CONFIG_SPL_STACK 		0x02040000

/* Power Down Modes */
#define S5P_CHECK_SLEEP			0x00000BAD
#define S5P_CHECK_DIDLE			0xBAD00000
#define S5P_CHECK_LPA			0xABAD0000

#define CFG_EXTRA_ENV_SETTINGS \
	"mmcbootdev=0\0" \
	"mmcbootpart=1\0" \
	"mmcrootpart=2\0" \
	"fdt_addr_r=0x41000000\0" \
	"kernel_addr_r=0x40007000\0" \
	"fdtfile=exynos4412-itop-elite.dtb\0" \
	"console=console=ttySAC2,115200n8\0" \
	"ipaddr=192.168.0.77\0" \
	"serverip=192.168.0.88\0" \
	"gatewayip=192.168.0.1\0" \
	"netmask=255.255.255.0\0" \
	"ethaddr=12:34:56:78:9A:BC\0" \
	"hostnfsaddr=/home/mt/Netshare/rootfs\0" \
	"net_args=" \
		"setenv bootargs root=/dev/nfs nfsroot=${serverip}:${hostnfsaddr},proto=tcp,nfsvers=4 rw " \
		"ip=${ipaddr}:${serverip}:${gatewayip}:${netmask}::eth0:off " \
		"${console} init=/linuxrc\0" \
	"load_netkernel=tftpboot ${kernel_addr_r} ${kernelname}\0" \
	"load_netdtb=tftpboot ${fdt_addr_r} ${fdtfile}\0" \
	"kernel_args=" \
		"setenv bootargs root=/dev/mmcblk${mmcbootdev}p${mmcrootpart} " \
		"rootfstype=ext4 rootwait ${console}\0" \
	"load_kernel=load mmc ${mmcbootdev}:${mmcbootpart} ${kernel_addr_r} " \
		"${kernelname}\0" \
	"load_dtb=load mmc ${mmcbootdev}:${mmcbootpart} ${fdt_addr_r} " \
		"${fdtfile}\0" \
	"check_dtb=" \
		"if run load_dtb; then;" \
			"setenv fdt_addr ${fdt_addr_r};" \
		"else " \
			"setenv fdt_addr;" \
		"fi;\0" \
	"check_netdtb=" \
		"if run load_netdtb; then;" \
			"setenv fdt_addr ${fdt_addr_r};" \
		"else " \
			"setenv fdt_addr;" \
		"fi;\0" \
	"boot_zimg=" \
		"setenv kernelname zImage;" \
		"run check_dtb;" \
		"run load_kernel;" \
		"run kernel_args;" \
		"bootz ${kernel_addr_r} - ${fdt_addr};\0" \
	"boot_uimg=" \
		"setenv kernelname uImage;" \
		"run check_dtb;" \
		"run load_kernel;" \
		"run kernel_args;" \
		"bootz ${kernel_addr_r} - ${fdt_addr};\0" \
	"boot_net=" \
		"setenv kernelname zImage;" \
		"run check_netdtb;" \
		"run load_netkernel;" \
		"run net_args;" \
		"bootz ${kernel_addr_r} - ${fdt_addr};\0"

#define CONFIG_BOOTCOMMAND \
	"usb start;" \
	"usb reset;" \
	"if test -e mmc ${mmcbootdev} zImage; then;" \
		"echo boot zImage from mmc ${mmcbootdev} \n" \
		"run boot_zimg;" \
	"elif test -e mmc ${mmcbootdev} uImage; then;" \
		"echo boot uImage from mmc ${mmcbootdev} \n" \
		"run boot_uimg;" \
	"else " \
		"echo boot zImage from net \n;" \
		"run boot_net;" \
	"fi;"

#endif	/* __CONFIG_H */

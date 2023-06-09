/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Configuation settings for the Freescale MCF53017EVB.
 *
 * Copyright (C) 2004-2008 Freescale Semiconductor, Inc.
 * TsiChung Liew (Tsi-Chung.Liew@freescale.com)
 */

/*
 * board/config.h - configuration options, board specific
 */

#ifndef _M53017EVB_H
#define _M53017EVB_H

/*
 * High Level Configuration Options
 * (easy to change)
 */

#define CFG_SYS_UART_PORT		(0)

#ifdef CONFIG_MCFFEC
#	define CFG_SYS_TX_ETH_BUFFER	8
#	define CFG_SYS_FEC_BUF_USE_SRAM
#endif

#define CFG_SYS_RTC_CNT		(0x8000)
#define CFG_SYS_RTC_SETUP		(RTC_OCEN_OSCBYP | RTC_OCEN_CLKEN)

/* I2C */

#define CFG_EXTRA_ENV_SETTINGS		\
	"netdev=eth0\0"				\
	"loadaddr=40010000\0"			\
	"u-boot=u-boot.bin\0"			\
	"load=tftp ${loadaddr) ${u-boot}\0"	\
	"upd=run load; run prog\0"		\
	"prog=prot off 0 3ffff;"		\
	"era 0 3ffff;"				\
	"cp.b ${loadaddr} 0 ${filesize};"	\
	"save\0"				\
	""

#define CFG_PRAM		512	/* 512 KB */

#define CFG_SYS_CLK		80000000
#define CFG_SYS_CPU_CLK	CFG_SYS_CLK * 3

#define CFG_SYS_MBAR		0xFC000000

/*
 * Low Level Configuration Settings
 * (address mappings, register initial values, etc.)
 * You should know what you are doing if you make changes here.
 */
/*
 * Definitions for initial stack pointer and data area (in DPRAM)
 */
#define CFG_SYS_INIT_RAM_ADDR	0x80000000
#define CFG_SYS_INIT_RAM_SIZE		0x20000	/* Size of used area in internal SRAM */
#define CFG_SYS_INIT_RAM_CTRL	0x221

/*
 * Start addresses for the final memory configuration
 * (Set up by the startup code)
 * Please note that CFG_SYS_SDRAM_BASE _must_ start at 0
 */
#define CFG_SYS_SDRAM_BASE		0x40000000
#define CFG_SYS_SDRAM_SIZE		64	/* SDRAM size in MB */
#define CFG_SYS_SDRAM_CFG1		0x43711630
#define CFG_SYS_SDRAM_CFG2		0x56670000
#define CFG_SYS_SDRAM_CTRL		0xE1092000
#define CFG_SYS_SDRAM_EMOD		0x80010000
#define CFG_SYS_SDRAM_MODE		0x00CD0000

/*
 * For booting Linux, the board info and command line data
 * have to be in the first 8 MB of memory, since this is
 * the maximum mapped by the Linux kernel during initialization ??
 */
#define CFG_SYS_BOOTMAPSZ		(CFG_SYS_SDRAM_BASE + (CFG_SYS_SDRAM_SIZE << 20))

/*-----------------------------------------------------------------------
 * FLASH organization
 */
#ifdef CONFIG_SYS_FLASH_CFI
#	define CFG_SYS_FLASH_SIZE		0x1000000	/* Max size that the board might have */
#endif

#define CFG_SYS_FLASH_BASE		CFG_SYS_CS0_BASE

/* Configuration for environment
 * Environment is embedded in u-boot in the second sector of the flash
 */

#define LDS_BOARD_TEXT \
	. = DEFINED(env_offset) ? env_offset : .; \
	env/embedded.o(.text*)

/*-----------------------------------------------------------------------
 * Cache Configuration
 */

#define ICACHE_STATUS			(CFG_SYS_INIT_RAM_ADDR + \
					 CFG_SYS_INIT_RAM_SIZE - 8)
#define DCACHE_STATUS			(CFG_SYS_INIT_RAM_ADDR + \
					 CFG_SYS_INIT_RAM_SIZE - 4)
#define CFG_SYS_ICACHE_INV		(CF_CACR_CINVA)
#define CFG_SYS_CACHE_ACR0		(CFG_SYS_SDRAM_BASE | \
					 CF_ADDRMASK(CFG_SYS_SDRAM_SIZE) | \
					 CF_ACR_EN | CF_ACR_SM_ALL)
#define CFG_SYS_CACHE_ICACR		(CF_CACR_EC | CF_CACR_CINVA | \
					 CF_CACR_DCM_P)

/*-----------------------------------------------------------------------
 * Chipselect bank definitions
 */
/*
 * CS0 - NOR Flash
 * CS1 - Ext SRAM
 * CS2 - Available
 * CS3 - Available
 * CS4 - Available
 * CS5 - Available
 */
#define CFG_SYS_CS0_BASE		0
#define CFG_SYS_CS0_MASK		0x00FF0001
#define CFG_SYS_CS0_CTRL		0x00001FA0

#define CFG_SYS_CS1_BASE		0xC0000000
#define CFG_SYS_CS1_MASK		0x00070001
#define CFG_SYS_CS1_CTRL		0x00001FA0

#define CFG_MCFTMR

#endif				/* _M53017EVB_H */

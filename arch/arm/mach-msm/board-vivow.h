/* linux/arch/arm/mach-msm/board-vivo.h
 *
 * Copyright (C) 2010-2011 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ARCH_ARM_MACH_MSM_BOARD_VIVOW_H
#define __ARCH_ARM_MACH_MSM_BOARD_VIVOW_H

#include <mach/board.h>

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

#define MSM_LINUX_BASE1		0x05000000
#define MSM_LINUX_SIZE1		0x0B000000
#define MSM_LINUX_BASE2		0x20000000
#define MSM_LINUX_SIZE2		0x09A00000	/* 156MB -> 154MB*/
#define MSM_MEM_256MB_OFFSET	0x10000000

#define MSM_GPU_MEM_BASE	0x00100000
#define MSM_GPU_MEM_SIZE	0x00300000

#define MSM_RAM_CONSOLE_BASE	0x00500000
#define MSM_RAM_CONSOLE_SIZE	0x00100000

#define MSM_PMEM_ADSP_SIZE	0x03A00000	/* 64MB -> 55MB -> 58MB */
#define PMEM_KERNEL_EBI0_SIZE   0x00900000	/* 7MB -> 9MB*/
#define MSM_PMEM_SF_SIZE	0x01B40000
#define MSM_PMEM_ADSP2_SIZE     0x002C0000
#define MSM_FB_SIZE		0x00500000

/* GPIO definition */
#define VIVOW_PMIC_GPIO_INT		(27)

#define VIVOW_GPIO_WIFI_IRQ		(147)
#define VIVOW_GPIO_WIFI_SHUTDOWN_N      (39)

#define VIVOW_GPIO_KEYPAD_POWER_KEY	(46)

#define VIVOW_GPIO_TORCH_EN		(98)

#define VIVOW_GPIO_TP_EN		(105)

#define VIVOW_GPIO_CHG_INT		(180)

#define VIVOW_LAYOUTS			{ \
		{ { 0,  1, 0}, { -1, 0,  0}, {0, 0,  1} }, \
		{ { 0, -1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
		{ { -1, 0, 0}, { 0, -1,  0}, {0, 0,  1} }, \
		{ {-1,  0, 0}, { 0,  0, -1}, {0, 1,  0} }  \
					}

#define VIVOW_MDDI_TE			(30)
#define VIVOW_LCD_RSTz			(126)
#define VIVOW_LCD_ID1			(128)
#define VIVOW_LCD_ID0			(129)

/* BT */
#define VIVOW_GPIO_BT_UART1_RTS		(134)
#define VIVOW_GPIO_BT_UART1_CTS		(135)
#define VIVOW_GPIO_BT_UART1_RX		(136)
#define VIVOW_GPIO_BT_UART1_TX		(137)
#define VIVOW_GPIO_BT_PCM_OUT		(138)
#define VIVOW_GPIO_BT_PCM_IN		(139)
#define VIVOW_GPIO_BT_PCM_SYNC		(140)
#define VIVOW_GPIO_BT_PCM_CLK		(141)
#define VIVOW_GPIO_BT_RESET_N		(41)
#define VIVOW_GPIO_BT_HOST_WAKE		(44)
#define VIVOW_GPIO_BT_CHIP_WAKE		(50)
#define VIVOW_GPIO_BT_SHUTDOWN_N	(38)

/* USB */
#define VIVOW_GPIO_USB_ID_PIN		(49)
#define VIVOW_GPIO_USB_ID1_PIN		(145)
#define VIVOW_AUDIOz_UART_SW		(95)
#define VIVOW_USBz_AUDIO_SW		(96)

#define VIVOW_GPIO_PS_HOLD		(29)

#define VIVOW_AUD_MICPATH_SEL		(127)

/* 35mm headset */
#define VIVOW_GPIO_35MM_HEADSET_DET	(26)

/* EMMC */
#define VIVOW_GPIO_EMMC_RST		(88)

/* Camera */
#define CAM1_PWD			(35)
#define CAM1_VCM_PWD			(34)
#define CAM2_PWD			(146)
#define CAM2_RST			(31)
#define CLK_SWITCH			(144)


/* PMIC GPIO */
#define PMGPIO(x) (x-1)
#define VIVOW_GPIO_TP_INT_N		PMGPIO(1)
#define VIVOW_GPIO_GSENSOR_INT		PMGPIO(7)
#define VIVOW_AUD_SPK_SD		PMGPIO(12)
#define VIVOW_GPIO_FLASH_EN		PMGPIO(15)
#define VIVOW_VOL_UP			PMGPIO(16)
#define VIVOW_VOL_DN			PMGPIO(17)
#define VIVOW_AUD_AMP_EN		PMGPIO(26)
#define VIVOW_GPIO_PS_EN		PMGPIO(20)
#define VIVOW_TP_RSTz			PMGPIO(21)
#define VIVOW_GPIO_PS_INT_N		PMGPIO(22)
#define VIVOW_GPIO_UP_INT_N		PMGPIO(23)
#define VIVOW_GREEN_LED			PMGPIO(24)
#define VIVOW_AMBER_LED			PMGPIO(25)
#define VIVOW_GPIO_SDMC_CD_N		PMGPIO(34)
#define VIVOW_GPIO_LS_EN		PMGPIO(35)
#define VIVOW_GPIO_uP_RST		PMGPIO(36)
#define VIVOW_GPIO_COMPASS_INT_N	PMGPIO(37)
#define VIVOW_GPIO_WIFI_BT_SLEEP_CLK_EN	PMGPIO(38)

extern struct platform_device msm_device_mdp;
extern struct platform_device msm_device_mddi0;
extern unsigned long msm_fb_base;
extern int panel_type;

int vivow_panel_sleep_in(void);
#ifdef CONFIG_MICROP_COMMON
void __init vivow_microp_init(void);
#endif
int vivow_init_mmc(unsigned int sys_rev);
void __init vivow_audio_init(void);
int __init vivow_init_keypad(void);
int __init vivow_wifi_init(void);
int __init vivow_init_panel(void);

#endif /* __ARCH_ARM_MACH_MSM_BOARD_VIVOW_H */

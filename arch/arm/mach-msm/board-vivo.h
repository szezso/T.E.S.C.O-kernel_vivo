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

#ifndef __ARCH_ARM_MACH_MSM_BOARD_VIVO_H
#define __ARCH_ARM_MACH_MSM_BOARD_VIVO_H

#include <mach/board.h>

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

#define MSM_LINUX_BASE1			0x04400000
#define MSM_LINUX_SIZE1			0x1BC00000
#define MSM_LINUX_BASE2			0x20000000
#define MSM_LINUX_SIZE2			0xFB00000

#define MSM_GPU_MEM_BASE		0x00100000
#define MSM_GPU_MEM_SIZE		0x00300000

#define MSM_RAM_CONSOLE_BASE	0x00500000
#define MSM_RAM_CONSOLE_SIZE	0x00100000

#define MSM_FB_BASE				0x2FB00000
#define MSM_FB_SIZE				0x00500000

/* GPIO definition */
#if 1
#define VIVO_PMIC_GPIO_INT			(27)

#define VIVO_GPIO_WIFI_IRQ             (147)
#define VIVO_GPIO_WIFI_SHUTDOWN_N       (39)

#define VIVO_GPIO_KEYPAD_POWER_KEY		(46)

#define VIVO_GPIO_TORCH_EN			(98)

#define VIVO_GPIO_TP_EN			(105)

#define VIVO_GPIO_CHG_INT	(180)

#define VIVO_LAYOUTS			{ \
		{ { 0,  1, 0}, { -1, 0,  0}, {0, 0,  1} }, \
		{ { 0, -1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
		{ { -1, 0, 0}, { 0, -1,  0}, {0, 0,  1} }, \
		{ {-1,  0, 0}, { 0,  0, -1}, {0, 1,  0} }  \
					}
#define VIVO_MDDI_TE			(30)
#define VIVO_LCD_RSTz		(126)
#define VIVO_LCD_ID1			(128)
#define VIVO_LCD_ID0			(129)

/* BT */
#define VIVO_GPIO_BT_UART1_RTS      (134)
#define VIVO_GPIO_BT_UART1_CTS      (135)
#define VIVO_GPIO_BT_UART1_RX       (136)
#define VIVO_GPIO_BT_UART1_TX       (137)
#define VIVO_GPIO_BT_PCM_OUT        (138)
#define VIVO_GPIO_BT_PCM_IN         (139)
#define VIVO_GPIO_BT_PCM_SYNC       (140)
#define VIVO_GPIO_BT_PCM_CLK        (141)
#define VIVO_GPIO_BT_RESET_N        (41)
#define VIVO_GPIO_BT_HOST_WAKE      (44)
#define VIVO_GPIO_BT_CHIP_WAKE      (50)
#define VIVO_GPIO_BT_SHUTDOWN_N     (38)

/* USB */
#define VIVO_GPIO_USB_ID_PIN			(49)
#define VIVO_GPIO_USB_ID1_PIN			(145)
#define VIVO_AUDIOz_UART_SW			(95)
#define VIVO_USBz_AUDIO_SW				(96)

#define VIVO_GPIO_PS_HOLD	(29)

#define VIVO_AUD_MICPATH_SEL		(127)

/* 35mm headset */
#define VIVO_GPIO_35MM_HEADSET_DET	(26)

/* EMMC */
#define VIVO_GPIO_EMMC_RST			  (88)

/* Camera */
#define CAM1_PWD			(35)
#define CAM1_VCM_PWD	(34)
#define CAM2_PWD			(146)
#define CAM2_RST			(31)
#define CLK_SWITCH		(144)


/* PMIC GPIO */
#define PMGPIO(x) (x-1)
#define VIVO_GPIO_TP_INT_N		PMGPIO(1)
#define VIVO_GPIO_GSENSOR_INT	PMGPIO(7)
#define VIVO_AUD_SPK_SD		PMGPIO(12)
#define VIVO_GPIO_FLASH_EN	PMGPIO(15)
#define VIVO_VOL_UP			PMGPIO(16)
#define VIVO_VOL_DN			PMGPIO(17)
#define VIVO_AUD_AMP_EN		PMGPIO(26)
#define VIVO_GPIO_PS_EN		PMGPIO(20)
#define VIVO_TP_RSTz			PMGPIO(21)
#define VIVO_GPIO_PS_INT_N		PMGPIO(22)
#define VIVO_GPIO_UP_INT_N		PMGPIO(23)
#define VIVO_GREEN_LED		PMGPIO(24)
#define VIVO_AMBER_LED		PMGPIO(25)
#define VIVO_GPIO_SDMC_CD_N		PMGPIO(34)
#define VIVO_GPIO_LS_EN		PMGPIO(35)
#define VIVO_GPIO_uP_RST PMGPIO(36)
#define VIVO_GPIO_COMPASS_INT_N PMGPIO(37)
#define VIVO_GPIO_WIFI_BT_SLEEP_CLK_EN	PMGPIO(38)
#else
#define VIVO_GPIO_WIFI_IRQ             (147)
#define VIVO_GPIO_WIFI_SHUTDOWN_N       (39)

#define VIVO_GPIO_UART2_RX 				(51)
#define VIVO_GPIO_UART2_TX 				(52)

#define VIVO_GPIO_KEYPAD_POWER_KEY		(46)

/* Proximity */
#define VIVO_GPIO_PROXIMITY_INT_N		(18)

#define VIVO_GPIO_COMPASS_INT			(42)
#define VIVO_GPIO_GSENSOR_INT_N			(180)
#define VIVO_LAYOUTS_XA_XB		{ \
		{ { 0,  1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
		{ { 0, -1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
		{ { 1,  0, 0}, { 0, -1,  0}, {0, 0, -1} }, \
		{ {-1,  0, 0}, { 0,  0, -1}, {0, 1,  0} }  \
					}
#define VIVO_LAYOUTS_XC			{ \
		{ { 0,  1, 0}, {-1,  0,  0}, {0, 0,  1} }, \
		{ { 0, -1, 0}, { 1,  0,  0}, {0, 0, -1} }, \
		{ {-1,  0, 0}, { 0, -1,  0}, {0, 0,  1} }, \
		{ {-1,  0, 0}, { 0,  0, -1}, {0, 1,  0} }  \
					}

#define VIVO_GPIO_UP_INT_N          (142)

#define VIVO_GPIO_PS_HOLD			(29)

#define VIVO_MDDI_RSTz				(162)
#define VIVO_LCD_ID0				(124)
#define VIVO_LCD_RSTz_ID1			(126)
#define VIVO_LCD_PCLK_1             (90)
#define VIVO_LCD_DE                 (91)
#define VIVO_LCD_VSYNC              (92)
#define VIVO_LCD_HSYNC              (93)
#define VIVO_LCD_G0                 (94)
#define VIVO_LCD_G1                 (95)
#define VIVO_LCD_G2                 (96)
#define VIVO_LCD_G3                 (97)
#define VIVO_LCD_G4                 (98)
#define VIVO_LCD_G5                 (99)
#define VIVO_LCD_B0					(22)
#define VIVO_LCD_B1                 (100)
#define VIVO_LCD_B2                 (101)
#define VIVO_LCD_B3                 (102)
#define VIVO_LCD_B4                 (103)
#define VIVO_LCD_B5                 (104)
#define VIVO_LCD_R0					(25)
#define VIVO_LCD_R1                 (105)
#define VIVO_LCD_R2                 (106)
#define VIVO_LCD_R3                 (107)
#define VIVO_LCD_R4                 (108)
#define VIVO_LCD_R5                 (109)

#define VIVO_LCD_SPI_CSz			(87)

/* Audio */
#define VIVO_AUD_SPI_CSz            (89)
#define VIVO_AUD_MICPATH_SEL_XA     (121)

/* Flashlight */
#define VIVO_GPIO_FLASHLIGHT_FLASH	(128)
#define VIVO_GPIO_FLASHLIGHT_TORCH	(129)

/* BT */
#define VIVO_GPIO_BT_UART1_RTS      (134)
#define VIVO_GPIO_BT_UART1_CTS      (135)
#define VIVO_GPIO_BT_UART1_RX       (136)
#define VIVO_GPIO_BT_UART1_TX       (137)
#define VIVO_GPIO_BT_PCM_OUT        (138)
#define VIVO_GPIO_BT_PCM_IN         (139)
#define VIVO_GPIO_BT_PCM_SYNC       (140)
#define VIVO_GPIO_BT_PCM_CLK        (141)
#define VIVO_GPIO_BT_RESET_N        (41)
#define VIVO_GPIO_BT_HOST_WAKE      (26)
#define VIVO_GPIO_BT_CHIP_WAKE      (50)
#define VIVO_GPIO_BT_SHUTDOWN_N     (38)

#define VIVO_GPIO_USB_ID_PIN		(145)

#define VIVO_VCM_PD					(34)
#define VIVO_CAM1_PD                (31)
#define VIVO_CLK_SEL                (23) /* camera select pin */
#define VIVO_CAM2_PD			    (24)
#define VIVO_CAM2_RSTz				(21)

#define VIVO_CODEC_3V_EN			(36)

/* PMIC 8058 GPIO */
#define PMGPIO(x) (x-1)
#define VIVO_GPIO_TP_INT_N		PMGPIO(1)
#define VIVO_AUD_MICPATH_SEL_XB		PMGPIO(10)
#define VIVO_AUD_EP_EN				PMGPIO(13)
#define VIVO_VOL_UP				PMGPIO(14)
#define VIVO_VOL_DN				PMGPIO(15)
#define VIVO_GPIO_CHG_INT		PMGPIO(16)
#define VIVO_AUD_HP_DETz		PMGPIO(17)
#define VIVO_AUD_SPK_EN			PMGPIO(18)
#define VIVO_AUD_HP_EN			PMGPIO(19)
#define VIVO_PS_SHDN			PMGPIO(20)
#define VIVO_TP_RSTz			PMGPIO(21)
#define VIVO_LED_3V3_EN			PMGPIO(22)
#define VIVO_SDMC_CD_N			PMGPIO(23)
#define VIVO_GREEN_LED			PMGPIO(24)
#define VIVO_AMBER_LED			PMGPIO(25)
#define VIVO_AUD_A3254_RSTz		PMGPIO(36)
#define VIVO_WIFI_SLOW_CLK		PMGPIO(38)
#define VIVO_SLEEP_CLK2			PMGPIO(39)
#endif


extern struct platform_device msm_device_mdp;
extern struct platform_device msm_device_mddi0;
extern int panel_type;

int vivo_panel_sleep_in(void);
#ifdef CONFIG_MICROP_COMMON
void __init vivo_microp_init(void);
#endif
int vivo_init_mmc(unsigned int sys_rev);
void __init vivo_audio_init(void);
int __init vivo_init_keypad(void);
int __init vivo_wifi_init(void);
int __init vivo_init_panel(void);

#endif /* __ARCH_ARM_MACH_MSM_BOARD_VIVO_H */

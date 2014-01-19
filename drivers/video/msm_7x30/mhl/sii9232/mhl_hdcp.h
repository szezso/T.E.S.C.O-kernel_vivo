/* drivers/video/msm/mhl/sii9232/mhl_hdcp.h - sii9232 MHL driver
 *
 * Copyright (C) 2010 HTC, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifdef _MHL_HDCP_C_

bool hdcp_check_support(void);
bool hdcp_check_aksv(void);
void hdcp_off(void);
void hdcp_init(struct mhl_info *info);
void hdcp_restart(void);
void hdcp_check_status(uint8_t InterruptStatusImage);

#else

extern bool HDCP_TxSupports;			// True if the TX supports HDCP... False otherwise.
extern bool HDCP_AksvValid;				// True if the TX has valid AKSVs... False otherwise.
extern bool HDCP_Started;				// True if the HDCP enable bit has been set... False otherwise.

extern void hdcp_on(const char *caller);
extern void hdcp_off(void);
extern void hdcp_init(struct mhl_info *info);
extern void hdcp_check_status(uint8_t InterruptStatusImage);

#endif

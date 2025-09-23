/*
 * FocalTech TouchScreen driver.
 *
 * Copyright (c) 2010-2017, FocalTech Systems, Ltd., all rights reserved.
 * Copyright (C) 2018 XiaoMi, Inc.
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

/*
 * File Name: focaltech_config.h
 * Author: Focaltech Driver Team
 * Created: 2016-08-08
 * Abstract: global configurations
 * Version: v1.0
*/
#ifndef _LINUX_FOCLATECH_CONFIG_H_
#define _LINUX_FOCLATECH_CONFIG_H_

/* G: A, I: B, S: C, U: D */
/* Chip type defines, do not modify */
#define _FT8716             0x87160805
#define _FT8736             0x87360806
#define _FT8006M            0x80060807
#define _FT8201             0x82010807
#define _FT7250             0x72500807
#define _FT8606             0x86060808
#define _FT8607             0x86070809
#define _FTE716             0xE716080A
#define _FT8006U            0x8006D80B
#define _FT8613             0x8613080C
#define _FT8719             0x8719080D

#define _FT5416             0x54160402
#define _FT5426             0x54260402
#define _FT5435             0x54350402
#define _FT5436             0x54360402
#define _FT5526             0x55260402
#define _FT5526I            0x5526B402
#define _FT5446             0x54460402
#define _FT5346             0x53460402
#define _FT5446I            0x5446B402
#define _FT5346I            0x5346B402
#define _FT7661             0x76610402
#define _FT7511             0x75110402
#define _FT7421             0x74210402
#define _FT7681             0x76810402
#define _FT3C47U            0x3C47D402
#define _FT3417             0x34170402
#define _FT3517             0x35170402
#define _FT3327             0x33270402
#define _FT3427             0x34270402
#define _FT7311             0x73110402

#define _FT5626             0x56260401
#define _FT5726             0x57260401
#define _FT5826B            0x5826B401
#define _FT5826S            0x5826C401
#define _FT7811             0x78110401
#define _FT3D47             0x3D470401
#define _FT3617             0x36170401
#define _FT3717             0x37170401
#define _FT3817B            0x3817B401
#define _FT3517U            0x3517D401

#define _FT6236U            0x6236D003
#define _FT6336G            0x6336A003
#define _FT6336U            0x6336D003
#define _FT6436U            0x6436D003

#define _FT3267             0x32670004
#define _FT3367             0x33670004
#define _FT5422U            0x5422D482
#define _FT3327DQQ_001      0x3327D482

/*
 * Choose your ic chip type of focaltech
 */
#define FTS_CHIP_TYPE _FT5726

/* Enables */
/* 1 to Enable, 0 to Disable */

/*
 * Show debug log info
 * Enable it for debug, disable it for release
 */
#define FTS_DEBUG_EN                            0

/*
 * Linux MultiTouch Protocol
 * 1: Protocol B(default), 0: Protocol A
 */
#define FTS_MT_PROTOCOL_B_EN                    1

/*
 * Report Pressure in multitouch
 * 1: Enable(default),0: Disable
 */
#define FTS_REPORT_PRESSURE_EN                  0

/*
 * Gesture function enable
 * Default: Disable
 */
#define FTS_GESTURE_EN                          1

/*
 * ESD check & protection
 * Default: Disable
 */
#define FTS_ESDCHECK_EN                         0

/*
 * Production test enable
 * 1: Enable, 0: Disable(Default)
 */
#define FTS_TEST_EN                             0

/*
 * Glove mode enable
 * 1: Enable, 0: Disable(default)
 */
#define FTS_GLOVE_EN                            0

/*
 * Cover enable
 * 1: Enable, 0: Disable(default)
 */
#define FTS_COVER_EN                            0
/*
 * Charger enable
 * 1: Enable, 0: Disable(default)
 */
#define FTS_CHARGER_EN                          0

/*
 * Nodes for tools, please keep enable
 */
#define FTS_SYSFS_NODE_EN                       0
#define FTS_APK_NODE_EN                         0

/*
 * Pinctrl Enable
 * Default: Disable
 */
#define FTS_PINCTRL_EN                          0

/*
 * Customer power enable
 * Enable it when customer need control TP power
 * Default: Disable
 */
#define FTS_POWER_SOURCE_CUST_EN                1

/* Upgrade */
/*
 * Auto upgrade, please keep enable
 */
#define FTS_AUTO_UPGRADE_EN                     0

#define FTS_LOCK_DOWN_INFO_EN				0
/*
 * Auto upgrade for lcd cfg
 */
#define FTS_AUTO_LIC_UPGRADE_EN                 0

/*
 * Check vendor_id number
 * 0: No check vendor_id (default)
 * 1/2/3: Check vendor_id for vendor compatibility
 */
#define FTS_GET_VENDOR_ID_NUM                   4

/*
 * vendor_id(s) for vendor(s) to be compatible with.
 * a confirmation of vendor_id(s) is recommended.
 * FTS_VENDOR_ID = PANEL_ID << 8 + VENDOR_ID
 * FTS_GET_VENDOR_ID_NUM == 0/1, no check vendor id, you may ignore them
 * FTS_GET_VENDOR_ID_NUM > 1, compatible with FTS_VENDOR_ID
 * FTS_GET_VENDOR_ID_NUM >= 2, compatible with FTS_VENDOR_ID2
 * FTS_GET_VENDOR_ID_NUM >= 3, compatible with FTS_VENDOR_ID3
 * FTS_GET_VENDOR_ID_NUM >= 4, compatible with FTS_VENDOR_ID4
 */
#define FTS_VENDOR_ID                          0x3b
#define FTS_VENDOR_ID2                         0x51
#define FTS_VENDOR_ID3                         0x3b
#define FTS_VENDOR_ID4                         0x51

#define FTS_CHIP_ID				0x54
#define FTS_CHIP_ID2			0x58
/*
 * FW.i file for auto upgrade, you must replace it with your own
 * define your own fw_file, the sample one to be replaced is invalid
 * NOTE: if FTS_GET_VENDOR_ID_NUM > 1, it's the fw corresponding with FTS_VENDOR_ID
 */
#define FTS_UPGRADE_FW_FILE "include/firmware/QL5018_D9_FT5526_Boen0x3b_Ver0x0e_20180908_app.i"

/*
 * if FTS_GET_VENDOR_ID_NUM >= 2, fw corrsponding with FTS_VENDOR_ID2
 * define your own fw_file, the sample one is invalid
 */
#define FTS_UPGRADE_FW2_FILE "include/firmware/QL5018_D9_FT5526_Ofilm0x51_Ver0x10_20180904_app.i"

/*
 * if FTS_GET_VENDOR_ID_NUM >= 3, fw corrsponding with FTS_VENDOR_ID3
 * define your own fw_file, the sample one is invalid
 */
#define FTS_UPGRADE_FW3_FILE "include/firmware/HQ_D9P_FT5726_Boen0x3b_Ver0x04_20180518_app.i"

/*
 * if FTS_GET_VENDOR_ID_NUM >= 4, fw corrsponding with FTS_VENDOR_ID4
 * define your own fw_file, the sample one is invalid
 */
#define FTS_UPGRADE_FW4_FILE "include/firmware/HQ_QL6019_D9P_FT5726_Ofilm0x51_Ver0x0a_20180830_app.i"

#endif /* _LINUX_FOCLATECH_CONFIG_H_ */

/**
 *******************************************************************************
 * @file  usb_dev_winusb_class.h
 * @brief Head file for usb_dev_winusb_class.c
 @verbatim
   Change Logs:
   Date             Author          Notes
   2024-05-31       CDT             First version
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2022-2025, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#ifndef __USB_DEV_WINUSB_CLASS_H__
#define __USB_DEV_WINUSB_CLASS_H__

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usb_dev_def.h"

/**
 * @addtogroup LL_USB_LIB
 * @{
 */

/**
 * @addtogroup LL_USB_DEV_CLASS
 * @{
 */

/**
 * @addtogroup LL_USB_WINUSB
 * @{
 */

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/
#define USB_WINUSB_CONFIG_DESC_SIZ             (32U)

#define MAX_WINUSB_IN_PACKET_SIZE              (MAX_WINUSB_PACKET_SIZE)
#define MAX_WINUSB_OUT_PACKET_SIZE             (MAX_WINUSB_PACKET_SIZE)

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/
extern usb_dev_class_func  class_winusb_cbk;

/* WinUSB Device library callbacks */
extern void usb_dev_winusb_init(void *pdev);;
extern void usb_dev_winusb_deinit(void *pdev);
extern uint8_t usb_dev_winusb_setup(void *pdev, USB_SETUP_REQ *req);
extern void usb_dev_winusb_datain(void *pdev, uint8_t epnum);
extern void usb_dev_winusb_dataout(void *pdev, uint8_t epnum);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif  /* __USB_DEV_WINUSB_CLASS_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/

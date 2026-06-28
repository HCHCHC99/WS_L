/**
 *******************************************************************************
 * @file  usb_host_cdc_class.c
 * @brief cdc class related functions
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
   2023-06-30       CDT             Modify for variable alignment
   2024-06-30       CDT             Optimize the interface init function
                                    Modify for MISRA
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

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "usb_host_cdc_class.h"
#include "usb_host_driver.h"

/**
 * @addtogroup LL_USB_LIB
 * @{
 */

/**
 * @addtogroup LL_USB_HOST_CLASS
 * @{
 */

/**
 * @addtogroup LL_USB_HOST_CDC USB Host CDC
 * @{
 */

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/
CDC_Usercb_TypeDef UserCb;

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
__USB_ALIGN_BEGIN static CDC_Machine_TypeDef CDC_Machine;

__USB_ALIGN_BEGIN static CDC_Xfer_TypeDef CDC_TxParam;
__USB_ALIGN_BEGIN static CDC_Xfer_TypeDef CDC_RxParam;

__USB_ALIGN_BEGIN static uint8_t TxBuf[CDC_BUFFER_SIZE];
__USB_ALIGN_BEGIN static uint8_t RxBuf[CDC_BUFFER_SIZE];

static uint8_t RX_Enabled = 0U;

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void usb_host_cdc_txrxparam_init(void);

static void usb_host_cdc_receivedata(CDC_Xfer_TypeDef *cdc_Data);

static void usb_host_cdc_senddata_process(usb_core_instance *pdev, USBH_HOST *phost);

static void usb_host_cdc_receivedata_process(usb_core_instance *pdev, USBH_HOST *phost);

static HOST_STATUS usb_host_cdc_interface_init(usb_core_instance *pdev, void *phost);

void usb_host_cdc_interface_deinit(usb_core_instance *pdev);

static HOST_STATUS usb_host_cdc_class_process(usb_core_instance *pdev, void *phost);

static HOST_STATUS usb_host_cdc_class_request(usb_core_instance *pdev, void *phost);

usb_host_class_callback_func CDC_cb = {
    usb_host_cdc_interface_init,
    usb_host_cdc_interface_deinit,
    usb_host_cdc_class_request,
    usb_host_cdc_class_process
};

/**
 * @brief  init the std cdc communication interface
 * @param  [in] pdev                device instance
 * @param  [in] phost               host state set
 * @param  [in] u8InterfaceIndex    interface index
 * @retval status defined by HOST_STATUS
 */
static HOST_STATUS usb_host_cdc_com_interface_init(usb_core_instance *pdev, void *phost, uint8_t u8InterfaceIndex)
{
    USBH_HOST *pphost = phost;
    HOST_STATUS status = HSTATUS_UNSUPPORTED;

    CDC_Machine.CDC_CommItf.cmdEp = 0U;
    CDC_Machine.CDC_CommItf.cmdEpSize = 0U;

    /* fill the communication endpoint address and length */
    if (0U != (pphost->device_prop.devepdesc[u8InterfaceIndex][0].bEndpointAddress & 0x80U)) {
        CDC_Machine.CDC_CommItf.cmdEp = pphost->device_prop.devepdesc[u8InterfaceIndex][0].bEndpointAddress;
        CDC_Machine.CDC_CommItf.cmdEpSize = pphost->device_prop.devepdesc[u8InterfaceIndex][0].wMaxPacketSize;

        /* distribute a channel for communication endpoint */
        CDC_Machine.CDC_CommItf.hc_num_cmd = usb_host_distrch(pdev, CDC_Machine.CDC_CommItf.cmdEp);

        /* open channel for communication endpoint */
        usb_host_chopen(pdev,
                        CDC_Machine.CDC_CommItf.hc_num_cmd,
                        pphost->device_prop.devaddr,
                        pphost->device_prop.devspeed,
                        EP_TYPE_INTR,
                        CDC_Machine.CDC_CommItf.cmdEpSize);

        status = HSTATUS_OK;
    }

    return status;
}

/**
 * @brief  init the std cdc data interface
 * @param  [in] pdev                device instance
 * @param  [in] phost               host state set
 * @param  [in] u8InterfaceIndex    interface index
 * @retval status defined by HOST_STATUS
 */
static HOST_STATUS usb_host_cdc_data_interface_init(usb_core_instance *pdev, void *phost, uint8_t u8InterfaceIndex)
{
    USBH_HOST *pphost = phost;
    HOST_STATUS status = HSTATUS_UNSUPPORTED;

    CDC_Machine.CDC_DataItf.cdcInEp = 0U;
    CDC_Machine.CDC_DataItf.cdcOutEp = 0U;

    if (0U != (pphost->device_prop.devepdesc[u8InterfaceIndex][0].bEndpointAddress & 0x80U)) {
        CDC_Machine.CDC_DataItf.cdcInEp = (pphost->device_prop.devepdesc[u8InterfaceIndex][0].bEndpointAddress);
        CDC_Machine.CDC_DataItf.cdcInEpSize = (pphost->device_prop.devepdesc[u8InterfaceIndex][0].wMaxPacketSize);
    } else {
        CDC_Machine.CDC_DataItf.cdcOutEp = (pphost->device_prop.devepdesc[u8InterfaceIndex][0].bEndpointAddress);
        CDC_Machine.CDC_DataItf.cdcOutEpSize = (pphost->device_prop.devepdesc[u8InterfaceIndex][0].wMaxPacketSize);
    }

    if (0U != (pphost->device_prop.devepdesc[u8InterfaceIndex][1].bEndpointAddress & 0x80U)) {
        CDC_Machine.CDC_DataItf.cdcInEp = (pphost->device_prop.devepdesc[u8InterfaceIndex][1].bEndpointAddress);
        CDC_Machine.CDC_DataItf.cdcInEpSize = (pphost->device_prop.devepdesc[u8InterfaceIndex][1].wMaxPacketSize);
    } else {
        CDC_Machine.CDC_DataItf.cdcOutEp = (pphost->device_prop.devepdesc[u8InterfaceIndex][1].bEndpointAddress);
        CDC_Machine.CDC_DataItf.cdcOutEpSize = (pphost->device_prop.devepdesc[u8InterfaceIndex][1].wMaxPacketSize);
    }

    /* check valid */
    if ((0U != CDC_Machine.CDC_DataItf.cdcOutEp) && (0U != CDC_Machine.CDC_DataItf.cdcInEp)) {
        /* distribute channels for cdc data endpoints */
        CDC_Machine.CDC_DataItf.hc_num_out = usb_host_distrch(pdev, CDC_Machine.CDC_DataItf.cdcOutEp);
        CDC_Machine.CDC_DataItf.hc_num_in = usb_host_distrch(pdev, CDC_Machine.CDC_DataItf.cdcInEp);

        /* open cdc data endpoints */
        usb_host_chopen(pdev,
                        CDC_Machine.CDC_DataItf.hc_num_out,
                        pphost->device_prop.devaddr,
                        pphost->device_prop.devspeed,
                        EP_TYPE_BULK,
                        CDC_Machine.CDC_DataItf.cdcOutEpSize);

        usb_host_chopen(pdev,
                        CDC_Machine.CDC_DataItf.hc_num_in,
                        pphost->device_prop.devaddr,
                        pphost->device_prop.devspeed,
                        EP_TYPE_BULK,
                        CDC_Machine.CDC_DataItf.cdcInEpSize);

        status = HSTATUS_OK;
    }

    return status;
}

/**
 * @brief  init the std cdc interface
 * @param  [in] pdev    device instance
 * @param  [in] phost   host state set
 * @retval status defined by HOST_STATUS
 */
static HOST_STATUS usb_host_cdc_interface_init(usb_core_instance *pdev, void *phost)
{
    USBH_HOST *pphost = phost;
    HOST_STATUS status;
    uint8_t u8InterfaceIndex = 0U;

    /* Communication Interface */
    if ((pphost->device_prop.devitfdesc[u8InterfaceIndex].bInterfaceClass == COMMUNICATION_DEVICE_CLASS_CODE) &&
        (pphost->device_prop.devitfdesc[u8InterfaceIndex].bInterfaceSubClass == ABSTRACT_CONTROL_MODEL) &&
        (pphost->device_prop.devitfdesc[u8InterfaceIndex].bInterfaceProtocol == COMMON_AT_COMMAND)) {
        status = usb_host_cdc_com_interface_init(pdev, phost, u8InterfaceIndex);
        u8InterfaceIndex++;
    } else {
        status = HSTATUS_UNSUPPORTED;
    }

    if (HSTATUS_OK != status) {
        pphost->user_callbk->huser_devunsupported();
    }

    /* Data Interface */
    if ((pphost->device_prop.devitfdesc[u8InterfaceIndex].bInterfaceClass == DATA_INTERFACE_CLASS_CODE) &&
        (pphost->device_prop.devitfdesc[u8InterfaceIndex].bInterfaceSubClass == RESERVED) &&
        (pphost->device_prop.devitfdesc[u8InterfaceIndex].bInterfaceProtocol == NO_CLASS_SPECIFIC_PROTOCOL_CODE)) {
        status = usb_host_cdc_data_interface_init(pdev, phost, u8InterfaceIndex);
    } else {
        status = HSTATUS_UNSUPPORTED;
    }

    if (HSTATUS_OK != status) {
        pphost->user_callbk->huser_devunsupported();

        /* If not a stand CDC ACM device, try vendor specific */
        if ((pphost->device_prop.devitfdesc[u8InterfaceIndex].bInterfaceClass == VENDOR_SPECIFIC) &&
            (pphost->device_prop.devitfdesc[u8InterfaceIndex].bInterfaceSubClass == RESERVED) &&
            (pphost->device_prop.devitfdesc[u8InterfaceIndex].bInterfaceProtocol == NO_CLASS_SPECIFIC_PROTOCOL_CODE)) {
            status = usb_host_cdc_data_interface_init(pdev, phost, u8InterfaceIndex);
        }
    }

    if (HSTATUS_OK == status) {
        /* txrx parms init */
        usb_host_cdc_txrxparam_init();
        /* host next state is HOST_CLASS_REQ, so set cdc request to GET_LINE_CODING */
        CDC_ReqState = CDC_GET_LINE_CODING_RQUEST;
    } else {
        /* device not support */
        pphost->user_callbk->huser_devunsupported();
        /* do nothing in next state */
        CDC_ReqState = CDC_IDLE_REQUEST;
    }

    /* Always return OK to avoid repeated interface init operations */
    return HSTATUS_OK;
}

/**
 * @brief  deinit the cdc interface
 * @param  [in] pdev    device instance
 * @retval None
 */
void usb_host_cdc_interface_deinit(usb_core_instance *pdev)
{
    /* halt and free all channels */
    if (0U != CDC_Machine.CDC_CommItf.hc_num_cmd) {
        usb_hchstop(&pdev->regs, CDC_Machine.CDC_CommItf.hc_num_cmd);
        (void)usb_host_freech(pdev, CDC_Machine.CDC_CommItf.hc_num_cmd);
        CDC_Machine.CDC_CommItf.hc_num_cmd = 0U;
    }

    if (0U != CDC_Machine.CDC_DataItf.hc_num_out) {
        usb_hchstop(&pdev->regs, CDC_Machine.CDC_DataItf.hc_num_out);
        (void)usb_host_freech(pdev, CDC_Machine.CDC_DataItf.hc_num_out);
        CDC_Machine.CDC_DataItf.hc_num_out = 0U;
    }

    if (0U != CDC_Machine.CDC_DataItf.hc_num_in) {
        usb_hchstop(&pdev->regs, CDC_Machine.CDC_DataItf.hc_num_in);
        (void)usb_host_freech(pdev, CDC_Machine.CDC_DataItf.hc_num_in);
        CDC_Machine.CDC_DataItf.hc_num_in = 0U;
    }
}

/**
 * @brief  handing host class request state
 * @param  [in] pdev    device instance
 * @param  [in] phost   host state set
 * @retval status defined by HOST_STATUS
 */
static HOST_STATUS usb_host_cdc_class_request(usb_core_instance *pdev, void *phost)
{
    USBH_HOST *pphost = phost;

    HOST_STATUS status = HSTATUS_BUSY;
    HOST_STATUS ClassReqStatus;

    switch (CDC_ReqState) {

        case CDC_GET_LINE_CODING_RQUEST:

            ClassReqStatus = usb_host_cdc_getlinecoding(pdev, phost);
            if (ClassReqStatus == HSTATUS_OK) {
                CDC_ReqState = CDC_SET_CONTROL_LINE_STATE_REQUEST;
            }
            break;

        case CDC_SET_LINE_CODING_RQUEST:

            ClassReqStatus = usb_host_cdc_setlinecoding(pdev, phost);
            if (ClassReqStatus == HSTATUS_OK) {
                CDC_ReqState = CDC_GET_LINE_CODING_RQUEST;
            }
            if (ClassReqStatus == HSTATUS_UNSUPPORTED) {
                /* Clear Feature should be issued */
                CDC_ReqState = CDC_ERROR_STATE;
            }
            break;

        case CDC_SET_CONTROL_LINE_STATE_REQUEST:

            ClassReqStatus = usb_host_cdc_setcontrollinestate(pdev, phost);
            if (ClassReqStatus == HSTATUS_OK) {
                /* change state to itself */
                CDC_ReqState = CDC_SET_CONTROL_LINE_STATE_REQUEST;
                /* change rx state to CDC_IDLE */
                CDC_RxParam.CDCState = CDC_IDLE;

                status = HSTATUS_OK;
            }
            break;

        case CDC_ERROR_STATE:

            ClassReqStatus = usb_host_clrfeature(pdev,
                                                 phost,
                                                 0x00U,
                                                 pphost->ctrlparam.hc_num_out);

            if (ClassReqStatus == HSTATUS_OK) {
                /* change state to GET_LINE_CODING */
                CDC_ReqState = CDC_GET_LINE_CODING_RQUEST;
            }
            break;
        default:
            break;
    }

    return status;
}

/**
 * @brief  process the cdc data send/receive state machine and user application
 * @param  [in] pdev   device instance
 * @param  [in] phost  host state set
 * @retval status defined by HOST_STATUS
 */
static HOST_STATUS usb_host_cdc_class_process(usb_core_instance *pdev, void *phost)
{
    HOST_STATUS status = HSTATUS_OK;
    USBH_HOST *pphost = phost;

    /* application process */
    pphost->user_callbk->huser_application();

    if (CDC_ReqState == CDC_SET_LINE_CODING_RQUEST) {
        return status;
    }
    /* send data process */
    usb_host_cdc_senddata_process(pdev, pphost);

    /* receive data process */
    usb_host_cdc_receivedata_process(pdev, pphost);

    return status;
}

/**
 * @brief  process the cdc data send state machine
 * @param  [in] pdev    device instance
 * @param  [in] phost   host state set
 * @retval None
 */
void usb_host_cdc_senddata_process(usb_core_instance *pdev, USBH_HOST *phost)
{
    static uint16_t len;
    HOST_CH_XFER_STATE URB_StatusTx;

    URB_StatusTx = host_driver_getxferstate(pdev, CDC_Machine.CDC_DataItf.hc_num_out);

    switch (CDC_TxParam.CDCState) {
        case CDC_IDLE:
            break;

        case CDC_SEND_DATA:

            if ((URB_StatusTx == HOST_CH_XFER_DONE) || (URB_StatusTx == HOST_CH_XFER_IDLE)) {
                /* check if send data len exceed CDC_DataItf.cdcOutEpSize */
                if (CDC_TxParam.DataLength > CDC_Machine.CDC_DataItf.cdcOutEpSize) {

                    len = CDC_Machine.CDC_DataItf.cdcOutEpSize;
                    /* send data */
                    usb_host_sendbulkdata(pdev,
                                          CDC_TxParam.pRxTxBuff,
                                          len,
                                          CDC_Machine.CDC_DataItf.hc_num_out);
                } else {
                    len = (uint16_t)CDC_TxParam.DataLength;
                    /* send all the remaining data */
                    usb_host_sendbulkdata(pdev,
                                          CDC_TxParam.pRxTxBuff,
                                          len,
                                          CDC_Machine.CDC_DataItf.hc_num_out);
                }
                CDC_TxParam.CDCState = CDC_DATA_SENT;
            }

            break;

        case CDC_DATA_SENT:
            /* check send complete */
            if (URB_StatusTx == HOST_CH_XFER_DONE) {
                /* move txbuffer point */
                CDC_TxParam.pRxTxBuff += len;

                /* ecrease data length */
                CDC_TxParam.DataLength -= len;

                if (0UL == CDC_TxParam.DataLength) {
                    CDC_TxParam.CDCState = CDC_IDLE;
                } else {
                    CDC_TxParam.CDCState = CDC_SEND_DATA;
                }
            } else if (URB_StatusTx == HOST_CH_XFER_UNREADY) {
                /* send again */
                usb_host_sendbulkdata(pdev,
                                      (CDC_TxParam.pRxTxBuff),
                                      len,
                                      CDC_Machine.CDC_DataItf.hc_num_out);
            } else {
                ;
            }

            break;

        case CDC_READ_DATA:
            break;

        case CDC_BUSY:
            break;

        case CDC_GET_DATA:
            break;

        case CDC_POLL:
            break;

        case CDC_CTRL_STATE:
            break;
        default:
            break;
    }
}

/**
 * @brief  process the cdc data receive state machine
 * @param  [in] pdev     device instance
 * @param  [in] phost    host state set
 * @retval None
 */
static void usb_host_cdc_receivedata_process(usb_core_instance *pdev, USBH_HOST *phost)
{

    if (1U == RX_Enabled) {
        HOST_CH_XFER_STATE URB_StatusRx = host_driver_getxferstate(pdev, CDC_Machine.CDC_DataItf.hc_num_in);

        switch (CDC_RxParam.CDCState) {

            case CDC_IDLE:

                /* check if free rxbuf exceed CDC_DataItf.cdcInEpSize */
                if (CDC_RxParam.DataLength < (CDC_RxParam.BufferLen - CDC_Machine.CDC_DataItf.cdcInEpSize)) {
                    /* receive data */
                    usb_host_recvbulkdata(pdev,
                                          CDC_RxParam.pFillBuff,
                                          CDC_Machine.CDC_DataItf.cdcInEpSize,
                                          CDC_Machine.CDC_DataItf.hc_num_in);

                    /* change sate to wait receive complete */
                    CDC_RxParam.CDCState = CDC_GET_DATA;
                }
                break;

            case CDC_GET_DATA:
                /* check XFER_DONE */
                if (URB_StatusRx == HOST_CH_XFER_DONE) {
                    /* increase rx data len */
                    CDC_RxParam.DataLength += pdev->host.hc[CDC_Machine.CDC_DataItf.hc_num_in].xfer_count;
                    /* move rxbuff poinit */
                    CDC_RxParam.pFillBuff += pdev->host.hc[CDC_Machine.CDC_DataItf.hc_num_in].xfer_count;

                    /* process the received data */
                    usb_host_cdc_receivedata(&CDC_RxParam);

                    /* chage state back to CDC_IDLE */
                    CDC_RxParam.CDCState = CDC_IDLE;
                }
                break;

            case CDC_READ_DATA:

                break;

            case CDC_BUSY:

                break;

            case CDC_SEND_DATA:

                break;

            case CDC_DATA_SENT:

                break;

            case CDC_POLL:

                break;

            case CDC_CTRL_STATE:

                break;
            default:
                break;
        }
    }
}

/**
 * @brief  init tx rx buffer for cdc
 * @param  None
 * @retval None
 */
static void usb_host_cdc_txrxparam_init(void)
{
    /* init tx buffer */
    CDC_TxParam.CDCState = CDC_IDLE;
    CDC_TxParam.DataLength = 0UL;
    CDC_TxParam.pRxTxBuff = TxBuf;

    /* init rx buffer */
    CDC_RxParam.CDCState = CDC_IDLE;
    CDC_RxParam.DataLength = 0UL;
    CDC_RxParam.pFillBuff = RxBuf;
    CDC_RxParam.pEmptyBuff = RxBuf;
    CDC_RxParam.BufferLen = sizeof(RxBuf);
}

/**
 * @brief  call user callback fucntion to process the received data
 * @param  [in] cdc_Data    type of CDC_Xfer_TypeDef
 * @retval None
 */
static void usb_host_cdc_receivedata(CDC_Xfer_TypeDef *cdc_Data)
{
    uint8_t *ptr;

    if (cdc_Data->pEmptyBuff < cdc_Data->pFillBuff) {
        ptr = cdc_Data->pFillBuff;
        *ptr = 0x00U;

        /* callback user function to process received data */
        UserCb.Receive(cdc_Data->pEmptyBuff, cdc_Data->DataLength);

        cdc_Data->pFillBuff = cdc_Data->pEmptyBuff;
        cdc_Data->DataLength = 0UL;
    }
}

/**
 * @brief  user call this function to send data to cdc device
 * @param  [in] data     send data buffer
 * @param  [in] length   send length
 * @retval None
 */
void usb_host_cdc_senddata(uint8_t *data, uint32_t length)
{

    if (CDC_TxParam.CDCState == CDC_IDLE) {
        CDC_TxParam.pRxTxBuff = data;
        CDC_TxParam.DataLength = length;
        CDC_TxParam.CDCState = CDC_SEND_DATA;
    }
}

/**
 * @brief  user call this function to enable receive data from device
 * @param  [in] pdev    device instance
 * @retval None
 */
void usb_host_cdc_enable_receive(usb_core_instance *pdev)
{
    RX_Enabled = 1U;
}

/**
 * @brief  user call this function to disable receive data from device
 * @param  [in] pdev    device instance
 * @retval None
 */
void usb_host_cdc_disable_receive(usb_core_instance *pdev)
{
    RX_Enabled = 0U;
    usb_hchstop(&pdev->regs, CDC_Machine.CDC_DataItf.hc_num_in);
    (void)usb_host_freech(pdev, CDC_Machine.CDC_DataItf.hc_num_in);
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/

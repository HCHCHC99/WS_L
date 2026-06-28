#include "dev_adc.h"
#include "TickTimer.h"
#include "rtt_log.h"
#include <string.h>
#include <stdlib.h>

// ========== 全局 Adp 层初始化控制 ==========
// 标记 Adp 层的 ADC 硬件是否已经初始化
// 第一个 ADC 设备初始化时会自动调用 Adc_Create/Adc_Init/Adc_Start
// 使用 uint8_t 替代 bool，兼容 Keil MDK C99 模式
static uint8_t s_bAdpAdcInitialized = 0U;

// ========== 内部辅助函数 ==========

// ADC值转电压(mV)
static uint16_t ADC_ValueToVoltage(uint16_t u16AdcValue)
{
    // Vref = 3.3V, 12bit ADC (0~4095)
    // Voltage(mV) = adc_value * 3300 / 4095
    return (uint16_t)(((uint32_t)u16AdcValue * 3300UL) / 4095UL);
}

// 更新ADC缓存数据（根据采集模式从Adapter层获取数据）
static void ADC_UpdateCache(ADC_Device_t* pstcDev)
{
    if (pstcDev == NULL) return;

    if (pstcDev->stcConfig.enAcqMode == ADC_ACQ_MODE_INTERRUPT)
    {
        // 中断模式：直接从Adapter层获取最新值
        pstcDev->u16RawValue = Adc_GetLatestValue(pstcDev->stcConfig.u8AdcId);
        pstcDev->u16VoltageMv = ADC_ValueToVoltage(pstcDev->u16RawValue);
        pstcDev->u16AverageValue = pstcDev->u16RawValue; // 中断模式下平均值等于最新值

        ADC_OUT("ADC_ID=%d, CH=%d, Mode=INT, Raw=%d, %dmV\r\n",
                pstcDev->stcConfig.u8AdcId,
                pstcDev->stcConfig.u8Channel,
                pstcDev->u16RawValue,
                pstcDev->u16VoltageMv);
    }
    else if (pstcDev->stcConfig.enAcqMode == ADC_ACQ_MODE_AOS_DMA)
    {
        // AOS+DMA模式：从Adapter层获取最新值和平均值
        pstcDev->u16RawValue = Adc_GetLatestValue(pstcDev->stcConfig.u8AdcId);
        pstcDev->u16VoltageMv = ADC_ValueToVoltage(pstcDev->u16RawValue);
        pstcDev->u16AverageValue = Adc_GetAverageValue(pstcDev->stcConfig.u8AdcId);

        ADC_OUT("ADC_ID=%d, CH=%d, Mode=DMA, Raw=%d, %dmV, Avg=%d\r\n",
                pstcDev->stcConfig.u8AdcId,
                pstcDev->stcConfig.u8Channel,
                pstcDev->u16RawValue,
                pstcDev->u16VoltageMv,
                pstcDev->u16AverageValue);
    }
    else
    {
        ADC_DEBUG("Unknown ADC acquisition mode: %d\r\n", (int)pstcDev->stcConfig.enAcqMode);
    }
}

/**
 * @brief  在 Adp 层创建并初始化 ADC 实例（仅执行一次）
 * @note   此函数将当前 dev_adc 设备的配置注册到 Adp/Adc.c 中，
 *         并在第一个设备初始化时统一调用 Adc_Init() 和 Adc_Start()
 *         完成所有底层硬件初始化。
 *         这样即使 Hardware.c 中注释掉 ADC 相关代码，ADC 仍能正常工作。
 */
static void ADC_AdpLayerInit(ADC_Device_t* pstcDev)
{
    stc_adc_config_t stcAdcConfig;

    /* 构造 Adp 层配置 */
    stcAdcConfig.u8Channel = pstcDev->stcConfig.u8Channel;
    stcAdcConfig.stcPin.u8Port = pstcDev->stcConfig.u8Port;
    stcAdcConfig.stcPin.u8Pin = pstcDev->stcConfig.u16Pin;
    stcAdcConfig.pfnCallback = NULL;  // dev_adc 层不使用 Adp 层的回调

    if (pstcDev->stcConfig.enAcqMode == ADC_ACQ_MODE_INTERRUPT)
    {
        stcAdcConfig.enMode = ADC_MODE_INTERRUPT;
        stcAdcConfig.stcDmaConfig.u16BufferSize = 0;
        stcAdcConfig.stcDmaConfig.u8DmaChannel = 0;
    }
    else /* ADC_ACQ_MODE_AOS_DMA */
    {
        stcAdcConfig.enMode = ADC_MODE_DMA;
        stcAdcConfig.stcDmaConfig.u16BufferSize = pstcDev->stcConfig.u16DmaBufferSize;
        stcAdcConfig.stcDmaConfig.u8DmaChannel = pstcDev->stcConfig.u8DmaChannel;
    }

    /* 在 Adp 层创建 ADC 实例 */
    uint8_t u8AdpId = Adc_Create(&stcAdcConfig);
    if (u8AdpId == 0xFF)
    {
        ADC_DEBUG("Adp_Create failed for CH%d (maybe duplicate?)\r\n", pstcDev->stcConfig.u8Channel);
        return;
    }

    /* 保存 Adp 层返回的 ID */
    pstcDev->stcConfig.u8AdcId = u8AdpId;

    ADC_DEBUG("Adp layer created: dev_ADC_ID=%d -> adp_ADC_ID=%d, CH=%d\r\n",
              pstcDev->stcConfig.u8AdcId, u8AdpId, pstcDev->stcConfig.u8Channel);

    /* 配置引脚为模拟模式（无论是否首次，每个新通道都需要配置）*/
    {
        stc_gpio_init_t stcGpioInit;
        (void)GPIO_StructInit(&stcGpioInit);
        stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
        LL_PERIPH_WE(LL_PERIPH_GPIO);
        (void)GPIO_Init(pstcDev->stcConfig.u8Port, pstcDev->stcConfig.u16Pin, &stcGpioInit);
        LL_PERIPH_WP(LL_PERIPH_GPIO);

        /* 使能新通道到 SEQ_A */
        ADC_ChCmd(ADC_UNIT, ADC_SEQ_A, pstcDev->stcConfig.u8Channel, ENABLE);

        ADC_DEBUG("ADC CH%d pin configured (P%c%d) and SEQ_A enabled\r\n",
                  pstcDev->stcConfig.u8Channel,
                  (pstcDev->stcConfig.u8Port == GPIO_PORT_A) ? 'A' :
                  (pstcDev->stcConfig.u8Port == GPIO_PORT_B) ? 'B' :
                  (pstcDev->stcConfig.u8Port == GPIO_PORT_C) ? 'C' :
                  (pstcDev->stcConfig.u8Port == GPIO_PORT_D) ? 'D' :
                  (pstcDev->stcConfig.u8Port == GPIO_PORT_E) ? 'E' : 'H',
                  (pstcDev->stcConfig.u16Pin == GPIO_PIN_00) ? 0U :
                  (pstcDev->stcConfig.u16Pin == GPIO_PIN_01) ? 1U :
                  (pstcDev->stcConfig.u16Pin == GPIO_PIN_02) ? 2U :
                  (pstcDev->stcConfig.u16Pin == GPIO_PIN_03) ? 3U :
                  (pstcDev->stcConfig.u16Pin == GPIO_PIN_04) ? 4U :
                  (pstcDev->stcConfig.u16Pin == GPIO_PIN_05) ? 5U :
                  (pstcDev->stcConfig.u16Pin == GPIO_PIN_06) ? 6U :
                  (pstcDev->stcConfig.u16Pin == GPIO_PIN_07) ? 7U :
                  (pstcDev->stcConfig.u16Pin == GPIO_PIN_08) ? 8U :
                  (pstcDev->stcConfig.u16Pin == GPIO_PIN_09) ? 9U :
                  (pstcDev->stcConfig.u16Pin == GPIO_PIN_10) ? 10U :
                  (pstcDev->stcConfig.u16Pin == GPIO_PIN_11) ? 11U :
                  (pstcDev->stcConfig.u16Pin == GPIO_PIN_12) ? 12U :
                  (pstcDev->stcConfig.u16Pin == GPIO_PIN_13) ? 13U :
                  (pstcDev->stcConfig.u16Pin == GPIO_PIN_14) ? 14U : 15U);
    }

    /* 如果是第一个设备，统一初始化 Adp 层硬件并启动 */
    if (!s_bAdpAdcInitialized)
    {
        ADC_DEBUG("First ADC device: initializing Adp layer hardware...\r\n");

        /* 统一初始化所有已创建的 Adp ADC 实例 */
        Adc_Init();

        /* 启动 ADC 转换 */
        Adc_Start();

        s_bAdpAdcInitialized = 1U;

        ADC_DEBUG("Adp layer ADC hardware initialized and started\r\n");
    }
    else
    {
        /* 非第一个设备：Adc_Init() 已调用过，但新通道可能是中断模式
         * 需要重新配置 ADC 中断，确保新通道的中断模式能正常工作 */
        if (pstcDev->stcConfig.enAcqMode == ADC_ACQ_MODE_INTERRUPT)
        {
            ADC_DEBUG("Reconfiguring ADC interrupt for new interrupt-mode CH%d...\r\n",
                      pstcDev->stcConfig.u8Channel);

            /* 调用 Adp 层的公共接口重新使能 ADC 中断 */
            Adc_EnableInterrupt();

            ADC_DEBUG("ADC interrupt reconfigured for CH%d\r\n", pstcDev->stcConfig.u8Channel);
        }
    }
}

// ========== 标准设备操作实现 ==========

DeviceResult_t ADC_Device_Init(void* handle)
{
    ADC_Device_t* pstcDev = (ADC_Device_t*)handle;
    if (pstcDev == NULL) return RESULT_PARAM_ERR;

    // 获取端口字母用于打印
    char cPort = '?';
    switch (pstcDev->stcConfig.u8Port)
    {
        case GPIO_PORT_A: cPort = 'A'; break;
        case GPIO_PORT_B: cPort = 'B'; break;
        case GPIO_PORT_C: cPort = 'C'; break;
        case GPIO_PORT_D: cPort = 'D'; break;
        case GPIO_PORT_E: cPort = 'E'; break;
        case GPIO_PORT_H: cPort = 'H'; break;
        default: break;
    }

    // 获取引脚号用于打印
    uint8_t u8PinNum = 0;
    switch (pstcDev->stcConfig.u16Pin)
    {
        case GPIO_PIN_00: u8PinNum = 0;  break;
        case GPIO_PIN_01: u8PinNum = 1;  break;
        case GPIO_PIN_02: u8PinNum = 2;  break;
        case GPIO_PIN_03: u8PinNum = 3;  break;
        case GPIO_PIN_04: u8PinNum = 4;  break;
        case GPIO_PIN_05: u8PinNum = 5;  break;
        case GPIO_PIN_06: u8PinNum = 6;  break;
        case GPIO_PIN_07: u8PinNum = 7;  break;
        case GPIO_PIN_08: u8PinNum = 8;  break;
        case GPIO_PIN_09: u8PinNum = 9;  break;
        case GPIO_PIN_10: u8PinNum = 10; break;
        case GPIO_PIN_11: u8PinNum = 11; break;
        case GPIO_PIN_12: u8PinNum = 12; break;
        case GPIO_PIN_13: u8PinNum = 13; break;
        case GPIO_PIN_14: u8PinNum = 14; break;
        case GPIO_PIN_15: u8PinNum = 15; break;
        default: break;
    }

    ADC_DEBUG("Init: ADC_ID=%d, CH=%d, Pin=P%c%d, Mode=%s\r\n",
              pstcDev->stcConfig.u8AdcId,
              pstcDev->stcConfig.u8Channel,
              cPort,
              u8PinNum,
              (pstcDev->stcConfig.enAcqMode == ADC_ACQ_MODE_INTERRUPT) ? "Interrupt" : "AOS+DMA");

    // ★ 关键：在 Adp 层创建并初始化 ADC 硬件（仅首次执行完整初始化）★
    ADC_AdpLayerInit(pstcDev);

    // 初始化缓存数据
    pstcDev->u16RawValue = 0;
    pstcDev->u16VoltageMv = 0;
    pstcDev->u16AverageValue = 0;

    // 如果是AOS+DMA模式，从Adapter层获取DMA缓冲区指针
    if (pstcDev->stcConfig.enAcqMode == ADC_ACQ_MODE_AOS_DMA)
    {
        // DMA缓冲区由Adapter层管理，这里记录大小用于后续计算
        pstcDev->u16DmaBufferSize = pstcDev->stcConfig.u16DmaBufferSize;
        pstcDev->pu16DmaBuffer = NULL; // 缓冲区由Adapter层维护，我们不直接操作
    }
    else
    {
        pstcDev->u16DmaBufferSize = 0;
        pstcDev->pu16DmaBuffer = NULL;
    }

    // 首次更新缓存
    ADC_UpdateCache(pstcDev);

    pstcDev->u8Initialized = 1;
    pstcDev->u32LastUpdateTime = tickTimer_GetCount();

    ADC_DEBUG("Init success: ADC_ID=%d\r\n", pstcDev->stcConfig.u8AdcId);

    return RESULT_OK;
}

DeviceResult_t ADC_Device_Deinit(void* handle)
{
    ADC_Device_t* pstcDev = (ADC_Device_t*)handle;
    if (pstcDev == NULL) return RESULT_PARAM_ERR;
    if (!pstcDev->u8Initialized) return RESULT_OK;

    ADC_DEBUG("Deinit: ADC_ID=%d\r\n", pstcDev->stcConfig.u8AdcId);

    pstcDev->u8Initialized = 0;
    pstcDev->u16RawValue = 0;
    pstcDev->u16VoltageMv = 0;
    pstcDev->u16AverageValue = 0;

    return RESULT_OK;
}

DeviceResult_t ADC_Device_Read(void* handle, void* data, uint32_t size)
{
    ADC_Device_t* pstcDev = (ADC_Device_t*)handle;
    if (pstcDev == NULL || data == NULL) return RESULT_PARAM_ERR;
    if (!pstcDev->u8Initialized) return RESULT_ERROR;

    if (size == sizeof(uint16_t))
    {
        // 读取原始值
        *(uint16_t*)data = pstcDev->u16RawValue;
    }
    else if (size == sizeof(ADC_ReadResponse_t))
    {
        // 读取完整响应结构体
        ADC_ReadResponse_t* pstcResp = (ADC_ReadResponse_t*)data;
        pstcResp->u16RawValue = pstcDev->u16RawValue;
        pstcResp->u16VoltageMv = pstcDev->u16VoltageMv;
        pstcResp->u16AverageValue = pstcDev->u16AverageValue;
    }
    else
    {
        return RESULT_PARAM_ERR;
    }

    return RESULT_OK;
}

DeviceResult_t ADC_Device_Write(void* handle, const void* data, uint32_t size)
{
    (void)handle;
    (void)data;
    (void)size;
    // ADC设备不支持写入操作
    return RESULT_ERROR;
}

DeviceResult_t ADC_Device_Control(void* handle, DeviceCommandData_t* pstcCmd)
{
    ADC_Device_t* pstcDev = (ADC_Device_t*)handle;
    if (pstcDev == NULL || pstcCmd == NULL) return RESULT_PARAM_ERR;
    if (!pstcDev->u8Initialized) return RESULT_ERROR;

    switch (pstcCmd->cmd)
    {
        case CMD_ADC_GET_RAW_VALUE:
            if (pstcCmd->response != NULL && pstcCmd->response_size >= sizeof(uint16_t))
            {
                *(uint16_t*)pstcCmd->response = pstcDev->u16RawValue;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;

        case CMD_ADC_GET_VOLTAGE_MV:
            if (pstcCmd->response != NULL && pstcCmd->response_size >= sizeof(uint16_t))
            {
                *(uint16_t*)pstcCmd->response = pstcDev->u16VoltageMv;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;

        case CMD_ADC_GET_AVERAGE_VALUE:
            if (pstcCmd->response != NULL && pstcCmd->response_size >= sizeof(uint16_t))
            {
                *(uint16_t*)pstcCmd->response = pstcDev->u16AverageValue;
                return RESULT_OK;
            }
            return RESULT_PARAM_ERR;

        default:
            return RESULT_ERROR;
    }
}

DeviceResult_t ADC_Device_Update(void* handle)
{
    ADC_Device_t* pstcDev = (ADC_Device_t*)handle;
    if (pstcDev == NULL) return RESULT_ERROR;
    if (!pstcDev->u8Initialized) return RESULT_ERROR;

    // 更新缓存数据（根据采集模式从Adapter层获取最新数据）
    ADC_UpdateCache(pstcDev);

    pstcDev->u32LastUpdateTime = tickTimer_GetCount();

    return RESULT_OK;
}

// ========== ADC特定接口 ==========

uint16_t ADC_Device_GetRawValue(ADC_Device_t* pstcDev)
{
    if (pstcDev == NULL || !pstcDev->u8Initialized) return 0;
    return pstcDev->u16RawValue;
}

uint16_t ADC_Device_GetVoltageMV(ADC_Device_t* pstcDev)
{
    if (pstcDev == NULL || !pstcDev->u8Initialized) return 0;
    return pstcDev->u16VoltageMv;
}

uint16_t ADC_Device_GetAverageValue(ADC_Device_t* pstcDev)
{
    if (pstcDev == NULL || !pstcDev->u8Initialized) return 0;
    return pstcDev->u16AverageValue;
}

ADC_Device_t* ADC_Device_Create(const ADC_Config_t* pstcConfig)
{
    if (pstcConfig == NULL) return NULL;

    ADC_Device_t* pstcDev = (ADC_Device_t*)malloc(sizeof(ADC_Device_t));
    if (pstcDev == NULL)
    {
        ADC_DEBUG("Failed to allocate memory for ADC device\r\n");
        return NULL;
    }

    memset(pstcDev, 0, sizeof(ADC_Device_t));
    memcpy(&pstcDev->stcConfig, pstcConfig, sizeof(ADC_Config_t));

    ADC_DEBUG("Create: ADC_ID=%d, CH=%d, Mode=%s\r\n",
              pstcConfig->u8AdcId,
              pstcConfig->u8Channel,
              (pstcConfig->enAcqMode == ADC_ACQ_MODE_INTERRUPT) ? "Interrupt" : "AOS+DMA");

    pstcDev->u8Initialized = 0;
    return pstcDev;
}

// ========== 全局操作函数表 ==========
const DeviceOps_t g_adc_ops = {
    .init    = ADC_Device_Init,
    .deinit = ADC_Device_Deinit,
    .read    = ADC_Device_Read,
    .write   = ADC_Device_Write,
    .control = ADC_Device_Control,
    .update  = ADC_Device_Update
};

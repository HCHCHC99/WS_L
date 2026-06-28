/**
 *******************************************************************************
 * @file  Dma.c
 * @brief DMA Driver for HC32F460 - Generic framework with multiple channels
 *        直接使用 hc32_ll_dma.h 中的宏定义
 *******************************************************************************
 */

#include "Dma.h"
#include "rtt_log.h"
#include <string.h>
#include <stdlib.h>

/*******************************************************************************
 * Local variables ('static')
 ******************************************************************************/

/* DMA 实例数组 */
static stc_dma_instance_t s_astcDmaInstances[DMA_MAX_INSTANCES];
static uint8_t s_u8DmaInstanceCount = 0;

/* DMA 全局初始化标志 */
static uint8_t s_bDmaInitialized = 0;

/* 中断服务函数映射表：[DMA单元-1][通道号] → 实例ID */
static int8_t s_a8DmaIrqMap[2][DMA_MAX_CHANNELS];

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

static CM_DMA_TypeDef* Dma_GetUnitPtr(uint8_t u8DmaUnit);
static uint32_t Dma_GetPeriphClk(uint8_t u8DmaUnit);
static void Dma_AllocateBuffer(stc_dma_instance_t *pstcInst);
static void Dma_FreeBuffer(stc_dma_instance_t *pstcInst);
static void Dma_IrqConfig(stc_dma_instance_t *pstcInst);

/* 每个 DMA 通道独立的中断处理函数 */
static void Dma1_Ch0_IrqHandler(void);
static void Dma1_Ch1_IrqHandler(void);
static void Dma1_Ch2_IrqHandler(void);
static void Dma1_Ch3_IrqHandler(void);
static void Dma2_Ch0_IrqHandler(void);
static void Dma2_Ch1_IrqHandler(void);
static void Dma2_Ch2_IrqHandler(void);
static void Dma2_Ch3_IrqHandler(void);

/*******************************************************************************
 * Local functions
 ******************************************************************************/

/**
 * @brief  获取 DMA 单元指针
 */
static CM_DMA_TypeDef* Dma_GetUnitPtr(uint8_t u8DmaUnit)
{
    return (u8DmaUnit == 1) ? CM_DMA1 : CM_DMA2;
}

/**
 * @brief  获取 DMA 外设时钟
 */
static uint32_t Dma_GetPeriphClk(uint8_t u8DmaUnit)
{
    return (u8DmaUnit == 1) ? FCG0_PERIPH_DMA1 : FCG0_PERIPH_DMA2;
}

/**
 * @brief  分配 DMA 缓冲区
 */
static void Dma_AllocateBuffer(stc_dma_instance_t *pstcInst)
{
    if (pstcInst->pu16Buffer != NULL) {
        return;
    }

    pstcInst->pu16Buffer = (uint16_t *)malloc(pstcInst->u16BufferSize * sizeof(uint16_t));
    if (pstcInst->pu16Buffer != NULL) {
        memset(pstcInst->pu16Buffer, 0, pstcInst->u16BufferSize * sizeof(uint16_t));
        DMA_Adp_DEBUG("Buffer allocated for DMA%d CH%d: size=%d bytes, addr=0x%08X\r\n",
               pstcInst->stcConfig.u8DmaUnit, pstcInst->stcConfig.u8Channel,
               pstcInst->u16BufferSize * sizeof(uint16_t),
               (uint32_t)pstcInst->pu16Buffer);
    } else {
        DMA_Adp_DEBUG("Failed to allocate buffer for DMA%d CH%d\r\n",
               pstcInst->stcConfig.u8DmaUnit, pstcInst->stcConfig.u8Channel);
    }
}

/**
 * @brief  释放 DMA 缓冲区
 */
static void Dma_FreeBuffer(stc_dma_instance_t *pstcInst)
{
    if (pstcInst->pu16Buffer != NULL) {
        free(pstcInst->pu16Buffer);
        pstcInst->pu16Buffer = NULL;
        DMA_Adp_DEBUG("Buffer freed for DMA%d CH%d\r\n",
               pstcInst->stcConfig.u8DmaUnit, pstcInst->stcConfig.u8Channel);
    }
}

/**
 * @brief  DMA 中断配置
 */
static void Dma_IrqConfig(stc_dma_instance_t *pstcInst)
{
    stc_irq_signin_config_t stcIrqSignConfig;
    uint8_t u8DmaUnit = pstcInst->stcConfig.u8DmaUnit;
    uint8_t u8Ch = pstcInst->stcConfig.u8Channel;

    /* 计算中断源和 IRQn */
    uint32_t u32IntSrc;
    IRQn_Type enIRQn;
    func_ptr_t pfnCallback = NULL;

    if (u8DmaUnit == 1) {
        u32IntSrc = INT_SRC_DMA1_BTC0 + u8Ch;
        enIRQn = (IRQn_Type)(INT038_IRQn + u8Ch);
        switch (u8Ch) {
            case 0: pfnCallback = Dma1_Ch0_IrqHandler; break;
            case 1: pfnCallback = Dma1_Ch1_IrqHandler; break;
            case 2: pfnCallback = Dma1_Ch2_IrqHandler; break;
            case 3: pfnCallback = Dma1_Ch3_IrqHandler; break;
            default: return;
        }
    } else {
        u32IntSrc = INT_SRC_DMA2_BTC0 + u8Ch;
        enIRQn = (IRQn_Type)(INT042_IRQn + u8Ch);
        switch (u8Ch) {
            case 0: pfnCallback = Dma2_Ch0_IrqHandler; break;
            case 1: pfnCallback = Dma2_Ch1_IrqHandler; break;
            case 2: pfnCallback = Dma2_Ch2_IrqHandler; break;
            case 3: pfnCallback = Dma2_Ch3_IrqHandler; break;
            default: return;
        }
    }

    stcIrqSignConfig.enIntSrc    = (en_int_src_t)u32IntSrc;
    stcIrqSignConfig.enIRQn      = enIRQn;
    stcIrqSignConfig.pfnCallback = pfnCallback;

    (void)INTC_IrqSignIn(&stcIrqSignConfig);

    /* 清除挂起标志 */
    CM_DMA_TypeDef *pDma = Dma_GetUnitPtr(u8DmaUnit);
    DMA_ClearTransCompleteStatus(pDma, (DMA_FLAG_BTC_CH0 << u8Ch));

    NVIC_ClearPendingIRQ(enIRQn);
    NVIC_SetPriority(enIRQn, pstcInst->stcConfig.u8IntPriority);
    NVIC_EnableIRQ(enIRQn);

    DMA_Adp_DEBUG("DMA%d CH%d interrupt configured (IRQn=%d, priority=%d)\r\n",
           u8DmaUnit, u8Ch, (int)enIRQn, pstcInst->stcConfig.u8IntPriority);
}

/**
 * @brief  DMA 中断通用处理函数
 * @param  u8DmaUnit DMA 单元 (1 或 2)
 * @param  u8Ch      通道号 (0-3)
 */
static void Dma_IrqHandler(uint8_t u8DmaUnit, uint8_t u8Ch)
{
    CM_DMA_TypeDef *pDma = Dma_GetUnitPtr(u8DmaUnit);

    /* 清除中断标志 */
    DMA_ClearTransCompleteStatus(pDma, (DMA_FLAG_BTC_CH0 << u8Ch));

    /* 查找实例 */
    int8_t s8Id = s_a8DmaIrqMap[u8DmaUnit - 1][u8Ch];
    if (s8Id >= 0 && s8Id < (int8_t)s_u8DmaInstanceCount) {
        stc_dma_instance_t *pstcInst = &s_astcDmaInstances[s8Id];
        pstcInst->u8DataUpdated = 1U;
        pstcInst->u32TransferCount++;

        /* 调用用户回调 */
        if (pstcInst->stcConfig.pfnCallback != NULL) {
            pstcInst->stcConfig.pfnCallback();
        }
    }
}

/* ===== DMA1 各通道中断处理函数 ===== */
static void Dma1_Ch0_IrqHandler(void) { Dma_IrqHandler(1, 0); }
static void Dma1_Ch1_IrqHandler(void) { Dma_IrqHandler(1, 1); }
static void Dma1_Ch2_IrqHandler(void) { Dma_IrqHandler(1, 2); }
static void Dma1_Ch3_IrqHandler(void) { Dma_IrqHandler(1, 3); }

/* ===== DMA2 各通道中断处理函数 ===== */
static void Dma2_Ch0_IrqHandler(void) { Dma_IrqHandler(2, 0); }
static void Dma2_Ch1_IrqHandler(void) { Dma_IrqHandler(2, 1); }
static void Dma2_Ch2_IrqHandler(void) { Dma_IrqHandler(2, 2); }
static void Dma2_Ch3_IrqHandler(void) { Dma_IrqHandler(2, 3); }

/*******************************************************************************
 * API functions - 实例管理
 ******************************************************************************/

/**
 * @brief  创建 DMA 实例
 */
uint8_t Dma_Create(stc_dma_config_t *pstcConfig)
{
    if (pstcConfig == NULL) {
        return 0xFF;
    }

    if (s_u8DmaInstanceCount >= DMA_MAX_INSTANCES) {
        DMA_Adp_DEBUG("DMA instance full! Max %d\r\n", DMA_MAX_INSTANCES);
        return 0xFF;
    }

    /* 检查通道是否已存在 */
    for (uint8_t i = 0; i < s_u8DmaInstanceCount; i++) {
        if (s_astcDmaInstances[i].stcConfig.u8DmaUnit == pstcConfig->u8DmaUnit &&
            s_astcDmaInstances[i].stcConfig.u8Channel == pstcConfig->u8Channel) {
            DMA_Adp_DEBUG("DMA%d CH%d already exists! Skip\r\n",
                   pstcConfig->u8DmaUnit, pstcConfig->u8Channel);
            return 0xFF;
        }
    }

    uint8_t u8Id = s_u8DmaInstanceCount;
    stc_dma_instance_t *pstcInst = &s_astcDmaInstances[u8Id];

    memset(pstcInst, 0, sizeof(stc_dma_instance_t));
    memcpy(&pstcInst->stcConfig, pstcConfig, sizeof(stc_dma_config_t));
    pstcInst->u8Id = u8Id;
    pstcInst->u8Initialized = 0;
    pstcInst->pu16Buffer = NULL;
    pstcInst->u16BufferSize = 0;

    s_u8DmaInstanceCount++;

    DMA_Adp_DEBUG("DMA instance created: ID=%d, DMA%d CH%d, Dir=%s, Width=%s, Block=%d\r\n",
           u8Id, pstcConfig->u8DmaUnit, pstcConfig->u8Channel,
           (pstcConfig->enDir == DMA_DIR_PERIPH_TO_MEM) ? "Periph->Mem" :
           (pstcConfig->enDir == DMA_DIR_MEM_TO_PERIPH) ? "Mem->Periph" : "Mem->Mem",
           (pstcConfig->u32DataWidth == DMA_DATAWIDTH_8BIT) ? "8bit" :
           (pstcConfig->u32DataWidth == DMA_DATAWIDTH_16BIT) ? "16bit" : "32bit",
           pstcConfig->u16BlockSize);

    return u8Id;
}

/**
 * @brief  初始化所有 DMA 实例
 */
void Dma_Init(void)
{
    if (s_bDmaInitialized) {
        DMA_Adp_DEBUG("DMA already initialized\r\n");
        return;
    }

    if (s_u8DmaInstanceCount == 0) {
        DMA_Adp_DEBUG("No DMA instance created! Call Dma_Create first\r\n");
        return;
    }

    /* 初始化中断映射表 */
    for (uint8_t i = 0; i < 2; i++) {
        for (uint8_t j = 0; j < DMA_MAX_CHANNELS; j++) {
            s_a8DmaIrqMap[i][j] = -1;
        }
    }

    /* 逐个配置 DMA 通道 */
    for (uint8_t i = 0; i < s_u8DmaInstanceCount; i++) {
        stc_dma_instance_t *pstcInst = &s_astcDmaInstances[i];
        CM_DMA_TypeDef *pDma = Dma_GetUnitPtr(pstcInst->stcConfig.u8DmaUnit);
        uint8_t u8Ch = pstcInst->stcConfig.u8Channel;

        /* 使能 DMA 时钟 */
        FCG_Fcg0PeriphClockCmd(Dma_GetPeriphClk(pstcInst->stcConfig.u8DmaUnit), ENABLE);

        /* 如果是外设到内存模式，分配缓冲区 */
        if (pstcInst->stcConfig.enDir == DMA_DIR_PERIPH_TO_MEM) {
            pstcInst->u16BufferSize = pstcInst->stcConfig.u16BlockSize;
            Dma_AllocateBuffer(pstcInst);

            /* 如果缓冲区分配成功，更新目标地址为缓冲区地址 */
            if (pstcInst->pu16Buffer != NULL) {
                pstcInst->stcConfig.u32DestAddr = (uint32_t)pstcInst->pu16Buffer;
            }
        }

        /* 配置 DMA 基本参数 */
        stc_dma_init_t stcDmaInit;
        (void)DMA_StructInit(&stcDmaInit);
        stcDmaInit.u32IntEn       = pstcInst->stcConfig.u8EnableInt ? DMA_INT_ENABLE : DMA_INT_DISABLE;
        stcDmaInit.u32SrcAddr     = pstcInst->stcConfig.u32SrcAddr;
        stcDmaInit.u32DestAddr    = pstcInst->stcConfig.u32DestAddr;
        stcDmaInit.u32DataWidth   = pstcInst->stcConfig.u32DataWidth;
        stcDmaInit.u32BlockSize   = pstcInst->stcConfig.u16BlockSize;
        stcDmaInit.u32TransCount  = pstcInst->stcConfig.u16TransCount;
        stcDmaInit.u32SrcAddrInc  = pstcInst->stcConfig.u32SrcAddrInc;
        stcDmaInit.u32DestAddrInc = pstcInst->stcConfig.u32DestAddrInc;

        (void)DMA_Init(pDma, u8Ch, &stcDmaInit);

        /* 如果是重复模式，配置重复参数 */
        if (pstcInst->stcConfig.enTransMode == DMA_TRANS_MODE_REPEAT) {
            stc_dma_repeat_init_t stcDmaRptInit;
            (void)DMA_RepeatStructInit(&stcDmaRptInit);
            stcDmaRptInit.u32Mode      = DMA_RPT_BOTH;
            stcDmaRptInit.u32SrcCount  = pstcInst->stcConfig.u16BlockSize;
            stcDmaRptInit.u32DestCount = pstcInst->stcConfig.u16BlockSize;
            (void)DMA_RepeatInit(pDma, u8Ch, &stcDmaRptInit);
        }

        /* 配置中断 */
        if (pstcInst->stcConfig.u8EnableInt) {
            s_a8DmaIrqMap[pstcInst->stcConfig.u8DmaUnit - 1][u8Ch] = (int8_t)i;
            Dma_IrqConfig(pstcInst);
        }

        DMA_Adp_DEBUG("DMA%d CH%d configured: SRC=0x%08X, DEST=0x%08X, Block=%d, Repeat=%s\r\n",
               pstcInst->stcConfig.u8DmaUnit, u8Ch,
               pstcInst->stcConfig.u32SrcAddr,
               pstcInst->stcConfig.u32DestAddr,
               pstcInst->stcConfig.u16BlockSize,
               (pstcInst->stcConfig.enTransMode == DMA_TRANS_MODE_REPEAT) ? "Yes" : "No");

        pstcInst->u8Initialized = 1;
    }

    s_bDmaInitialized = 1;
    DMA_Adp_DEBUG("DMA initialized with %d instance(s)\r\n", s_u8DmaInstanceCount);
}

/**
 * @brief  反初始化 DMA
 */
void Dma_DeInit(void)
{
    Dma_StopAll();

    for (uint8_t i = 0; i < s_u8DmaInstanceCount; i++) {
        stc_dma_instance_t *pstcInst = &s_astcDmaInstances[i];
        Dma_FreeBuffer(pstcInst);
        pstcInst->u8Initialized = 0;
    }

    s_u8DmaInstanceCount = 0;
    s_bDmaInitialized = 0;
    DMA_Adp_DEBUG("DMA deinitialized\r\n");
}

/*******************************************************************************
 * API functions - 控制
 ******************************************************************************/

void Dma_Start(uint8_t u8Ch)
{
    if (u8Ch >= s_u8DmaInstanceCount) return;

    stc_dma_instance_t *pstcInst = &s_astcDmaInstances[u8Ch];
    CM_DMA_TypeDef *pDma = Dma_GetUnitPtr(pstcInst->stcConfig.u8DmaUnit);

    DMA_ChCmd(pDma, pstcInst->stcConfig.u8Channel, ENABLE);
    DMA_Cmd(pDma, ENABLE);

    DMA_Adp_DEBUG("DMA%d CH%d started\r\n",
           pstcInst->stcConfig.u8DmaUnit, pstcInst->stcConfig.u8Channel);
}

void Dma_Stop(uint8_t u8Ch)
{
    if (u8Ch >= s_u8DmaInstanceCount) return;

    stc_dma_instance_t *pstcInst = &s_astcDmaInstances[u8Ch];
    CM_DMA_TypeDef *pDma = Dma_GetUnitPtr(pstcInst->stcConfig.u8DmaUnit);

    DMA_ChCmd(pDma, pstcInst->stcConfig.u8Channel, DISABLE);

    DMA_Adp_DEBUG("DMA%d CH%d stopped\r\n",
           pstcInst->stcConfig.u8DmaUnit, pstcInst->stcConfig.u8Channel);
}

void Dma_StartAll(void)
{
    for (uint8_t i = 0; i < s_u8DmaInstanceCount; i++) {
        Dma_Start(i);
    }
}

void Dma_StopAll(void)
{
    for (uint8_t i = 0; i < s_u8DmaInstanceCount; i++) {
        Dma_Stop(i);
    }
}

/*******************************************************************************
 * API functions - 运行时配置
 ******************************************************************************/

void Dma_SetSrcAddr(uint8_t u8Ch, uint32_t u32Addr)
{
    if (u8Ch >= s_u8DmaInstanceCount) return;
    stc_dma_instance_t *pstcInst = &s_astcDmaInstances[u8Ch];
    CM_DMA_TypeDef *pDma = Dma_GetUnitPtr(pstcInst->stcConfig.u8DmaUnit);
    DMA_SetSrcAddr(pDma, pstcInst->stcConfig.u8Channel, u32Addr);
    pstcInst->stcConfig.u32SrcAddr = u32Addr;
}

void Dma_SetDestAddr(uint8_t u8Ch, uint32_t u32Addr)
{
    if (u8Ch >= s_u8DmaInstanceCount) return;
    stc_dma_instance_t *pstcInst = &s_astcDmaInstances[u8Ch];
    CM_DMA_TypeDef *pDma = Dma_GetUnitPtr(pstcInst->stcConfig.u8DmaUnit);
    DMA_SetDestAddr(pDma, pstcInst->stcConfig.u8Channel, u32Addr);
    pstcInst->stcConfig.u32DestAddr = u32Addr;
}

void Dma_SetBlockSize(uint8_t u8Ch, uint16_t u16Size)
{
    if (u8Ch >= s_u8DmaInstanceCount) return;
    stc_dma_instance_t *pstcInst = &s_astcDmaInstances[u8Ch];
    CM_DMA_TypeDef *pDma = Dma_GetUnitPtr(pstcInst->stcConfig.u8DmaUnit);
    DMA_SetBlockSize(pDma, pstcInst->stcConfig.u8Channel, u16Size);
    pstcInst->stcConfig.u16BlockSize = u16Size;
}

void Dma_SetTransCount(uint8_t u8Ch, uint16_t u16Count)
{
    if (u8Ch >= s_u8DmaInstanceCount) return;
    stc_dma_instance_t *pstcInst = &s_astcDmaInstances[u8Ch];
    CM_DMA_TypeDef *pDma = Dma_GetUnitPtr(pstcInst->stcConfig.u8DmaUnit);
    DMA_SetTransCount(pDma, pstcInst->stcConfig.u8Channel, u16Count);
    pstcInst->stcConfig.u16TransCount = u16Count;
}

void Dma_SetDataWidth(uint8_t u8Ch, uint32_t u32DataWidth)
{
    if (u8Ch >= s_u8DmaInstanceCount) return;
    stc_dma_instance_t *pstcInst = &s_astcDmaInstances[u8Ch];
    CM_DMA_TypeDef *pDma = Dma_GetUnitPtr(pstcInst->stcConfig.u8DmaUnit);
    DMA_SetDataWidth(pDma, pstcInst->stcConfig.u8Channel, u32DataWidth);
    pstcInst->stcConfig.u32DataWidth = u32DataWidth;
}

/*******************************************************************************
 * API functions - 数据获取
 ******************************************************************************/

uint16_t* Dma_GetBuffer(uint8_t u8Ch)
{
    if (u8Ch >= s_u8DmaInstanceCount) return NULL;
    return s_astcDmaInstances[u8Ch].pu16Buffer;
}

uint16_t Dma_GetLatestValue(uint8_t u8Ch)
{
    if (u8Ch >= s_u8DmaInstanceCount) return 0;
    stc_dma_instance_t *pstcInst = &s_astcDmaInstances[u8Ch];
    if (pstcInst->pu16Buffer != NULL && pstcInst->u16BufferSize > 0) {
        return pstcInst->pu16Buffer[pstcInst->u16BufferSize - 1];
    }
    return 0;
}

uint16_t Dma_GetAverageValue(uint8_t u8Ch)
{
    if (u8Ch >= s_u8DmaInstanceCount) return 0;
    stc_dma_instance_t *pstcInst = &s_astcDmaInstances[u8Ch];
    if (pstcInst->pu16Buffer != NULL && pstcInst->u16BufferSize > 0) {
        uint32_t u32Sum = 0;
        for (uint16_t j = 0; j < pstcInst->u16BufferSize; j++) {
            u32Sum += pstcInst->pu16Buffer[j];
        }
        return (uint16_t)(u32Sum / pstcInst->u16BufferSize);
    }
    return 0;
}

uint32_t Dma_GetTransferCount(uint8_t u8Ch)
{
    if (u8Ch >= s_u8DmaInstanceCount) return 0;
    return s_astcDmaInstances[u8Ch].u32TransferCount;
}

uint8_t Dma_IsDataUpdated(uint8_t u8Ch)
{
    if (u8Ch >= s_u8DmaInstanceCount) return 0;
    return s_astcDmaInstances[u8Ch].u8DataUpdated;
}

void Dma_ClearDataUpdated(uint8_t u8Ch)
{
    if (u8Ch >= s_u8DmaInstanceCount) return;
    s_astcDmaInstances[u8Ch].u8DataUpdated = 0;
}

/*******************************************************************************
 * API functions - 状态查询
 ******************************************************************************/

/**
 * @brief  检查通道是否正在传输
 */
uint8_t Dma_IsChannelBusy(uint8_t u8Ch)
{
    if (u8Ch >= s_u8DmaInstanceCount) return 0;
    stc_dma_instance_t *pstcInst = &s_astcDmaInstances[u8Ch];
    CM_DMA_TypeDef *pDma = Dma_GetUnitPtr(pstcInst->stcConfig.u8DmaUnit);

    uint32_t u32Status = 0;
    switch (pstcInst->stcConfig.u8Channel) {
        case 0: u32Status = DMA_STAT_TRANS_CH0; break;
        case 1: u32Status = DMA_STAT_TRANS_CH1; break;
        case 2: u32Status = DMA_STAT_TRANS_CH2; break;
        case 3: u32Status = DMA_STAT_TRANS_CH3; break;
        default: return 0;
    }

    return (DMA_GetTransStatus(pDma, u32Status) == SET) ? 1 : 0;
}

/**
 * @brief  检查传输是否完成（TC 标志）
 */
uint8_t Dma_IsTransferComplete(uint8_t u8Ch)
{
    if (u8Ch >= s_u8DmaInstanceCount) return 0;
    stc_dma_instance_t *pstcInst = &s_astcDmaInstances[u8Ch];
    CM_DMA_TypeDef *pDma = Dma_GetUnitPtr(pstcInst->stcConfig.u8DmaUnit);

    uint32_t u32Flag = 0;
    switch (pstcInst->stcConfig.u8Channel) {
        case 0: u32Flag = DMA_FLAG_TC_CH0; break;
        case 1: u32Flag = DMA_FLAG_TC_CH1; break;
        case 2: u32Flag = DMA_FLAG_TC_CH2; break;
        case 3: u32Flag = DMA_FLAG_TC_CH3; break;
        default: return 0;
    }

    return (DMA_GetTransCompleteStatus(pDma, u32Flag) == SET) ? 1 : 0;
}

/**
 * @brief  检查块传输是否完成（BTC 标志）
 */
uint8_t Dma_IsBlockComplete(uint8_t u8Ch)
{
    if (u8Ch >= s_u8DmaInstanceCount) return 0;
    stc_dma_instance_t *pstcInst = &s_astcDmaInstances[u8Ch];
    CM_DMA_TypeDef *pDma = Dma_GetUnitPtr(pstcInst->stcConfig.u8DmaUnit);

    uint32_t u32Flag = 0;
    switch (pstcInst->stcConfig.u8Channel) {
        case 0: u32Flag = DMA_FLAG_BTC_CH0; break;
        case 1: u32Flag = DMA_FLAG_BTC_CH1; break;
        case 2: u32Flag = DMA_FLAG_BTC_CH2; break;
        case 3: u32Flag = DMA_FLAG_BTC_CH3; break;
        default: return 0;
    }

    return (DMA_GetTransCompleteStatus(pDma, u32Flag) == SET) ? 1 : 0;
}

/**
 * @brief  检查是否有错误
 */
uint8_t Dma_IsError(uint8_t u8Ch)
{
    if (u8Ch >= s_u8DmaInstanceCount) return 0;
    stc_dma_instance_t *pstcInst = &s_astcDmaInstances[u8Ch];
    CM_DMA_TypeDef *pDma = Dma_GetUnitPtr(pstcInst->stcConfig.u8DmaUnit);

    uint32_t u32ReqErr = 0, u32TransErr = 0;
    switch (pstcInst->stcConfig.u8Channel) {
        case 0:
            u32ReqErr = DMA_FLAG_REQ_ERR_CH0;
            u32TransErr = DMA_FLAG_TRANS_ERR_CH0;
            break;
        case 1:
            u32ReqErr = DMA_FLAG_REQ_ERR_CH1;
            u32TransErr = DMA_FLAG_TRANS_ERR_CH1;
            break;
        case 2:
            u32ReqErr = DMA_FLAG_REQ_ERR_CH2;
            u32TransErr = DMA_FLAG_TRANS_ERR_CH2;
            break;
        case 3:
            u32ReqErr = DMA_FLAG_REQ_ERR_CH3;
            u32TransErr = DMA_FLAG_TRANS_ERR_CH3;
            break;
        default: return 0;
    }

    return ((DMA_GetErrStatus(pDma, u32ReqErr) == SET) ||
            (DMA_GetErrStatus(pDma, u32TransErr) == SET)) ? 1 : 0;
}

/**
 * @brief  清除指定标志
 */
void Dma_ClearFlag(uint8_t u8Ch, uint32_t u32Flag)
{
    if (u8Ch >= s_u8DmaInstanceCount) return;
    stc_dma_instance_t *pstcInst = &s_astcDmaInstances[u8Ch];
    CM_DMA_TypeDef *pDma = Dma_GetUnitPtr(pstcInst->stcConfig.u8DmaUnit);

    if ((u32Flag == DMA_FLAG_REQ_ERR_CH0) || (u32Flag == DMA_FLAG_REQ_ERR_CH1) ||
        (u32Flag == DMA_FLAG_REQ_ERR_CH2) || (u32Flag == DMA_FLAG_REQ_ERR_CH3) ||
        (u32Flag == DMA_FLAG_TRANS_ERR_CH0) || (u32Flag == DMA_FLAG_TRANS_ERR_CH1) ||
        (u32Flag == DMA_FLAG_TRANS_ERR_CH2) || (u32Flag == DMA_FLAG_TRANS_ERR_CH3)) {
        DMA_ClearErrStatus(pDma, u32Flag);
    } else {
        DMA_ClearTransCompleteStatus(pDma, u32Flag);
    }
}

/*******************************************************************************
 * API functions - 地址/计数读取
 ******************************************************************************/

uint32_t Dma_GetSrcAddr(uint8_t u8Ch)
{
    if (u8Ch >= s_u8DmaInstanceCount) return 0;
    stc_dma_instance_t *pstcInst = &s_astcDmaInstances[u8Ch];
    CM_DMA_TypeDef *pDma = Dma_GetUnitPtr(pstcInst->stcConfig.u8DmaUnit);
    return DMA_GetSrcAddr(pDma, pstcInst->stcConfig.u8Channel);
}

uint32_t Dma_GetDestAddr(uint8_t u8Ch)
{
    if (u8Ch >= s_u8DmaInstanceCount) return 0;
    stc_dma_instance_t *pstcInst = &s_astcDmaInstances[u8Ch];
    CM_DMA_TypeDef *pDma = Dma_GetUnitPtr(pstcInst->stcConfig.u8DmaUnit);
    return DMA_GetDestAddr(pDma, pstcInst->stcConfig.u8Channel);
}

uint32_t Dma_GetBlockSize(uint8_t u8Ch)
{
    if (u8Ch >= s_u8DmaInstanceCount) return 0;
    stc_dma_instance_t *pstcInst = &s_astcDmaInstances[u8Ch];
    CM_DMA_TypeDef *pDma = Dma_GetUnitPtr(pstcInst->stcConfig.u8DmaUnit);
    return DMA_GetBlockSize(pDma, pstcInst->stcConfig.u8Channel);
}

/*******************************************************************************
 * API functions - 查找
 ******************************************************************************/

int8_t Dma_FindIdByChannel(uint8_t u8DmaUnit, uint8_t u8Ch)
{
    for (uint8_t i = 0; i < s_u8DmaInstanceCount; i++) {
        if (s_astcDmaInstances[i].stcConfig.u8DmaUnit == u8DmaUnit &&
            s_astcDmaInstances[i].stcConfig.u8Channel == u8Ch) {
            return (int8_t)i;
        }
    }
    return -1;
}

/*******************************************************************************
 * API functions - 调试
 ******************************************************************************/

void Dma_PrintDebugInfo(void)
{
    MAIN_D("=== DMA Driver Debug Info ===\r\n");
    MAIN_D("Initialized: %s\r\n", s_bDmaInitialized ? "Yes" : "No");
    MAIN_D("Instance count: %d\r\n", s_u8DmaInstanceCount);

    for (uint8_t i = 0; i < s_u8DmaInstanceCount; i++) {
        stc_dma_instance_t *pstcInst = &s_astcDmaInstances[i];
        MAIN_D("  ID%d: DMA%d CH%d, Dir=%s, Block=%d, Buffer=0x%08X, Count=%lu\r\n",
               pstcInst->u8Id,
               pstcInst->stcConfig.u8DmaUnit,
               pstcInst->stcConfig.u8Channel,
               (pstcInst->stcConfig.enDir == DMA_DIR_PERIPH_TO_MEM) ? "Periph->Mem" :
               (pstcInst->stcConfig.enDir == DMA_DIR_MEM_TO_PERIPH) ? "Mem->Periph" : "Mem->Mem",
               pstcInst->stcConfig.u16BlockSize,
               (uint32_t)pstcInst->pu16Buffer,
               pstcInst->u32TransferCount);
    }
}

/*******************************************************************************
 * EOF
 ******************************************************************************/

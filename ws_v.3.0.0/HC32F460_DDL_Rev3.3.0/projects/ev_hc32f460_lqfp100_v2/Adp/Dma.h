/**
 *******************************************************************************
 * @file  Dma.h
 * @brief DMA Driver for HC32F460 - Generic framework with multiple channels
 *        直接使用 hc32_ll_dma.h 中的宏定义，不重复定义
 *******************************************************************************
 */

#ifndef __DMA_H__
#define __DMA_H__

#include "main.h"
#include "hc32_ll_dma.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/

/* 调试开关 */
#define DMA_RTT 1

#ifdef DEBUG_DMA_Adp
    #define DMA_Adp_DEBUG(fmt, ...)    MAIN_D("[DMA_DEBUG] " fmt, ##__VA_ARGS__)
#else
    #define DMA_Adp_DEBUG(fmt, ...)    ((void)0)
#endif

/* ============================================================================
 * DMA 单元选择
 * ==========================================================================*/
#define DMA_UNIT_1                      (1U)                    /* DMA 单元 1 */
#define DMA_UNIT_2                      (2U)                    /* DMA 单元 2 */

/* ============================================================================
 * DMA 最大通道数
 * ==========================================================================*/
#define DMA_MAX_CHANNELS                (4U)                    /* 每个 DMA 单元 4 通道 */
#define DMA_MAX_INSTANCES               (8U)                    /* DMA1 + DMA2 共 8 通道 */

/* ============================================================================
 * 默认中断优先级
 * ==========================================================================*/
#define DMA_DEFAULT_INT_PRIO            (DDL_IRQ_PRIO_03)

/* ============================================================================
 * 事件源定义（DMA 请求触发源）
 * 参考 HC32F460 数据手册，DMA 请求可以由多种外设事件触发
 * 这些事件源通过 AOS 路由到 DMA 目标
 * ==========================================================================*/

/* ---- ADC 事件 ---- */
#define DMA_EVT_ADC1_EOCA               (INT_SRC_ADC1_EOCA)     /* ADC1 序列A转换完成 */
#define DMA_EVT_ADC1_EOCB               (INT_SRC_ADC1_EOCB)     /* ADC1 序列B转换完成 */
#define DMA_EVT_ADC2_EOCA               (INT_SRC_ADC2_EOCA)     /* ADC2 序列A转换完成 */
#define DMA_EVT_ADC2_EOCB               (INT_SRC_ADC2_EOCB)     /* ADC2 序列B转换完成 */

/* ---- Timer0 事件 ---- */
#define DMA_EVT_TMR0_1_CMP_A            (INT_SRC_TMR0_1_CMP_A)  /* TMR0_1 比较A */
#define DMA_EVT_TMR0_1_CMP_B            (INT_SRC_TMR0_1_CMP_B)  /* TMR0_1 比较B */
#define DMA_EVT_TMR0_2_CMP_A            (INT_SRC_TMR0_2_CMP_A)  /* TMR0_2 比较A */
#define DMA_EVT_TMR0_2_CMP_B            (INT_SRC_TMR0_2_CMP_B)  /* TMR0_2 比较B */

/* ---- Timer4 事件 ---- */
#define DMA_EVT_TMR4_1_CMP_A            (INT_SRC_TMR4_1_CMPA)   /* TMR4_1 比较A */
#define DMA_EVT_TMR4_1_CMP_B            (INT_SRC_TMR4_1_CMPB)   /* TMR4_1 比较B */
#define DMA_EVT_TMR4_1_CMP_C            (INT_SRC_TMR4_1_CMPC)   /* TMR4_1 比较C */
#define DMA_EVT_TMR4_1_CMP_D            (INT_SRC_TMR4_1_CMPD)   /* TMR4_1 比较D */
#define DMA_EVT_TMR4_1_OVF              (INT_SRC_TMR4_1_OVF)    /* TMR4_1 溢出 */
#define DMA_EVT_TMR4_2_CMP_A            (INT_SRC_TMR4_2_CMPA)   /* TMR4_2 比较A */
#define DMA_EVT_TMR4_2_CMP_B            (INT_SRC_TMR4_2_CMPB)   /* TMR4_2 比较B */
#define DMA_EVT_TMR4_2_CMP_C            (INT_SRC_TMR4_2_CMPC)   /* TMR4_2 比较C */
#define DMA_EVT_TMR4_2_CMP_D            (INT_SRC_TMR4_2_CMPD)   /* TMR4_2 比较D */
#define DMA_EVT_TMR4_2_OVF              (INT_SRC_TMR4_2_OVF)    /* TMR4_2 溢出 */

/* ---- Timer6 事件 ---- */
#define DMA_EVT_TMR6_1_CMP_A            (INT_SRC_TMR6_1_CMPA)   /* TMR6_1 比较A */
#define DMA_EVT_TMR6_1_CMP_B            (INT_SRC_TMR6_1_CMPB)   /* TMR6_1 比较B */
#define DMA_EVT_TMR6_1_CMP_C            (INT_SRC_TMR6_1_CMPC)   /* TMR6_1 比较C */
#define DMA_EVT_TMR6_1_CMP_D            (INT_SRC_TMR6_1_CMPD)   /* TMR6_1 比较D */
#define DMA_EVT_TMR6_1_OVF              (INT_SRC_TMR6_1_OVF)    /* TMR6_1 溢出 */
#define DMA_EVT_TMR6_2_CMP_A            (INT_SRC_TMR6_2_CMPA)   /* TMR6_2 比较A */
#define DMA_EVT_TMR6_2_CMP_B            (INT_SRC_TMR6_2_CMPB)   /* TMR6_2 比较B */
#define DMA_EVT_TMR6_2_CMP_C            (INT_SRC_TMR6_2_CMPC)   /* TMR6_2 比较C */
#define DMA_EVT_TMR6_2_CMP_D            (INT_SRC_TMR6_2_CMPD)   /* TMR6_2 比较D */
#define DMA_EVT_TMR6_2_OVF              (INT_SRC_TMR6_2_OVF)    /* TMR6_2 溢出 */

/* ---- TimerA 事件 ---- */
#define DMA_EVT_TMRA_1_CMP_A            (INT_SRC_TMRA_1_CMPA)   /* TMRA_1 比较A */
#define DMA_EVT_TMRA_1_CMP_B            (INT_SRC_TMRA_1_CMPB)   /* TMRA_1 比较B */
#define DMA_EVT_TMRA_1_CMP_C            (INT_SRC_TMRA_1_CMPC)   /* TMRA_1 比较C */
#define DMA_EVT_TMRA_1_CMP_D            (INT_SRC_TMRA_1_CMPD)   /* TMRA_1 比较D */
#define DMA_EVT_TMRA_1_OVF              (INT_SRC_TMRA_1_OVF)    /* TMRA_1 溢出 */
#define DMA_EVT_TMRA_2_CMP_A            (INT_SRC_TMRA_2_CMPA)   /* TMRA_2 比较A */
#define DMA_EVT_TMRA_2_CMP_B            (INT_SRC_TMRA_2_CMPB)   /* TMRA_2 比较B */
#define DMA_EVT_TMRA_2_CMP_C            (INT_SRC_TMRA_2_CMPC)   /* TMRA_2 比较C */
#define DMA_EVT_TMRA_2_CMP_D            (INT_SRC_TMRA_2_CMPD)   /* TMRA_2 比较D */
#define DMA_EVT_TMRA_2_OVF              (INT_SRC_TMRA_2_OVF)    /* TMRA_2 溢出 */

/* ---- USART 事件 ---- */
#define DMA_EVT_USART1_RX               (INT_SRC_USART1_RI)     /* USART1 接收 */
#define DMA_EVT_USART1_TX               (INT_SRC_USART1_TI)     /* USART1 发送 */
#define DMA_EVT_USART2_RX               (INT_SRC_USART2_RX)     /* USART2 接收 */
#define DMA_EVT_USART2_TX               (INT_SRC_USART2_TX)     /* USART2 发送 */
#define DMA_EVT_USART3_RX               (INT_SRC_USART3_RX)     /* USART3 接收 */
#define DMA_EVT_USART3_TX               (INT_SRC_USART3_TX)     /* USART3 发送 */
#define DMA_EVT_USART4_RX               (INT_SRC_USART4_RX)     /* USART4 接收 */
#define DMA_EVT_USART4_TX               (INT_SRC_USART4_TX)     /* USART4 发送 */

/* ---- SPI 事件 ---- */
#define DMA_EVT_SPI1_RX                 (INT_SRC_SPI1_RX)       /* SPI1 接收 */
#define DMA_EVT_SPI1_TX                 (INT_SRC_SPI1_TX)       /* SPI1 发送 */
#define DMA_EVT_SPI2_RX                 (INT_SRC_SPI2_RX)       /* SPI2 接收 */
#define DMA_EVT_SPI2_TX                 (INT_SRC_SPI2_TX)       /* SPI2 发送 */
#define DMA_EVT_SPI3_RX                 (INT_SRC_SPI3_RX)       /* SPI3 接收 */
#define DMA_EVT_SPI3_TX                 (INT_SRC_SPI3_TX)       /* SPI3 发送 */
#define DMA_EVT_SPI4_RX                 (INT_SRC_SPI4_RX)       /* SPI4 接收 */
#define DMA_EVT_SPI4_TX                 (INT_SRC_SPI4_TX)       /* SPI4 发送 */

/* ---- I2C 事件 ---- */
#define DMA_EVT_I2C1_RX                 (INT_SRC_I2C1_RX)       /* I2C1 接收 */
#define DMA_EVT_I2C1_TX                 (INT_SRC_I2C1_TX)       /* I2C1 发送 */
#define DMA_EVT_I2C2_RX                 (INT_SRC_I2C2_RX)       /* I2C2 接收 */
#define DMA_EVT_I2C2_TX                 (INT_SRC_I2C2_TX)       /* I2C2 发送 */
#define DMA_EVT_I2C3_RX                 (INT_SRC_I2C3_RX)       /* I2C3 接收 */
#define DMA_EVT_I2C3_TX                 (INT_SRC_I2C3_TX)       /* I2C3 发送 */

/* ---- 外部中断事件 ---- */
#define DMA_EVT_EIRQ0                   (INT_SRC_PORT_EIRQ0)    /* 外部中断 0 */
#define DMA_EVT_EIRQ1                   (INT_SRC_PORT_EIRQ1)    /* 外部中断 1 */
#define DMA_EVT_EIRQ2                   (INT_SRC_PORT_EIRQ2)    /* 外部中断 2 */
#define DMA_EVT_EIRQ3                   (INT_SRC_PORT_EIRQ3)    /* 外部中断 3 */
#define DMA_EVT_EIRQ4                   (INT_SRC_PORT_EIRQ4)    /* 外部中断 4 */
#define DMA_EVT_EIRQ5                   (INT_SRC_PORT_EIRQ5)    /* 外部中断 5 */
#define DMA_EVT_EIRQ6                   (INT_SRC_PORT_EIRQ6)    /* 外部中断 6 */
#define DMA_EVT_EIRQ7                   (INT_SRC_PORT_EIRQ7)    /* 外部中断 7 */
#define DMA_EVT_EIRQ8                   (INT_SRC_PORT_EIRQ8)    /* 外部中断 8 */
#define DMA_EVT_EIRQ9                   (INT_SRC_PORT_EIRQ9)    /* 外部中断 9 */
#define DMA_EVT_EIRQ10                  (INT_SRC_PORT_EIRQ10)   /* 外部中断 10 */
#define DMA_EVT_EIRQ11                  (INT_SRC_PORT_EIRQ11)   /* 外部中断 11 */
#define DMA_EVT_EIRQ12                  (INT_SRC_PORT_EIRQ12)   /* 外部中断 12 */
#define DMA_EVT_EIRQ13                  (INT_SRC_PORT_EIRQ13)   /* 外部中断 13 */
#define DMA_EVT_EIRQ14                  (INT_SRC_PORT_EIRQ14)   /* 外部中断 14 */
#define DMA_EVT_EIRQ15                  (INT_SRC_PORT_EIRQ15)   /* 外部中断 15 */

/* ---- GPIO 事件 ---- */
#define DMA_EVT_GPIO_PA                 (INT_SRC_GPIO_PA)       /* GPIO PA 端口 */
#define DMA_EVT_GPIO_PB                 (INT_SRC_GPIO_PB)       /* GPIO PB 端口 */
#define DMA_EVT_GPIO_PC                 (INT_SRC_GPIO_PC)       /* GPIO PC 端口 */
#define DMA_EVT_GPIO_PD                 (INT_SRC_GPIO_PD)       /* GPIO PD 端口 */
#define DMA_EVT_GPIO_PE                 (INT_SRC_GPIO_PE)       /* GPIO PE 端口 */

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/**
 * @brief  DMA 传输方向
 */
typedef enum {
    DMA_DIR_MEM_TO_MEM = 0,     /* 内存到内存 */
    DMA_DIR_PERIPH_TO_MEM = 1,  /* 外设到内存 */
    DMA_DIR_MEM_TO_PERIPH = 2,  /* 内存到外设 */
} en_dma_dir_t;

/**
 * @brief  DMA 传输模式
 */
typedef enum {
    DMA_TRANS_MODE_SINGLE = 0,  /* 单次传输 */
    DMA_TRANS_MODE_REPEAT = 1,  /* 循环传输（重复模式） */
} en_dma_trans_mode_t;

/**
 * @brief  DMA 配置结构体
 */
typedef struct {
    uint8_t         u8DmaUnit;          /* DMA 单元：DMA_UNIT_1 或 DMA_UNIT_2 */
    uint8_t         u8Channel;          /* 通道号 (0-3) */
    en_dma_dir_t    enDir;              /* 传输方向 */
    en_dma_trans_mode_t enTransMode;    /* 传输模式 */
    
    uint32_t        u32SrcAddr;         /* 源地址 */
    uint32_t        u32DestAddr;        /* 目标地址 */
    uint32_t        u32DataWidth;       /* 数据宽度 (DMA_DATAWIDTH_8BIT/16BIT/32BIT) */
    uint16_t        u16BlockSize;       /* 块大小（每次触发传输的数据量） */
    uint16_t        u16TransCount;      /* 传输计数（0=无限） */
    
    uint32_t        u32SrcAddrInc;      /* 源地址增量模式 */
    uint32_t        u32DestAddrInc;     /* 目标地址增量模式 */
    
    uint8_t         u8EnableInt;        /* 是否使能中断 */
    uint8_t         u8IntPriority;      /* 中断优先级 */
    
    void (*pfnCallback)(void);          /* 传输完成回调函数 */
} stc_dma_config_t;

/**
 * @brief  DMA 实例结构体
 */
typedef struct {
    uint8_t         u8Id;               /* 实例 ID */
    uint8_t         u8Initialized;      /* 初始化标志 */
    
    stc_dma_config_t stcConfig;         /* 配置 */
    
    uint16_t       *pu16Buffer;         /* 数据缓冲区指针（外设到内存模式使用） */
    uint16_t        u16BufferSize;      /* 缓冲区大小 */
    
    uint32_t        u32TransferCount;   /* 累计传输计数 */
    uint8_t         u8DataUpdated;      /* 数据更新标志 */
} stc_dma_instance_t;

/*******************************************************************************
 * Global function prototypes
 ******************************************************************************/

/* ============================================================================
 * DMA 实例管理
 * ==========================================================================*/
uint8_t Dma_Create(stc_dma_config_t *pstcConfig);
void Dma_Init(void);
void Dma_DeInit(void);

/* ============================================================================
 * DMA 控制
 * ==========================================================================*/
void Dma_Start(uint8_t u8Ch);
void Dma_Stop(uint8_t u8Ch);
void Dma_StartAll(void);
void Dma_StopAll(void);

/* ============================================================================
 * DMA 运行时配置（可在传输过程中动态修改）
 * ==========================================================================*/
void Dma_SetSrcAddr(uint8_t u8Ch, uint32_t u32Addr);
void Dma_SetDestAddr(uint8_t u8Ch, uint32_t u32Addr);
void Dma_SetBlockSize(uint8_t u8Ch, uint16_t u16Size);
void Dma_SetTransCount(uint8_t u8Ch, uint16_t u16Count);
void Dma_SetDataWidth(uint8_t u8Ch, uint32_t u32DataWidth);

/* ============================================================================
 * DMA 数据获取
 * ==========================================================================*/
uint16_t* Dma_GetBuffer(uint8_t u8Ch);
uint16_t  Dma_GetLatestValue(uint8_t u8Ch);
uint16_t  Dma_GetAverageValue(uint8_t u8Ch);
uint32_t  Dma_GetTransferCount(uint8_t u8Ch);
uint8_t   Dma_IsDataUpdated(uint8_t u8Ch);
void      Dma_ClearDataUpdated(uint8_t u8Ch);

/* ============================================================================
 * DMA 状态查询
 * ==========================================================================*/
uint8_t   Dma_IsChannelBusy(uint8_t u8Ch);           /* 通道是否正在传输 */
uint8_t   Dma_IsTransferComplete(uint8_t u8Ch);      /* 传输是否完成 */
uint8_t   Dma_IsBlockComplete(uint8_t u8Ch);         /* 块传输是否完成 */
uint8_t   Dma_IsError(uint8_t u8Ch);                 /* 是否有错误 */
void      Dma_ClearFlag(uint8_t u8Ch, uint32_t u32Flag);  /* 清除标志 */

/* ============================================================================
 * DMA 地址/计数读取
 * ==========================================================================*/
uint32_t  Dma_GetSrcAddr(uint8_t u8Ch);
uint32_t  Dma_GetDestAddr(uint8_t u8Ch);
uint32_t  Dma_GetBlockSize(uint8_t u8Ch);

/* ============================================================================
 * 查找
 * ==========================================================================*/
int8_t Dma_FindIdByChannel(uint8_t u8DmaUnit, uint8_t u8Ch);

/* ============================================================================
 * 调试
 * ==========================================================================*/
void Dma_PrintDebugInfo(void);

#ifdef __cplusplus
}
#endif

#endif /* __DMA_H__ */

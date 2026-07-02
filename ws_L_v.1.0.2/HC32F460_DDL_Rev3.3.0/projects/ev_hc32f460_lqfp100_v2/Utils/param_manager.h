#ifndef PARAM_MANAGER_H_
#define PARAM_MANAGER_H_

#include "main.h"
#include "hc32f46x_flash.h"
#include <stdint.h>
#include <stdbool.h>
#include "Adapter.h"

#include "rtt_manager.h"

/*=============================================================================
 * 调试宏定义（旧名称兼容）
 * 开关在 rtt_manager.h 中统一管理：PARAM_DEBUG
 *============================================================================*/
#ifdef PARAM_DEBUG
    #define PARAM_DEBUG(fmt, ...)    MAIN_D("[PARAM] " fmt, ##__VA_ARGS__)
#else
    #define PARAM_DEBUG(fmt, ...)    ((void)0)
#endif

/*=============================================================================
 * 魔数定义
 *============================================================================*/
#define PARAM_MAGIC_HEAD    0x55AA55AA  /* 参数存储头部魔数 */
#define PARAM_MAGIC_TAIL    0xAA44AA44  /* 参数存储尾部魔数 */

/*=============================================================================
 * 返回值定义
 *============================================================================*/
#define PARAM_OK            0    /* 操作成功 */
#define PARAM_ERR           -1   /* 操作失败 */
#define PARAM_ERR_INVD_PARAM -3  /* 无效参数 */
#define PARAM_ERR_NOT_RDY   -5   /* 未就绪 */

/*=============================================================================
 * 调试监控结构体
 *============================================================================*/
typedef struct {
    uint16_t    curr_sec;      /* 当前正在用的扇区编号 (62-56) */
    uint32_t    curr_addr;     /* 当前有效数据的物理地址 (相对Flash基址) */
    uint32_t    next_addr;     /* 下次保存将写入的地址 */
    uint32_t    save_count;    /* 本次上电后的成功保存次数 */
    int32_t     last_res;      /* 最后一次操作结果 (使用PARAM_OK/PARAM_ERR) */
} Param_Debug_t;

/* 全局变量声明 */
extern Param_Debug_t    g_ParamDebug;   /* 参数调试信息 */

/*=============================================================================
 * 参数管理器初始化配置结构体
 *============================================================================*/
typedef struct {
    void       *pParamBuf;     /* 参数缓冲区指针（指向全局参数结构体） */
    uint32_t    paramSize;     /* 参数结构体大小（字节） */
    uint32_t    magicHead;     /* 头部魔数 */
    uint32_t    magicTail;     /* 尾部魔数 */
    uint32_t    checksumOffset;/* checksum 字段在结构体中的字节偏移 */
    uint32_t    seqOffset;     /* sequence_id 字段在结构体中的字节偏移 */
    uint32_t    eraseCntOffset;/* erase_count 字段在结构体中的字节偏移 */
} Param_Config_t;

/*=============================================================================
 * 函数声明
 *============================================================================*/

/**
 * @brief  参数管理器初始化
 * @param  pConfig  初始化配置（参数缓冲区指针、大小、魔数等）
 * @return PARAM_OK 成功，PARAM_ERR 失败
 * @note   从 Flash 中扫描有效参数块并加载到 pConfig->pParamBuf
 *         如果未找到有效块，则调用 pSetDefaults 设置默认值并写入 Flash
 */
int32_t Param_Init(const Param_Config_t *pConfig, void (*pSetDefaults)(void));

/**
 * @brief  参数保存到 Flash
 * @param  pConfig  初始化配置
 * @return PARAM_OK 成功，PARAM_ERR 失败
 */
int32_t Param_Save(const Param_Config_t *pConfig);

/**
 * @brief  调试功能：擦除所有参数扇区
 * @param  pConfig  初始化配置
 */
void Param_Debug_EraseAll(const Param_Config_t *pConfig, void (*pSetDefaults)(void));

/**
 * @brief  调试用：导出擦除函数
 */
int32_t Param_EraseSector(uint32_t address);

#endif /* PARAM_MANAGER_H_ */

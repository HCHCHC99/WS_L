#include "hc32f46x_flash.h"
#include "main.h"
#include "stdint.h"
#include "rtt_log.h"

// 测试结果变量
volatile uint32_t flash_test_result = 0;

// 将旧状态映射到新状态
static HC32FLASH_STATUS MapEFMStatusToHC32(int32_t efmStatus)
{
    switch(efmStatus)
    {
        case LL_OK:
            return HC32FLASH_OK;
        case LL_ERR_NOT_RDY:
            return HC32FLASH_BUSY;
        case LL_ERR_TIMEOUT:
            return HC32FLASH_BUSY;
        default:
        {
            // 检查具体错误标志
            if (SET == EFM_GetStatus(EFM_FLAG_PEPRTERR))
                return HC32FLASH_WPRERR;
            if (SET == EFM_GetStatus(EFM_FLAG_PGSZERR))
                return HC32FLASH_PGAERR;
            if (SET == EFM_GetStatus(EFM_FLAG_PEWERR))
                return HC32FLASH_PEWERR;
            if (SET == EFM_GetStatus(EFM_FLAG_COLERR))
                return HC32FLASH_COLERR;
            if (SET == EFM_GetStatus(EFM_FLAG_PGMISMTCH))
                return HC32FLASH_PGMISMTCH;
            return HC32FLASH_OK;
        }
    }
}

// 获取Flash状态（带调试输出）
HC32FLASH_STATUS HC32FLASH_GetStatus(void)
{
    HC32FLASH_STATUS status = HC32FLASH_OK;
    uint32_t fsr_value = 0;
    
    // 读取FSR寄存器值用于调试
    fsr_value = CM_EFM->FSR;
    
    if (RESET == EFM_GetStatus(EFM_FLAG_RDY))
    {
        status = HC32FLASH_BUSY;
        FLASH_DEBUG("FLASH status: BUSY, FSR=0x%08lX\r\n", fsr_value);
    }
    else if (SET == EFM_GetStatus(EFM_FLAG_PEPRTERR))
    {
        status = HC32FLASH_WPRERR;
        FLASH_DEBUG("FLASH status: WPRERR, FSR=0x%08lX\r\n", fsr_value);
    }
    else if (SET == EFM_GetStatus(EFM_FLAG_PGSZERR))
    {
        status = HC32FLASH_PGAERR;
        FLASH_DEBUG("FLASH status: PGAERR, FSR=0x%08lX\r\n", fsr_value);
    }
    else if (SET == EFM_GetStatus(EFM_FLAG_PEWERR))
    {
        status = HC32FLASH_PEWERR;
        FLASH_DEBUG("FLASH status: PEWERR, FSR=0x%08lX\r\n", fsr_value);
    }
    else if (SET == EFM_GetStatus(EFM_FLAG_COLERR))
    {
        status = HC32FLASH_COLERR;
        FLASH_DEBUG("FLASH status: COLERR, FSR=0x%08lX\r\n", fsr_value);
    }
    else if (SET == EFM_GetStatus(EFM_FLAG_PGMISMTCH))
    {
        status = HC32FLASH_PGMISMTCH;
        FLASH_DEBUG("FLASH status: PGMISMTCH, FSR=0x%08lX\r\n", fsr_value);
    }
    else
    {
        status = HC32FLASH_OK;
        FLASH_DEBUG("FLASH status: OK, FSR=0x%08lX\r\n", fsr_value);
    }
    
    return status;
}

// 解锁Flash寄存器
static void HC32FLASH_Unlock(void)
{
    FLASH_DEBUG("FLASH unlock start\r\n");
    EFM_REG_Unlock();       // 解除FLASH寄存器写保护
    EFM_FWMC_Cmd(ENABLE);   // 设定编程擦写模式许可
    EFM_SetBusStatus(EFM_BUS_HOLD); // 总线占用控制
    FLASH_DEBUG("FLASH unlock done\r\n");
}

// 锁定Flash寄存器
static void HC32FLASH_Lock(void)
{
    FLASH_DEBUG("FLASH lock start\r\n");
    EFM_FWMC_Cmd(DISABLE);  // 取消编程擦写模式许可
    EFM_REG_Lock();         // 重新上锁
    FLASH_DEBUG("FLASH lock done\r\n");
}

// 擦除扇区
__attribute__((section(".ramfunc")))
HC32FLASH_STATUS HC32FLASH_EraseSector(uint32_t u32Addr)
{
    int32_t efmRet;
    HC32FLASH_STATUS status;
    
    FLASH_DEBUG("Erase sector start, addr=0x%08lX\r\n", u32Addr);
    
    // 步骤1: 解除FLASH的寄存器写保护
    HC32FLASH_Unlock();
    
    // 步骤2: 清除所有错误标志
    EFM_ClearStatus(EFM_FLAG_ALL);
    
    // 步骤3: 扇区擦除
    efmRet = EFM_SectorErase(u32Addr);
    
    // 步骤4: 获取状态
    status = MapEFMStatusToHC32(efmRet);
    
    // 步骤5: 清除编程结束标志位
    EFM_ClearStatus(EFM_FLAG_ALL);
    
    // 步骤6: 退出擦除模式并上锁
    HC32FLASH_Lock();
    
    if (status == HC32FLASH_OK)
    {
        FLASH_DEBUG("Erase sector success, addr=0x%08lX\r\n", u32Addr);
    }
    else
    {
        FLASH_DEBUG("Erase sector failed, addr=0x%08lX, status=%d\r\n", u32Addr, status);
    }
    
    return status;
}

// 无回读校验的单字写入
HC32FLASH_STATUS HC32FLASH_WritedWord_NoCheck(uint32_t u32Addr, uint32_t data)
{
    int32_t efmRet;
    HC32FLASH_STATUS status;
    
    FLASH_DEBUG("Write word no check start, addr=0x%08lX, data=0x%08lX\r\n", u32Addr, data);
    
    HC32FLASH_Unlock();                    // 解锁
    EFM_ClearStatus(EFM_FLAG_ALL);         // 清除所有错误标记
    
    // 使用单编程模式写入数据
    efmRet = EFM_ProgramWord(u32Addr, data);
    
    status = MapEFMStatusToHC32(efmRet);
    
    EFM_ClearStatus(EFM_FLAG_ALL);         // 清除所有错误标记
    HC32FLASH_Lock();                      // 上锁
    
    if (status == HC32FLASH_OK)
    {
        FLASH_DEBUG("Write word no check success, addr=0x%08lX, data=0x%08lX\r\n", u32Addr, data);
    }
    else
    {
        FLASH_DEBUG("Write word no check failed, addr=0x%08lX, data=0x%08lX, status=%d\r\n", u32Addr, data, status);
    }
    
    return status;
}

// 带回读校验的单字写入
__attribute__((section(".ramfunc")))
HC32FLASH_STATUS HC32FLASH_WritedWord_Check(uint32_t u32Addr, uint32_t data)
{
    int32_t efmRet;
    HC32FLASH_STATUS status;
    uint32_t read_back_data;
    
    FLASH_DEBUG("Write word with check start, addr=0x%08lX, data=0x%08lX\r\n", u32Addr, data);
    
    HC32FLASH_Unlock();                    // 解锁
    EFM_ClearStatus(EFM_FLAG_ALL);         // 清除所有错误标记
    
    // 使用单编程回读模式（已经支持回读校验）
    efmRet = EFM_ProgramWordReadBack(u32Addr, data);
    
    status = MapEFMStatusToHC32(efmRet);
    
    // 如果状态显示成功，进行额外的读取验证
    if (status == HC32FLASH_OK)
    {
        // 立即读取刚写入的数据进行验证
        read_back_data = HC32FLASH_ReaddWord(u32Addr);
        
        // 如果读取的数据与写入的不一致，认为扇区可能损坏
        if (read_back_data != data)
        {
            status = HC32FLASH_PGMISMTCH;  // 标记为编程不匹配错误
            FLASH_DEBUG("Write word verify failed, addr=0x%08lX, expect=0x%08lX, actual=0x%08lX\r\n", 
                       u32Addr, data, read_back_data);
        }
        else
        {
            FLASH_DEBUG("Write word with check success, addr=0x%08lX, data=0x%08lX, verify=0x%08lX\r\n", 
                       u32Addr, data, read_back_data);
        }
    }
    else
    {
        FLASH_DEBUG("Write word with check failed, addr=0x%08lX, data=0x%08lX, status=%d\r\n", 
                   u32Addr, data, status);
    }
    
    EFM_ClearStatus(EFM_FLAG_ALL);         // 清除所有错误标记
    HC32FLASH_Lock();                      // 上锁
    
    return status;
}

// 读取一个字
__inline uint32_t HC32FLASH_ReaddWord(uint32_t u32Addr)
{
    uint32_t data = *(volatile uint32_t*)u32Addr;
    FLASH_DEBUG("Read word, addr=0x%08lX, data=0x%08lX\r\n", u32Addr, data);
    return data;
}

// Flash测试示例函数
void flash_test_example(void)
{
    HC32FLASH_STATUS status;
    uint32_t read_data;
    
    FLASH_DEBUG("Flash test example start\r\n");
    
    // 1. 擦除扇区
    status = HC32FLASH_EraseSector(TEST_FLASH_ADDR);
    if (status != HC32FLASH_OK)
    {
        flash_test_result = 1;
        FLASH_DEBUG("Erase failed, status=%d\r\n", status);
    }
    
    // 2. 写入单个字（无校验）
    status = HC32FLASH_WritedWord_NoCheck(TEST_FLASH_ADDR, 0x33333333);
    if (status != HC32FLASH_OK)
    {
        flash_test_result = 2;
        FLASH_DEBUG("Write no check failed, status=%d\r\n", status);
    }
    
    // 3. 带校验的写入
    status = HC32FLASH_WritedWord_Check(TEST_FLASH_ADDR + 16, 0x12345678);
    if (status != HC32FLASH_OK) 
    {
        flash_test_result = 2;
        FLASH_DEBUG("Write with check failed, status=%d\r\n", status);
    }
    
    // 4. 再次擦除扇区
    status = HC32FLASH_EraseSector(TEST_FLASH_ADDR);
    if (status != HC32FLASH_OK)
    {
        flash_test_result = 1;
        FLASH_DEBUG("Erase failed, status=%d\r\n", status);
    }
    
    // 5. 写入并验证
    status = HC32FLASH_WritedWord_NoCheck(TEST_FLASH_ADDR, 0x11111111);
    if (status != HC32FLASH_OK)
    {
        flash_test_result = 2;
        FLASH_DEBUG("Write no check failed, status=%d\r\n", status);
    }
    
    read_data = HC32FLASH_ReaddWord(TEST_FLASH_ADDR);
    if (read_data != 0x11111111)
    {
        flash_test_result = 3;
        FLASH_DEBUG("Verify failed, expect=0x11111111, actual=0x%08lX\r\n", read_data);
    }
    else
    {
        FLASH_DEBUG("Flash test example all success\r\n");
    }
}

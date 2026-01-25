#include "main.h"
#include "string.h"
#include "stdio.h"

uint32_t PageAddress = 0x08060000; // 起始页地址
void erase_flash(void)
{
    uint32_t page_error = 0;
    FLASH_EraseInitTypeDef erase_init_struct = {0};
    erase_init_struct.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init_struct.PageAddress = PageAddress; // 起始页地址
    erase_init_struct.NbPages = 1; // 擦除页数
    HAL_FLASH_Unlock(); // 解锁闪存
    if (HAL_FLASHEx_Erase(&erase_init_struct, &page_error) != HAL_OK)
    {
        // 擦除失败处理
        HAL_FLASH_Lock(); // 锁定闪存
        return;
    }
    HAL_FLASH_Lock(); // 锁定闪存
}

void write_flash(float data[DATA_NUM])
{
    uint32_t *data_ptr = (uint32_t*)data; // 将数据转换为指针类型
    HAL_FLASH_Unlock(); // 解锁闪存
    // 检查是否对齐
    if ((PageAddress % 4) != 0)
    {
        // 地址不对齐处理
        HAL_FLASH_Lock(); // 锁定闪存
        return;
    }
    for (int i = 0; i < DATA_NUM; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, PageAddress + i * 4, data_ptr[i]) != HAL_OK)
        {
            // 写入失败处理
            HAL_FLASH_Lock(); // 锁定闪存
            return;
        }
    }
    HAL_FLASH_Lock(); // 锁定闪存
}

float read_flash(uint8_t index)
{
    uint32_t address = PageAddress + index * 4; // 计算地址
    return *(__IO float*)address;
}

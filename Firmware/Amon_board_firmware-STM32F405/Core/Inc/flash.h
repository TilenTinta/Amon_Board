/*****************************************************************
 * File Name          : flash.h
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2024/08/31
 * Description        : MCU flash driver (save/read data)
*****************************************************************/

#ifndef FLASH_H_
#define FLASH_H_

#include "main.h"
#include <stdint.h>

// MCU with 1MB of flash - correct linker script: FLASH = 1024K -> 896K
#define FLASH_USER_START_ADDR  0x080E0000U
#define FLASH_USER_END_ADDR    0x080FFFFFU
#define FLASH_USER_SECTOR      FLASH_SECTOR_11


uint32_t Flash_Write_Data(uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords);

void Flash_Read_Data(uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords);

void Flash_Write_NUM(uint32_t StartSectorAddress, float Num);

float Flash_Read_NUM(uint32_t StartSectorAddress);

#endif

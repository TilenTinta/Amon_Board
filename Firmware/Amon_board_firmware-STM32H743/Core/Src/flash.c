/*****************************************************************
 * File Name          : flash.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2024/08/31
 * Description        : MCU flash driver (save/read data) - sector based
*****************************************************************/

#include "flash.h"
#include "string.h"
#include "stdio.h"

uint8_t bytes_temp[4];

void Convert_To_Str(uint32_t *Data, char *Buf);
float Bytes2float(uint8_t * ftoa_bytes_temp);


/*###########################################################################################################################################################*/
/* Functions */

/*********************************************************************
* @fn     	GetSector
*
* @param 	Address: wanted sector value for specific address
*
* @brief   	Return a value of sector where searched address is located
*
* @return  	sector value
*/
uint32_t GetSector(uint32_t Address)
{
    if (Address < 0x08100000)
    {
        return (Address - 0x08000000) / FLASH_PAGE_SIZE;
    }
    else
    {
        return (Address - 0x08100000) / FLASH_PAGE_SIZE;
    }
}



/*********************************************************************
* @fn     	GetBank
*
* @param 	Address: wanted bank value for specific address
*
* @brief   	Return a value of bank where searched address is located
*
* @return  	bank value
*/
uint32_t GetBank(uint32_t Address)
{
    if (Address < 0x08100000)
        return FLASH_BANK_1;
    else
        return FLASH_BANK_2;
}



/*********************************************************************
* @fn     	float2Bytes - unused
*
* @param 	*ftoa_bytes_temp: pointer to output variable
* @param 	float_variable: input value
*
* @brief   	Convert float value to bytes
*
* @return  	none
*/
void float2Bytes(uint8_t * ftoa_bytes_temp, float float_variable)
{
	union {
	  float a;
	  uint8_t bytes[4];
	} thing;

	thing.a = float_variable;

	for (uint8_t i = 0; i < 4; i++)
	{
		ftoa_bytes_temp[i] = thing.bytes[i];
	}
}



/*********************************************************************
* @fn     	Bytes2float - unused
*
* @param 	*ftoa_bytes_temp: input data
*
* @brief   	Conversion of 4 byte variable to float
*
* @return  	float value
*/
float Bytes2float(uint8_t * ftoa_bytes_temp)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    for (uint8_t i = 0; i < 4; i++) {
    	thing.bytes[i] = ftoa_bytes_temp[i];
    }

   float float_variable =  thing.a;
   return float_variable;
}




/*********************************************************************
* @fn     	Flash_Write_Data
*
* @param 	StartPageAddress: Address where data will be writen - 32-byte alignment
* @param 	*Data: pointer to data you want to write
* @param	numberofwords: amount of data you want to write
*
* @brief   	Write data to flash
*
* @return  	OK: 0, NOK: >0
*/
uint32_t Flash_Write_Data(uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords)
{
    FLASH_EraseInitTypeDef EraseInitStruct;

    __disable_irq(); // *may cause a problem
    HAL_FLASH_Unlock();

    // Get mcu flash memory layout
	uint32_t FirstSector  = GetSector(StartPageAddress);
	uint32_t EndAddress = StartPageAddress + (numberofwords * 4U) - 1U;
	uint32_t LastSector  = GetSector(EndAddress);
	uint32_t NumberOfSectors  = (LastSector - FirstSector) + 1U;
	uint32_t Bank = GetBank(StartPageAddress);

    // Erase structure for STM32H7 (page erase)
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.Banks     = Bank;
	EraseInitStruct.Sector    = FirstSector;
	EraseInitStruct.NbSectors = NumberOfSectors;

    uint32_t PageError = 0;
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        __enable_irq();
        return HAL_FLASH_GetError();
    }

    // Program words
    uint32_t current_address = StartPageAddress;
    uint32_t words_written = 0;
    // __attribute__((aligned(32)))
    uint32_t flashword[8];

    while (words_written < numberofwords)
    {
	    // Fill one 32-byte
	    for (uint32_t i = 0; i < 8; i++)
	    {
		    if (words_written < numberofwords)
		    {
			    flashword[i] = Data[words_written];
			    words_written++;
		    }
		    else
		    {
		       flashword[i] = 0xFFFFFFFF;
		    }
	    }

	    // Write 32-byte (uint32_t = 4byte * 8 = 32byte)
	    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, current_address, (uint32_t)flashword) != HAL_OK)
	    {
		    HAL_FLASH_Lock();
		    __enable_irq();
		    return HAL_FLASH_GetError();
	    }

	    current_address += 32; // 32-bytes written at once
    }

    HAL_FLASH_Lock();
    __enable_irq();

    return 0;
}



/*********************************************************************
* @fn     	Flash_Read_Data
*
* @param 	StartPageAddress: Address from where data will be read
* @param 	*RxBuf: pointer to variable where data will be stored
* @param	numberofwords: amount of data you want to read
*
* @brief   	Read data from flash
*
* @return  	OK: 0, NOK: >0
*/
void Flash_Read_Data(uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords)
{
    while (numberofwords--)
    {
        *RxBuf = *(volatile uint32_t *)StartPageAddress; // __IO
        StartPageAddress += 4U;
        RxBuf++;
    }
}



/*********************************************************************
* @fn     	Convert_To_Str
*
* @param 	*Data: Input data
* @param 	*Buf: output data
*
* @brief   	Convert 4 byte data to string
*
* @return  	none
*/
void Convert_To_Str(uint32_t *Data, char *Buf)
{
	int numberofbytes = ((strlen((char *)Data) / 4) + ((strlen((char *)Data) % 4) != 0)) * 4;

	for (int i = 0; i < numberofbytes; i++)
	{
		Buf[i] = Data[i/4]>>(8 * (i % 4));
	}
}



/*********************************************************************
* @fn     	Flash_Write_NUM
*
* @param 	StartPageAddress: Address where data will be writen
* @param 	Num: float value
*
* @brief   	Write data to MCU flash - float number -> one 4 byte number
*
* @return  	none
*/
void Flash_Write_NUM(uint32_t StartSectorAddress, float Num)
{
    uint32_t word;
    memcpy(&word, &Num, sizeof(word));
    Flash_Write_Data(StartSectorAddress, &word, 1);
}



/*********************************************************************
* @fn     	Flash_Read_NUM
*
* @param 	StartPageAddress: Address from where data will be read
*
* @brief   	Read data from MCU flash - one 4 byte number -> float number
*
* @return  	float number
*/
float Flash_Read_NUM(uint32_t StartSectorAddress)
{
    uint32_t word = 0xFFFFFFFF;
    float value;
    Flash_Read_Data(StartSectorAddress, &word, 1);
    memcpy(&value, &word, sizeof(value));
    return value;
}

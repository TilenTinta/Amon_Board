/*****************************************************************
 * File Name          : logging.c
 * Author             : Tinta T.
 * Version            : V1.0.0
 * Date               : 2026/02/26
 * Description        : Log data to flash / SD card
*****************************************************************/

#include "logging.h"
#include <string.h>

/*###########################################################################################################################################################*/
/* Defines */
// LittleFS region in external flash
// Change this if you want to reserve space at beginning/end of flash.

#ifndef LFS_EXT_FLASH_BASE_ADDR
#define LFS_EXT_FLASH_BASE_ADDR   (0u)                 // start of W25Q128
#endif

#ifndef LFS_EXT_FLASH_SIZE_BYTES
#define LFS_EXT_FLASH_SIZE_BYTES  (EXT_FLASH_SIZE)     // 16MB for W25Q128JV
#endif

#define LFS_BLOCK_SIZE            (EXT_FLASH_SECTOR_SIZE)   // 4096
#define LFS_PROG_SIZE             (EXT_FLASH_PAGE_SIZE)     // 256
#define LFS_READ_SIZE             (EXT_FLASH_PAGE_SIZE)     // 256

// Check if region aligns to erase blocks
#if (LFS_EXT_FLASH_BASE_ADDR % LFS_BLOCK_SIZE) != 0
#error "LFS_EXT_FLASH_BASE_ADDR must be aligned to EXT_FLASH_SECTOR_SIZE (4KB)"
#endif
#if (LFS_EXT_FLASH_SIZE_BYTES % LFS_BLOCK_SIZE) != 0
#error "LFS_EXT_FLASH_SIZE_BYTES must be multiple of EXT_FLASH_SECTOR_SIZE (4KB)"
#endif


static lfs_t lfs;
static struct lfs_config cfg;
static lfs_file_t file;


s_logging_buffer log_buffer[LOG_BUFFER_SIZE]; // buffer where data is stored before logging to NVM


int log_flash_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
int log_flash_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
int log_flash_erase(const struct lfs_config *c, lfs_block_t block);
int log_flash_sync(const struct lfs_config *c);


/*###########################################################################################################################################################*/
/* Functions - helpers */

/*********************************************************************
 * @fcn     log_addr
 *
 * @param   *c: pointer to littlefs config structure
 * @param   block: logical block number from littlefs
 * @param   off: offset inside the block
 *
 * @brief   Convert littlefs block + offset into absolute external
 *          flash memory address
 *
 * @return  Absolute flash address
 */
static inline uint32_t log_addr(const struct lfs_config *c, lfs_block_t block, lfs_off_t off)
{
    (void)c;
    return (uint32_t)LFS_EXT_FLASH_BASE_ADDR + (uint32_t)block * (uint32_t)LFS_BLOCK_SIZE + (uint32_t)off;
}



/*********************************************************************
 * @fcn     log_flash_read
 *
 * @param   *c: pointer to littlefs config structure
 * @param   block: logical block number to read from
 * @param   off: offset inside the block
 * @param   *buffer: pointer to destination buffer
 * @param   size: number of bytes to read
 *
 * @brief   LittleFS callback function for reading data from
 *          external flash memory
 *
 * @return  0 on success or negative littlefs error code
 */
int log_flash_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
    // Optional safety checks (can remove for speed)
    if (!buffer) return LFS_ERR_INVAL;
    if ((off + size) > c->block_size) return LFS_ERR_INVAL;

    uint32_t addr = log_addr(c, block, off);
    Flash_Read(addr, (uint8_t*)buffer, (uint32_t)size);
    return 0;
}



/*********************************************************************
 * @fcn     log_flash_prog
 *
 * @param   *c: pointer to littlefs config structure
 * @param   block: logical block number to program
 * @param   off: offset inside the block (must be prog_size aligned)
 * @param   *buffer: pointer to data buffer to write
 * @param   size: number of bytes to write (must be prog_size aligned)
 *
 * @brief   LittleFS callback function for programming (writing)
 *          data to external flash memory
 *
 * @return  0 on success or negative littlefs error code
 */
int log_flash_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
    if (!buffer) return LFS_ERR_INVAL;
    if ((off + size) > c->block_size) return LFS_ERR_INVAL;

    // littlefs expects prog aligned to prog_size
    if ((off % c->prog_size) != 0) return LFS_ERR_INVAL;
    if ((size % c->prog_size) != 0) return LFS_ERR_INVAL;

    uint32_t addr = log_addr(c, block, off);
    Flash_Write(addr, (uint8_t*)buffer, (uint32_t)size);
    return 0;
}



/*********************************************************************
 * @fcn     log_flash_erase
 *
 * @param   *c: pointer to littlefs config structure
 * @param   block: logical block number to erase
 *
 * @brief   LittleFS callback function for erasing one 4KB sector
 *          in external flash memory
 *
 * @return  0 on success or negative littlefs error code
 */
int log_flash_erase(const struct lfs_config *c, lfs_block_t block)
{
    (void)c;

    uint32_t addr = (uint32_t)LFS_EXT_FLASH_BASE_ADDR + (uint32_t)block * (uint32_t)LFS_BLOCK_SIZE;

    // littlefs erase block = 4KB sector
    Flash_SErase4k(addr);
    return 0;
}



/*********************************************************************
 * @fcn     log_flash_sync
 *
 * @param   *c: pointer to littlefs config structure
 *
 * @brief   Synchronization callback for littlefs.
 *          Used to ensure all pending flash operations are complete.
 *          For blocking flash driver this function performs no action.
 *
 * @return  0 (always successful)
 */
int log_flash_sync(const struct lfs_config *c)
{
    (void)c;
    // Driver is blocking + waits internally after program/erase
    return 0;
}



/*###########################################################################################################################################################*/
/* Functions - API */


/*********************************************************************
 * @fcn    log_init
 *
 * @brief  Initialization of functions used in LittleFS
 *
 * @return  none
 */
void log_init(void)
{
    Flash_Init();

    cfg.read  = log_flash_read;
    cfg.prog  = log_flash_prog;
    cfg.erase = log_flash_erase;
    cfg.sync  = log_flash_sync;

    cfg.read_size = LFS_READ_SIZE;
    cfg.prog_size = LFS_PROG_SIZE;
    cfg.block_size = LFS_BLOCK_SIZE;
    cfg.block_count = LFS_EXT_FLASH_SIZE_BYTES / LFS_BLOCK_SIZE;
    cfg.cache_size = 256;
    cfg.lookahead_size = 256;
    cfg.block_cycles = 500;

    if (lfs_mount(&lfs, &cfg))
    {
        lfs_format(&lfs, &cfg);
        lfs_mount(&lfs, &cfg);
    }
}



/*********************************************************************
 * @fcn    log_test_write
 *
 * @brief  Write dummy data with LittleFS to file
 *
 * @return  none
 */
void log_test_write(void)
{
    const char *msg = "Hello Drone\n";

    lfs_file_open(&lfs, &file,
                  "test.txt",
                  LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND);

    lfs_file_write(&lfs, &file, msg, strlen(msg));

    lfs_file_close(&lfs, &file);
}



/*********************************************************************
 * @fcn    log_test_read
 *
 * @brief  Read data from file with LittleFS
 *
 * @return  none
 */
void log_test_read(void)
{
    char buffer[64];
    int bytes_read;

    lfs_file_open(&lfs, &file, "test.txt", LFS_O_RDONLY);

    //while()
     bytes_read = lfs_file_read(&lfs,
							   &file,
							   buffer,
							   sizeof(buffer));
//    {
//        HAL_UART_Transmit(&huart1,
//                          (uint8_t*)buffer,
//                          bytes_read,
//                          HAL_MAX_DELAY);
//    }

    lfs_file_close(&lfs, &file);
}



/*********************************************************************
 * @fcn    log_list_files
 *
 * @brief  Check available files in file system of LittleFS
 *
 * @return  TODO
 */
void log_list_files(void)
{
    lfs_dir_t dir;
    struct lfs_info info;

    lfs_dir_open(&lfs, &dir, "/");

    lfs_dir_close(&lfs, &dir);
}



/*********************************************************************
 * @fcn    log_open_file
 *
 * @brief  Open file where logs will be saved
 *
 * @return  0: OK, else: NOK
 */
int log_open_file(void)
{
    int err = lfs_file_open(&lfs, &file,
                  "log.txt",
                  LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND);
    return err;
}



/*********************************************************************
 * @fcn    log_close_file
 *
 * @brief  Close file where logs were saved
 *
 * @return  0: OK, else: NOK
 */
int log_close_file(void)
{
    int err = lfs_file_close(&lfs, &file);
    return err;
}



/*********************************************************************
 * @fcn    log_flush
 *
 * @brief  Save data to flash
 *
 * @param	size: amount of data instance
 *
 * @return  none
 */
void log_write_buffer(uint8_t size)
{
    lfs_file_write(&lfs,
                   &file,
				   log_buffer,
                   size * sizeof(s_logging_buffer));
}



/*********************************************************************
 * @fcn    	log_add_sample
 *
 * @brief  	add data to buffer
 *
 * @param	pos: pointer to drone position struct
 * @param	data: pointer to drone data struct
 *
 * @return  none
 */
void log_add_sample(s_position *pos, s_data *data)
{
	static uint8_t index = 0;

    log_buffer[index].timestamp = HAL_GetTick(); // miliseconds
    log_buffer[index].Pitch = pos->Pitch;
    log_buffer[index].Roll = pos->Roll;
    log_buffer[index].Yaw = pos->Yaw;
    log_buffer[index].gyroTemp = pos->gyroTemp;
    log_buffer[index].accel_x = pos->accel_x;
    log_buffer[index].accel_y = pos->accel_y;
    log_buffer[index].accel_z = pos->accel_z;
    log_buffer[index].gyro_x = pos->gyro_x;
    log_buffer[index].gyro_y = pos->gyro_y;
    log_buffer[index].gyro_z = pos->gyro_z;
    log_buffer[index].height_TOF_mm = pos->height_TOF_mm;
    log_buffer[index].height_baro_m = pos->height_baro_m;
    log_buffer[index].battery_main_voltage = data->battery_main_voltage;
    log_buffer[index].battery_edf_voltage = data->battery_edf_voltage;
    log_buffer[index].temperature = data->temperature;
    log_buffer[index].humidity = data->humidity;
    log_buffer[index].pressure = data->pressure;

    index++;

    if (index >= LOG_BUFFER_SIZE)
    {
    	log_write_buffer(index);
        index = 0;
    }
}



// Sends full LittleFS file over UART as raw bytes.
// Return: 0 on success, negative littlefs error on FS failure, -1000 on UART failure.

/*********************************************************************
 * @fcn    	log_dump_uart
 *
 * @brief  	Sends full LittleFS file over UART as raw bytes.
 *
 * @param	path: pointer to file you want to read
 * @param	huart: pointer to uart data struct
 *
 * @return  0: OK, else: NOK
 */
int log_dump_uart(const char *path, UART_HandleTypeDef *huart)
{
    lfs_file_t f;
    uint8_t txbuf[128];

    int err = lfs_file_open(&lfs, &f, path, LFS_O_RDONLY);
    if (err < 0)
    {
        return err;
    }

    // Read starts at beginning of file
    lfs_soff_t s = lfs_file_seek(&lfs, &f, 0, LFS_SEEK_SET);
    if (s < 0)
    {
        lfs_file_close(&lfs, &f);
        return (int)s;
    }

    while (1)
    {
        lfs_ssize_t rd = lfs_file_read(&lfs, &f, txbuf, sizeof(txbuf));
        if (rd < 0)
        {
            err = (int)rd;
            break;
        }
        if (rd == 0)
        {
            err = 0; // EOF
            break;
        }

        if (HAL_UART_Transmit(huart, txbuf, (uint16_t)rd, HAL_MAX_DELAY) != HAL_OK)
        {
            err = -100; // UART transmit error
            break;
        }
    }

	int cerr = lfs_file_close(&lfs, &f);
	if (err == 0 && cerr < 0)
	{
		err = cerr;
	}

    return err;
}


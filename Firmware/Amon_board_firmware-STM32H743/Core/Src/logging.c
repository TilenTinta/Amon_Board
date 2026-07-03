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
static uint8_t log_buffer_index = 0;

#ifdef LOG_RAW_FLASH
static uint32_t raw_log_write_addr = 0;
static uint32_t raw_log_size_bytes = 0;
static uint8_t raw_log_open = 0;
#endif


int log_flash_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
int log_flash_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
int log_flash_erase(const struct lfs_config *c, lfs_block_t block);
int log_flash_sync(const struct lfs_config *c);
void log_write_buffer(uint8_t size);


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
int log_init(void)
{
    Flash_Init();

	#ifdef LOG_LITTLEFS
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
	#endif

	#ifdef LOG_RAW_FLASH
		raw_log_write_addr = 0;
		raw_log_size_bytes = 0;
		raw_log_open = 0;
		log_buffer_index = 0;
	#endif

    return 0;
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
	#ifdef LOG_LITTLEFS
		const char *msg = "Hello Drone\n";

		lfs_file_open(&lfs, &file,
					  "test.txt",
					  LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND);

		lfs_file_write(&lfs, &file, msg, strlen(msg));

		lfs_file_close(&lfs, &file);
	#endif
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
	#ifdef LOG_LITTLEFS
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
	#endif
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
	#ifdef LOG_LITTLEFS
		lfs_dir_t dir;
		struct lfs_info info;

		lfs_dir_open(&lfs, &dir, "/");

		lfs_dir_close(&lfs, &dir);
	#endif
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
	#ifdef LOG_LITTLEFS
		int err = lfs_file_open(&lfs, &file,
					  "log.txt",
					  LFS_O_WRONLY | LFS_O_CREAT | LFS_O_APPEND);
		return err;
	#endif

	#ifdef LOG_RAW_FLASH
		raw_log_write_addr = 0;
		raw_log_size_bytes = 0;
		raw_log_open = 1;
		log_buffer_index = 0;

		Flash_ChipErase();
		return 0;
	#endif
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
	#ifdef LOG_LITTLEFS
		if (log_buffer_index > 0)
		{
			log_write_buffer(log_buffer_index);
			log_buffer_index = 0;
		}

		int err = lfs_file_close(&lfs, &file);
		return err;
	#endif

	#ifdef LOG_RAW_FLASH
		if (raw_log_open && log_buffer_index > 0)
		{
			log_write_buffer(log_buffer_index);
			log_buffer_index = 0;
		}

		raw_log_open = 0;
		return 0;
	#endif
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
	#ifdef LOG_LITTLEFS
		lfs_file_write(&lfs,
					   &file,
					   log_buffer,
					   size * sizeof(s_logging_buffer));
	#endif

	#ifdef LOG_RAW_FLASH
		uint32_t bytes = (uint32_t)size * (uint32_t)sizeof(s_logging_buffer);

		if (!raw_log_open || bytes == 0)
		{
			return;
		}

		if ((raw_log_write_addr + bytes) > EXT_FLASH_SIZE)
		{
			raw_log_open = 0;
			return;
		}

		Flash_Write(raw_log_write_addr, (uint8_t*)log_buffer, bytes);
		raw_log_write_addr += bytes;
		raw_log_size_bytes += bytes;
	#endif
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
void log_add_sample(s_position *pos, s_data *data, s_actuators *actuators)
{
    log_buffer[log_buffer_index].timestamp = HAL_GetTick(); // miliseconds
    log_buffer[log_buffer_index].servo_xp = actuators->servo_xp;
    log_buffer[log_buffer_index].servo_xn = actuators->servo_xn;
    log_buffer[log_buffer_index].servo_yp = actuators->servo_yp;
    log_buffer[log_buffer_index].servo_yn = actuators->servo_yn;
    log_buffer[log_buffer_index].nmpc_solver_time = data->nmpc_solver_time;
    log_buffer[log_buffer_index].heading_deg = pos->heading_deg;
    log_buffer[log_buffer_index].Pitch = pos->Pitch;
    log_buffer[log_buffer_index].Roll = pos->Roll;
    log_buffer[log_buffer_index].Yaw = pos->Yaw;
    log_buffer[log_buffer_index].accel_x = pos->accel_x;
    log_buffer[log_buffer_index].accel_y = pos->accel_y;
    log_buffer[log_buffer_index].accel_z = pos->accel_z;
    log_buffer[log_buffer_index].gyro_x = pos->gyro_x;
    log_buffer[log_buffer_index].gyro_y = pos->gyro_y;
    log_buffer[log_buffer_index].gyro_z = pos->gyro_z;
    log_buffer[log_buffer_index].gyroTemp = pos->gyroTemp;
    log_buffer[log_buffer_index].height_TOF_mm = pos->height_TOF_mm;
    log_buffer[log_buffer_index].height_baro_m = pos->height_baro_m;
    log_buffer[log_buffer_index].battery_main_voltage = data->battery_main_voltage;
    log_buffer[log_buffer_index].battery_edf_voltage = data->battery_edf_voltage;
    log_buffer[log_buffer_index].temperature = data->temperature;
    log_buffer[log_buffer_index].pressure = data->pressure;
    log_buffer[log_buffer_index].humidity = data->humidity;
    log_buffer[log_buffer_index].edf_percent = actuators->edf_percent;

    log_buffer_index++;

    if (log_buffer_index >= LOG_BUFFER_SIZE)
    {
    	log_write_buffer(log_buffer_index);
        log_buffer_index = 0;
    }
}



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
	#ifdef LOG_LITTLEFS
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
	#endif

	#ifdef LOG_RAW_FLASH
		(void)path;

		uint8_t txbuf[128];
		uint32_t addr = 0;
		uint32_t remaining = raw_log_size_bytes;

		while (remaining > 0)
		{
			uint32_t chunk = remaining;
			if (chunk > sizeof(txbuf))
			{
				chunk = sizeof(txbuf);
			}

			Flash_Read(addr, txbuf, chunk);

			if (HAL_UART_Transmit(huart, txbuf, (uint16_t)chunk, HAL_MAX_DELAY) != HAL_OK)
			{
				return -100;
			}

			addr += chunk;
			remaining -= chunk;
		}

		return 0;
	#endif
}



/*********************************************************************
 * @fcn     log_delete
 *
 * @brief   Delete flight log file from flash
 *
 * @return  none
 */
void log_remove(void)
{
	#ifdef LOG_LITTLEFS
		int err = lfs_remove(&lfs, "log.txt");

		if (err < 0)
		{
			printf("Delete failed: %d\n", err);
		}
	#endif

	#ifdef LOG_RAW_FLASH
		raw_log_open = 0;
		raw_log_write_addr = 0;
		raw_log_size_bytes = 0;
		log_buffer_index = 0;
		Flash_ChipErase();
	#endif
}


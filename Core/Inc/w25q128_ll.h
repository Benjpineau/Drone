/**
 * @file w25q128_ll.h 
 * @brief w25q128 low-level driver
 * @author Filip Stojanovic 
 */

#ifndef W25Q128_H 
#define W25Q128_H

#include "w25q128_conf_ll.h"

#define ERASE_BEFORE_PAGE_WRITE_AUTO 0
#define W25Q128_SECTOR_SIZE 4096
#define W25Q128_PAGE_SIZE   256
#define W25Q128_SECTOR_COUNT 4096

typedef enum {
    W25Q128_SUCCESS = 0,
    W25Q128_ERROR = 1,
    W25Q128_READY = 2,
    W25Q128_BUSY = 3,
    W25Q128_ERROR_TIMEOUT = 4,
} W25Q128_StatusTypeDef;

typedef enum {
    INST_WRITE_ENABLE = 0x06,
    INST_VOLATILE_SR_WRITE_ENABLE = 0x50,
    INST_WRITE_DISABLE = 0x04,

    INST_RELEASE_POWER_DOWN_ID = 0xAB,
    INST_MANUFACTURER_DEVICE_ID = 0x90,
    INST_JEDEC_ID = 0x9F,
    INST_READ_UNIQUE_ID = 0x4B,

    INST_READ_DATA = 0x03,
    INST_FAST_READ = 0x0B,
        
    INST_PAGE_PROGRAM = 0x02,

    INST_SECTOR_ERASE_4KB = 0x20,
    INST_BLOCK_ERASE_32KB = 0x52,
    INST_BLOCK_ERASE_64KB = 0xD8,
    INST_CHIP_ERASE = 0xC7, // 0x60

    INST_READ_STATUS_REG_1 = 0x05,
    INST_WRITE_STATUS_REG_1 = 0x01,
    INST_READ_STATUS_REG_2 = 0x35,
    INST_WRITE_STATUS_REG_2 = 0x31,
    INST_READ_STATUS_REG_3 = 0x15,
    INST_WRITE_STATUS_REG_3 = 0x11,

    INST_READ_SFDP_REG = 0x5A,
    INST_ERASE_SECURITY_REG = 0x44,
    INST_PROGRAM_SECURITY_REG = 0x42,
    INST_READ_SECURITY_REG = 0x48,

    INST_GLOBAL_BLOCK_LOCK = 0x7E,
    INST_GLOBAL_BLOCK_UNLOCK = 0x98,
    INST_READ_BLOCK_LOCK = 0x3D,
    INST_INDIVIDUAL_BLOCK_LOCK = 0x36,
    INST_INDIVIDUAL_BLOCK_UNLOCK = 0x39,

    INST_ERASE_PROGRAM_SUSPEND = 0x75,
    INST_ERASE_PROGRAM_RESUME = 0x7A,
    INST_POWER_DOWN = 0xB9,
        
    INST_ENABLE_RESET = 0x66,
    INST_RESET_DEVICE = 0x99,
} W25Q128_InstructionTypeDef;

typedef enum {
    ID_RELEASE_POWER_DOWN = 0,
    ID_MANUFACTURER_DEVICE = 1,
    ID_JEDEC = 2,
    ID_READ_UNIQUE = 3,
} W25Q128_ID_TypeDef;

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} W25Q128_TypeDef;


/**
 * @brief w25q128 SPI Chip Select function
 * @param w25q128 Pointer to the flash configuration struct
 * @return None
 */
void W25Q128_ChipSelect(W25Q128_TypeDef *w25q128);

/**
 * @brief w25q128 SPI Chip Deselect function
 * @param w25q128 Pointer to the flash configuration struct
 * @return None
 */
void W25Q128_ChipDeselect(W25Q128_TypeDef *w25q128);

/**
 * @brief w25q128 SPI Write function
 * @param w25q128 Pointer to the flash configuration struct
 * @param data Data pointer
 * @param len Size of data 
 * @param timeout Timeout of SPI write operation
 * @return None
 */
void W25Q128_SPIWrite(W25Q128_TypeDef *w25q128, uint8_t *data, uint16_t len, 
                                                            uint32_t timeout);

/**
 * @brief w25q128 SPI Read function
 * @param w25q128 Pointer to the flash configuration struct
 * @param data Data pointer
 * @param len Size of data 
 * @param timeout Timeout of SPI read operation
 * @return None
 */
void W25Q128_SPIRead(W25Q128_TypeDef *w25q128, uint8_t *data, uint16_t len, 
                                                            uint32_t timeout);

/**
 * @brief w25q128 delay function
 * @param delay_ms Delay in milliseconds
 * @return None
 * @note This function blocks the thread it is called in, use it with concern
 */
void W25Q128_DelayMs(uint32_t delay_ms);

/**
 * @brief Function used to reset w25q128 flash memory
 * @param w25q128 Pointer to the flash configuration struct
 * @return None
 */
void W25Q128_Reset(W25Q128_TypeDef *w25q128);

/**
 * @brief Function that reads the flash ID
 * @param w25q128 Pointer to the flash configuration struct
 * @param id Type of ID that is being read
 * @return w25q128 ID
 * @note Only JEDEC_ID can be obtained using this function at the moment.
 *       All other ID's will be provided in future versions of this library. 
 */
uint32_t W25Q128_ReadID(W25Q128_TypeDef *w25q128, W25Q128_ID_TypeDef id);

/**
 * @brief Function that reads data from w25q128 memory 
 * @param w25q128 Pointer to the flash configuration struct
 * @param start_page Start page of flash memory from which data reading
 *                   should start
 * @param offset Offset represented in bytes in the page from which data is 
 *               being red from
 * @param size Data size that is red
 * @param r_data Pointer to the receive buffer
 * @retval ::W25Q128_StatusTypeDef
 */
W25Q128_StatusTypeDef W25Q128_Read(W25Q128_TypeDef *w25,
                                    uint32_t start_page,
                                    uint8_t offset,
                                    uint32_t size,
                                    uint8_t *r_data);

/**
 * @brief Function that reads data in fast mode from w25q128 memory
 * @param w25q128 Pointer to the flash configuration struct
 * @param start_page Start page of flash memory from which data reading
 *                   should start
 * @param offset Offset represented in bytes in the page from which data is 
 *               being red from
 * @param size Data size that is red
 * @param r_data Pointer to the receive buffer
 * @retval ::W25Q128_StatusTypeDef
 * @note This function reads data much faster than regular W25Q128_Read function, 
 *       but note that is must send one dummy byte.
 */
W25Q128_StatusTypeDef W25Q128_FastRead(W25Q128_TypeDef *w25,
                                        uint32_t start_page,
                                        uint8_t offset,
                                        uint32_t size,
                                        uint8_t *r_data);

/**
 * @brief Function that enables any operation with w25q128
 * @param w25q128 Pointer to the flash configuration struct
 * @retval ::W25Q128_StatusTypeDef
 */
W25Q128_StatusTypeDef W25Q128_WriteEnable(W25Q128_TypeDef *w25);

/**
 * @brief Function that disables any operation with w25q128
 * @param w25q128 Pointer to the flash configuration struct
 * @retval ::W25Q128_StatusTypeDef
 */
W25Q128_StatusTypeDef W25Q128_WriteDisable(W25Q128_TypeDef *w25);

/**
 * @brief Function that erases specified sector
 * @param w25q128 Pointer to the flash configuration struct
 * @param num_sector Number of sector
 * @retval ::W25Q128_StatusTypeDef
 */
W25Q128_StatusTypeDef W25Q128_EraseSector(W25Q128_TypeDef *w25, 
                                                        uint16_t num_sector);

/**
 * @brief Function used to read status register
 * @param w25q128 Pointer to the flash configuration struct
 * @return Status register value
 */
uint8_t W25Q128_ReadStatusRegister(W25Q128_TypeDef *w25);

/**
 * @brief Function used to check if w25q128 has finished all operations
 * @param w25q128 Pointer to the flash configuration struct
 * @retval ::W25Q128_StatusTypeDef
 */
W25Q128_StatusTypeDef W25Q128_CheckBUSY(W25Q128_TypeDef *w25);

/**
 * @brief Function that writes data to a single page
 * @param w25q128 Pointer to the flash configuration struct
 * @param page Number of page that will is written
 * @param offset Offset represented in bytes inside specified page
 * @param data_size Data size written to page
 * @param data Data pointer
 * @retval ::W25Q128_StatusTypeDef
 */
W25Q128_StatusTypeDef W25Q128_WritePage(W25Q128_TypeDef *w25, uint32_t page, 
                                        uint16_t offset, uint32_t data_size, 
                                        uint8_t *data);
/**
 * @brief Function that writes multiple pages
 * @param w25q128 Pointer to the flash configuration struct
 * @param page Number of page that will is written
 * @param offset Offset represented in bytes inside specified page
 * @param size Data size written to page
 * @param data Data pointer
 * @retval ::W25Q128_StatusTypeDef
 */
W25Q128_StatusTypeDef W25Q128_Write(W25Q128_TypeDef *w25, uint32_t page, 
                                    uint16_t offset, uint32_t size, 
                                    uint8_t *data);


#endif 
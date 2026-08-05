#include "ICM42688P.h"
#include <stdio.h>

extern TIM_HandleTypeDef htim5;  //TIM5
extern SPI_HandleTypeDef hspi2; //SPI2

ICM42688P_Data_t imudata;
uint8_t buf[15]; // Reçoit 1 octet dummy + 14 octets de données
uint16_t last_count = 0;

/* ====================================================================
 * CONFIGURATION MATÉRIELLE : Remplacer par TES vraies broches de CS
 * ==================================================================== */
#define IMU_CS_PORT  GPIOA        //PA10
#define IMU_CS_PIN   GPIO_PIN_10  //PA10

// static inline void Icm_CS_LOW(void) {
//     HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
// }

// static inline void Icm_CS_HIGH(void) {
//     HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
// }

/* ====================================================================
 * FONCTIONS DE LECTURE/ÉCRITURE SÉRIE (SPI2 Bloquant pour l'Init)
 * ==================================================================== */
 uint8_t hal_Spi2_ReadWriteByte(uint8_t txdata)
 {
     uint8_t rxdata = 0;
     if (HAL_SPI_TransmitReceive(&hspi2, &txdata, &rxdata, 1, 100) != HAL_OK) {
         return 0x00; 
     }
     return rxdata;
 }

void Icm_Spi_ReadWriteNbytes(uint8_t* pBuffer, uint8_t len)
{
    for(uint8_t i = 0; i < len; i++)
    {
        *pBuffer = hal_Spi2_ReadWriteByte(*pBuffer);
        pBuffer++;
    }
}

uint8_t ICM42688P_ReadReg(uint8_t reg)
{
    uint8_t regval = 0;
    Icm_CS_LOW();                
    reg |= 0x80;                 // Bit D7 à 1 pour spécifier une lecture
    Icm_Spi_ReadWriteNbytes(&reg, 1);   
    Icm_Spi_ReadWriteNbytes(&regval, 1);
    Icm_CS_HIGH();               
    return regval;
}

uint8_t ICM42688P_WriteReg(uint8_t reg, uint8_t value)
{
    Icm_CS_LOW();
    reg &= ~0x80;                // Bit D7 à 0 pour spécifier une écriture
    Icm_Spi_ReadWriteNbytes(&reg, 1);   
    Icm_Spi_ReadWriteNbytes(&value, 1); 
    Icm_CS_HIGH();
    return 0;
}


void ICM42688P_Init(void)
{
    HAL_Delay(50); // Attente stabilisation de l'alim

    // Vérification du WHO_AM_I
    ICM42688P_WriteReg(MPUREG_REG_BANK_SEL, 0);
    uint8_t who_am_i = ICM42688P_ReadReg(MPUREG_WHO_AM_I);
    
    if (who_am_i != ICM_WHOAMI) {
        while(1) {
            printf("Erreur critique : ICM-42688-P non détecté !\r\n");
        }
    }

    // Configuration des modes Low Noise et fréquences
    ICM42688P_WriteReg(MPUREG_PWR_MGMT_0, ICM426XX_PWR_MGMT_0_GYRO_MODE_LN | ICM426XX_PWR_MGMT_0_ACCEL_MODE_LN);
    ICM42688P_WriteReg(MPUREG_GYRO_CONFIG0, ICM426XX_GYRO_CONFIG0_FS_SEL_500dps | ICM426XX_GYRO_CONFIG0_ODR_1_KHZ);
    ICM42688P_WriteReg(MPUREG_ACCEL_CONFIG0, ICM426XX_ACCEL_CONFIG0_FS_SEL_4g | ICM426XX_ACCEL_CONFIG0_ODR_1_KHZ);
    
    // Bypass du FIFO interne
    ICM42688P_WriteReg(MPUREG_FIFO_CONFIG, ICM426XX_FIFO_CONFIG_MODE_BYPASS);
    
    // Configuration de l'interruption matérielle (pour plus tard)
    ICM42688P_WriteReg(MPUREG_INT_CONFIG, ICM426XX_INT_CONFIG_INT1_DRIVE_CIRCUIT_PP | ICM426XX_INT_CONFIG_INT1_POLARITY_HIGH);
    ICM42688P_WriteReg(MPUREG_INT_SOURCE0, BIT_INT_SOURCE0_RESET_DONE_INT1_EN | BIT_INT_SOURCE0_UI_DRDY_INT1_EN);
    
    HAL_Delay(50);
}

/* ====================================================================
 * CONVERSIONS ET DECODAGE
 * ==================================================================== */
float ICM42688P_ConvertAccel(int16_t raw_accel, float sensitivity) {
    return (float)raw_accel * sensitivity;
}

float ICM42688P_ConvertGyro(int16_t raw_gyro, float sensitivity) {
    return (float)raw_gyro * sensitivity;
}

float ICM42688P_ConvertTemp(int16_t raw_temp) {
    return ((float)raw_temp / 132.48f) + 25.0f; // Constantes constructeur ICM
}

uint8_t ICM42688P_decode(ICM42688P_Data_t *data)
{

    data->temperature_C = ICM42688P_ConvertTemp((int16_t)((buf[1] << 8) | buf[2]));

    data->accel_x = ICM42688P_ConvertAccel((int16_t)((buf[3] << 8) | buf[4]), 1.0f / 8192.0f); // Sensibilité 4G
    data->accel_y = ICM42688P_ConvertAccel((int16_t)((buf[5] << 8) | buf[6]), 1.0f / 8192.0f);
    data->accel_z = ICM42688P_ConvertAccel((int16_t)((buf[7] << 8) | buf[8]), 1.0f / 8192.0f);
    
    data->gyro_x = ICM42688P_ConvertGyro((int16_t)((buf[9] << 8) | buf[10]), 1.0f / 65.5f);     // Sensibilité 500dps
    data->gyro_y = ICM42688P_ConvertGyro((int16_t)((buf[11] << 8) | buf[12]), 1.0f / 65.5f);
    data->gyro_z = ICM42688P_ConvertGyro((int16_t)((buf[13] << 8) | buf[14]), 1.0f / 65.5f);


    return 1;
}

/* ====================================================================
 * RECUPERATION DES DONNEES (Version synchrone à intervalle régulier) | en attendant de fix le DMA
 * ==================================================================== */
uint8_t ICM42688P_GetData(ICM42688P_Data_t *data) 
{
    uint8_t tx_buf[15] = {0};
    tx_buf[0] = 0x1D | 0x80; // 0x1D = Registre de départ des données (TEMP_DATA1) + Mode Read (0x80)
    
    Icm_CS_LOW();
    
    // On démarre le transfert sur SPI2
    HAL_SPI_TransmitReceive_DMA(&hspi2, tx_buf, buf, 15);
    
    while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY) {
        // Attente de la fin du transfert SPI2
    }
    
    Icm_CS_HIGH(); 

    ICM42688P_decode(data);
    
    return 1;
}
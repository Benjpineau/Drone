/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ICM42688P.h"
#include "barometre.h"
#include "pwm.h"
#include "filter.h"
#include "w25q128_ll.h"
#include "blackbox.h"
#include "pid.h"
#include "attitude.h"
#include "rc_link.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_tx;
DMA_HandleTypeDef hdma_i2c2_rx;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi3_rx;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim5_ch1;
DMA_HandleTypeDef hdma_tim5_ch2;
DMA_HandleTypeDef hdma_tim5_ch3;
DMA_HandleTypeDef hdma_tim5_ch4;

UART_HandleTypeDef huart7;

/* USER CODE BEGIN PV */

ICM42688P_Data_t donnees_imu;
baro_data_t donnees_baro;
Filtre_PT1_Axe_t lpf_gyro_pt1[3];
//Filtre_PT2_Axe_t lpf_gyro_pt2[3];
Filtre_PT1_Axe_t filtre_accel[3];
float altitude_relative = 0.0f;

Filtre_Notch_3Axes_t filtre_gyro_notch; // Pour le filtre Notch du gyro
Quaternion_t orientation_drone = {1.0f, 0.0f, 0.0f, 0.0f};         // Pour stocker le quaternion
Euler_t angles_drone;             // Pour stocker les angles d'Euler (Pitch, Roll, Yaw)



uint32_t tick_precedent_boucle = 0;
uint32_t tick_precedent_baro = 0;
uint32_t tick_precedent_blackbox = 0;

#define GRAVITY 9.81f
#define MASS 500.0f


#define KR_ROLL   250.0f
#define KR_PITCH  250.0f
#define MAX_RATE_CMD_DEGPS 200.0f // sécurité : sature la consigne de vitesse envoyée à la boucle interne


float consigne_forward = 0.0f;
float consigne_lateral = 0.0f;
float consigne_vitesse_yaw = 0.0f; // deg/s, commandée directement par le stick droit
float consigne_accel_z = 0.0f;
uint16_t base_throttle = 1200; // PLACEHOLDER : à ajuster une fois le vol stationnaire caractérisé

Controller_Input_t cmd_in;
Controller_Output_t cmd_out;
Controller_State_t cmd_state;




// ---- Mapping stick -> consignes ----
#define STICK_RANGE           512.0f  // plage brute du stick (~-511..512, cf rc_link.h)
#define MAX_TILT_ANGLE_DEG    25.0f   // déflexion max du stick gauche -> angle roll/pitch cible
#define MAX_YAW_RATE_DEGPS    180.0f  // PLACEHOLDER : déflexion max du stick droit -> vitesse de lacet

// ---- Gestion gaz (boutons haut/bas) ----
#define CLIMB_RATE_PWM_PER_S  100.0f  // vitesse de montée/descente du throttle pendant l'appui
#define CLIMB_OFFSET_MAX_PWM  200.0f  // amplitude max de l'offset par rapport au throttle de base
#define BASE_THROTTLE_HOVER   1200    // PLACEHOLDER : throttle de vol stationnaire, à mesurer au banc
static float throttle_offset = 0.0f;  // offset courant, ramené à 0 si aucun bouton n'est appuyé

#define ACCEL_VERT 100 //TROUVER LA BONNE VALEUR PLUS TARD

//BLACKBOX PV
W25Q128_TypeDef ma_flash;
uint32_t blackbox_current_page = 0;   // On commence à la page 0
uint16_t blackbox_current_offset = 0; // Octet 0 de la page 0
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM5_Init(void);
static void MX_UART7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline int32_t Clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float Clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// Etat armé/désarmé :
static uint8_t armed = 0;


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM5_Init();
  MX_UART7_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  InitState(&cmd_state);//Initialisation du State

  //Ajout des constantes dans la commande
  cmd_in.mass = MASS;
  cmd_in.g = GRAVITY;

  // Arme la réception UART7 depuis l'ESP32 (manette PS4 -> ESP32 -> UART7)
  RC_Init(&huart7);

  //Démarrage des moteurs.
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, PWM_ZERO);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, PWM_ZERO);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, PWM_ZERO);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, PWM_ZERO);
  HAL_Delay(3000);

  // --- AJOUT : Force la broche CS (PA10) à 1 pour désélectionner proprement l'IMU au repos ---
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
  //Init des capteurs
  ICM42688P_Init();

  // Pour le PT1 (f_coupure, dt)
  Filtre_PT1_3Axes_Init(lpf_gyro_pt1, 40.0f, 0.005f); //Fc:40Hz, dt : 0.005 -> Fe : 200Hz
  //Filtre_Notch_3Axes_Config(&filtre_gyro_notch, 200.0f, 1000.0f, 3.0f);
  
  // Pour le PT2 (f_coupure, f_echantillonnage de la boucle soit 200 Hz)
  //Filtre_PT2_3Axes_Init(lpf_gyro_pt2, 40.0f, 200.0f); //Fc:40Hz, Fe : 200Hz

  Filtre_PT1_3Axes_Init(filtre_accel, 15.0f, 0.005f);//Fc:15Hz, dt : 0.005 -> Fe : 200Hz


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t tick_actuel = HAL_GetTick();

      if (tick_actuel - tick_precedent_boucle >= 1) 
      {
          float dt = (float)(tick_actuel - tick_precedent_boucle) / 1000.0f;
          tick_precedent_boucle = tick_actuel;

          // 1. LECTURE DE L'IMU (1000Hz)
          ICM42688P_GetData(&donnees_imu);
          donnees_imu.dt = dt; 
          donnees_imu.timestamp = tick_actuel;

          LPF_Gyro_PT1(&donnees_imu, lpf_gyro_pt1);
          //LPF_Gyro_PT2(&donnees_imu, &filtre_gyro);
          LPF_Accel_PT1(&donnees_imu, filtre_accel);

          // 3. Étape Notch : Résonance moteurs
          //Notch_Gyro(&donnees_imu, &filtre_gyro_notch);

          // 4. MISE À JOUR DE L'ORIENTATION (Quaternions)
          Update_Orientation_Quaternion(&donnees_imu, &orientation_drone, dt);

          // 5. EXTRACTION DES ANGLES
          Quaternion_To_Euler(&orientation_drone, &angles_drone);



          // 2. LECTURE DU BAROMÈTRE (Toutes les 100 ms = 10 Hz)
          if (tick_actuel - tick_precedent_baro >= 100) 
          {
              tick_precedent_baro = tick_actuel;
              donnees_baro = obtenirDonneesCompensees();
              altitude_relative = obtenirAltitudeRelative();
          }

          // // ====================================================================
          // // LIAISON RC (ESP32 -> UART7) : lecture + failsafe
          // // ====================================================================
          // // rc_command.{roll,pitch,yaw,aux,throttle,buttons} contient la dernière
          // // trame valide reçue (mise à jour dans rc_link.c via interruption UART7).
          // // RC_IsLinkValid() renvoie 0 si aucune trame valide n'est arrivée depuis
          // // plus de 300 ms
          // uint8_t rc_link_ok = RC_IsLinkValid(300);
          // if (!rc_link_ok) {
          //     /* TO DO : IMPLEMENTER UNE DESCENTE DOUCE SI LA LIAISON EST PERDUE */
          // }

          // // ====================================================================
          // // ARMEMENT / DÉSARMEMENT (bouton bascule)
          // // ====================================================================
          // // PLACEHOLDER : bit2 -- remapper sur le bouton DS4 voulu.
          // // Détection de front montant pour n'inverser l'état qu'une fois par appui,
          // // pas en continu tant que le bouton reste enfoncé.
          // {
          //     static uint8_t bouton_arm_precedent = 0;
          //     uint8_t bouton_arm = rc_command.buttons & (1 << 2);

          //     if (bouton_arm && !bouton_arm_precedent) {
          //         armed = !armed;
          //         InitState(&cmd_state);
          //     }
          //     bouton_arm_precedent = bouton_arm;

          //     // Perte de liaison -> retour forcé à désarmé, pour ne pas rester "armé"
          //     // en silence si le lien revient plus tard sans qu'on l'ait demandé.
          //     if (!rc_link_ok) {
          //         armed = 0;
          //     }
          // }

          // // ====================================================================
          // // MAPPING STICK -> CONSIGNES
          // // ====================================================================
          // consigne_forward  = (rc_command.roll  / STICK_RANGE) * MAX_TILT_ANGLE_DEG;
          // consigne_lateral = (rc_command.pitch / STICK_RANGE) * MAX_TILT_ANGLE_DEG;
          // consigne_vitesse_yaw = (rc_command.yaw / STICK_RANGE) * MAX_YAW_RATE_DEGPS;
          // if (rc_command.buttons & (1 << 0)){
          //   consigne_accel_z =  -ACCEL_VERT;
          // }
          // else if (rc_command.buttons & (1 << 1)){
          //   consigne_accel_z = ACCEL_VERT;
          // }
          // else{
          //   consigne_accel_z = 0.f;
          // }
          // Vec3_t accel_d = {consigne_forward, consigne_lateral, consigne_accel_z};
          // Vec3_t accel_s = {donnees_imu.accel_x,donnees_imu.accel_y,donnees_imu.accel_z};
          // Vec3_t ws = {donnees_imu.gyro_x,donnees_imu.gyro_y,donnees_imu.gyro_z};

          // // ====================================================================
          // // CALCUL DU PID EN CASCADE
          // // ====================================================================
          // MapInput(&cmd_in, accel_d, accel_s, ws, &orientation_drone, consigne_vitesse_yaw, donnees_imu.dt);
          // Global_Controller(&cmd_in,&cmd_state, &cmd_out, tick_actuel);


          // // ====================================================================
          // // MIXER MATRICIEL & ENVOI AUX MOTEURS
          // // ====================================================================
          // if (rc_link_ok && armed) {
          //     int32_t m1 = ...; // TO DO : IMPLEMENTER LA COMMANDE DES MOTEURS
          //     int32_t m2 = ...; // TO DO : IMPLEMENTER LA COMMANDE DES MOTEURS
          //     int32_t m3 = ...; // TO DO : IMPLEMENTER LA COMMANDE DES MOTEURS
          //     int32_t m4 = ...; // TO DO : IMPLEMENTER LA COMMANDE DES MOTEURS

          //     // Envoi des commandes sécurisées à tes Timers PWM (computePower() dans pwm.c
          //     // sature déjà 1000-2000, mais on protège aussi contre un int32_t hors uint16_t)
          //     setMotor1((uint16_t)Clamp_i32(m1, 1000, 2000));
          //     setMotor2((uint16_t)Clamp_i32(m2, 1000, 2000));
          //     setMotor3((uint16_t)Clamp_i32(m3, 1000, 2000));
          //     setMotor4((uint16_t)Clamp_i32(m4, 1000, 2000));
          // } else if (rc_link_ok && !armed) {
          //       /* TO DO : IMPLEMENTER LA DESCENTE DOUCE SI LA LIAISON EST PERDUE */
          // }


          // ====================================================================
          // ESC CALIBRATION & TEST - remplace le bloc PID pour les tests
          // ====================================================================

          // ÉTAPE 1 : Calibration de la plage de gaz (à faire une seule fois)
          // Décommente ce bloc, flash, alimente les ESC, attends les bips, recommente
          
          setMotor1(2000); setMotor2(2000); setMotor3(2000); setMotor4(2000);
          HAL_Delay(3000); // ESC mémorise le MAX
          setMotor1(1000); setMotor2(1000); setMotor3(1000); setMotor4(1000);
          HAL_Delay(3000); // ESC mémorise le MIN → bips de confirmation
          while(1);        // Stop ici, recycle l'alim avant de continuer
          

          // ÉTAPE 2 : Test de montée progressive (après calibration)
          // Fait tourner tous les moteurs de 1000 à 1300 en 3 secondes puis stop
          static uint32_t t_debut = 0;
          static uint16_t commande = 1000;

          if (t_debut == 0) t_debut = HAL_GetTick();

          uint32_t elapsed = HAL_GetTick() - t_debut;

          if (elapsed < 3000) {
              // Montée linéaire de 1000 à 1300 sur 3 secondes
              commande = 1000 + (uint16_t)((elapsed * 300) / 3000);
          } else {
              // Stop moteurs après 3 secondes
              commande = 1000;
          }

          setMotor1(commande);
          setMotor2(commande);
          setMotor3(commande);
          setMotor4(commande);

          //ECRITURE SUR LA BLACKBOX
          if (tick_actuel-tick_precedent_blackbox >= 5)
            {               
                tick_precedent_blackbox = tick_actuel;

                Blackbox_Frame_t ma_trame;
                
                ma_trame.tick     = tick_actuel;
                ma_trame.gyro_x   = donnees_imu.gyro_x;
                ma_trame.gyro_y   = donnees_imu.gyro_y;
                ma_trame.gyro_z   = donnees_imu.gyro_z;
                ma_trame.accel_x  = donnees_imu.accel_x;
                ma_trame.accel_y  = donnees_imu.accel_y;
                ma_trame.accel_z  = donnees_imu.accel_z;
                ma_trame.altitude = altitude_relative;

                ma_trame.consigne_forward = consigne_forward;
                ma_trame.consigne_lateral = consigne_lateral;
                //ma_trame.consigne_vitesse_yaw = consigne_vitesse_yaw;
                //ma_trame.consigne_accel_z = consigne_accel_z;
  
                W25Q128_WritePage(&ma_flash, blackbox_current_page, blackbox_current_offset, sizeof(ma_trame), (uint8_t *)&ma_trame);

                blackbox_current_offset += sizeof(ma_trame); // +40 octets

                if ((blackbox_current_offset + sizeof(ma_trame)) > 256) 
                {
                    blackbox_current_page++;   
                    blackbox_current_offset = 0; 
                }
            }




      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 15;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 239;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 2499;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  // // Activer l'horloge du port A (si ce n'est pas déjà fait)
  // __HAL_RCC_GPIOA_CLK_ENABLE();

  // // Configurer PA10 en Sortie Push-Pull pour le CS de l'IMU
  // GPIO_InitStruct.Pin = GPIO_PIN_10;
  // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  // HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // // Mettre le CS à 1 par défaut (Désélectionné)
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

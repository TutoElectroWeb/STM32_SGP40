/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : 'exemple_sgp40_polling_aht20_compensation.c'
 * @brief          : [Polling] SGP40 avec compensation dynamique AHT20 (T/RH)
 ******************************************************************************
 * @details
 * Cet exemple lit température/humidité sur AHT20 en polling, puis injecte ces
 * valeurs dans SGP40_SetCompensation() avant chaque mesure VOC index.
 *
 * Configuration matérielle :
 * - I2C choisi dans CubeMX : mode standard/fast avec pull-up externes 4.7kΩ
 * - UART2 : console debug 115200 bauds
 *
 * API couverte :
 * - AHT20_Init, AHT20_ReadMeasurements, AHT20_StatusToString
 * - SGP40_Init, SGP40_SetCompensation, SGP40_SetSampleInterval,
 *   SGP40_PrimeForVocIndex, SGP40_MeasureVOCIndex,
 *   SGP40_GetVOCCategory, SGP40_VOCCategoryToString,
 *   SGP40_IsVOCWarmupComplete, SGP40_GetVOCWarmupSamples,
 *   SGP40_StatusToString
 *
 * @note
 * - Si AHT20 ne répond plus, SGP40 continue avec la dernière compensation T/RH
 *   valide (les valeurs figées sont conservées dans le handle).
 * - SGP40_DeInit() : appeler avant bootloader, test unitaire ou après
 *   SGP40_MAX_CONSECUTIVE_ERRORS erreurs consécutives pour réinitialiser le handle.
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define SGP40_DEBUG_ENABLE              /* Active SGP40_StatusToString pour cet exemple */
#define AHT20_DEBUG_ENABLE              /* Active AHT20_StatusToString pour cet exemple */
#include "STM32_SGP40.h"  // Driver SGP40 — mesure VOC raw + VOC Index avec compensation T/RH
#include "STM32_AHT20.h"  // Driver AHT20 — source T/RH pour la compensation SGP40
#include <stdio.h>        // pour printf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_NAME       "exemple_sgp40_polling_aht20_compensation"  ///< Nom pour identification dans les logs
#define LOOP_PERIOD_MS 1000U                                        ///< Intervalle de mesure = cadence algorithme VOC (ms)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
SGP40_HandleTypeDef hsgp40;  ///< Handle principal du capteur SGP40
AHT20_HandleTypeDef haht20;  ///< Handle du capteur AHT20 (source T/RH de compensation)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Retransmet un caractère via UART pour redirection stdout (printf).
  * @param  ch  Caractère à transmettre.
  * @retval Caractère transmis.
  */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);  // Envoi bloquant vers UART2 (console debug 115200 bauds)
    return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();

  /* USER CODE BEGIN 2 */
  /* 1) Bannière exemple */
  printf("\r\n========================================\r\n");
  printf("  Fichier: " LOG_NAME "\r\n");                        // Identification de l'exemple dans les logs
  printf("  SGP40 - Polling + compensation AHT20\r\n");
  printf("========================================\r\n\r\n");

  /* 2) Initialisation des handles */
  printf("Initialisation AHT20...\r\n");
  AHT20_Status aht_status = AHT20_Init(&haht20, &hi2c3);          // Init AHT20 — vérifie la calibration interne et l'adresse I2C 0x38
  if (aht_status != AHT20_OK) {
    printf("ERREUR  AHT20_Init: %s\r\n", AHT20_StatusToString(aht_status));
    Error_Handler();
  }
  printf("OK  AHT20 initialisé\r\n\r\n");

  printf("Initialisation SGP40...\r\n");
  SGP40_Status sgp_status = SGP40_Init(&hsgp40, &hi2c3);           // Init SGP40 — lit serial number, configure timeout/adresse/compensation par défaut
  if (sgp_status != SGP40_OK) {
    printf("ERREUR  SGP40_Init: %s\r\n", SGP40_StatusToString(sgp_status));
    Error_Handler();
  }
  printf("OK  SGP40 initialisé\r\n");
  printf("   Serial Number: 0x%04lX%08lX\r\n\r\n",
      (unsigned long)((hsgp40.serial_number >> 32) & 0xFFFFu),
      (unsigned long)(hsgp40.serial_number & 0xFFFFFFFFu));

  // SetSampleInterval et PrimeForVocIndex APRÈS Init() — Init() écrase les champs avec les valeurs par défaut
  sgp_status = SGP40_SetSampleInterval(&hsgp40, LOOP_PERIOD_MS);   // Cadence algorithmique = intervalle entre mesures (1 Hz requis par l'algo VOC Index)
  if (sgp_status != SGP40_OK) {
    printf("ERREUR  SGP40_SetSampleInterval: %s\r\n", SGP40_StatusToString(sgp_status));
    Error_Handler();
  }

  sgp_status = SGP40_PrimeForVocIndex(&hsgp40);                    // Prime l'algorithme VOC — effectue une mesure de chauffe et jette la première valeur
  if (sgp_status != SGP40_OK) {
    printf("ERREUR  SGP40_PrimeForVocIndex: %s\r\n", SGP40_StatusToString(sgp_status));
    Error_Handler();
  }

  /* 3) Paramètres runtime de la boucle principale */
  const uint32_t warmup_samples = SGP40_GetVOCWarmupSamples(&hsgp40);  // Durée du blackout algorithme (45 éch. par défaut, configurable)
  uint32_t warmup_count         = 0U;    // Compteur mesures phase warmup
  uint32_t nominal_count        = 0U;    // Compteur mesures phase nominale — repart à 1 après warmup
  uint8_t  warmup_done_announced = 0U;   // Flag : 1 = message "Warmup terminé" déjà affiché
  uint8_t  sgp_error_count      = 0U;    // Compteur d'erreurs SGP40 consécutives
  float    latest_temp_c        = 25.0f; // Dernière T°C valide lue sur AHT20 (défaut 25°C)
  float    latest_rh_pct        = 50.0f; // Dernier RH% valide lu sur AHT20 (défaut 50%)

  printf("Note: phase warmup algorithme VOC = %lu secondes\r\n\r\n", warmup_samples);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    AHT20_Data aht_data = {0};
    uint16_t voc_raw   = 0U;
    uint16_t voc_index = 0U;

    /* 1) Lecture AHT20 — mise à jour de la compensation T/RH */
    aht_status = AHT20_ReadMeasurements(&haht20, &aht_data);           // Mesure T/RH — ~10ms bloquant (AHT20 temps de conversion)
    if (aht_status == AHT20_OK) {
      latest_temp_c = aht_data.temperature;                            // Mémorise la dernière T°C valide
      latest_rh_pct = aht_data.humidity;                               // Mémorise le dernier RH% valide
      (void)SGP40_SetCompensation(&hsgp40, latest_temp_c, latest_rh_pct);  // Injecte T/RH dans le calcul VOC — retour ignoré (toujours SGP40_OK)
    } else {
      printf("INFO  AHT20 lecture: %s (compensation figée à T=%.1f°C RH=%.1f%%)\r\n",
             AHT20_StatusToString(aht_status), latest_temp_c, latest_rh_pct);  // Dégradation de service — VOC continue avec dernières valeurs T/RH connues
    }

    /* 2) Mesure VOC avec compensation T/RH courante */
    sgp_status = SGP40_MeasureVOCIndex(&hsgp40, &voc_raw, &voc_index);  // Mesure bloquante ~30ms — calcule VOC raw + VOC Index via algorithme Sensirion
    if (sgp_status != SGP40_OK) {
      sgp_error_count++;
      printf("ERREUR  Erreur mesure [%u/%u]: %s\r\n",
             sgp_error_count, SGP40_MAX_CONSECUTIVE_ERRORS,
             SGP40_StatusToString(sgp_status));                          // Affiche cause et progression vers le seuil d'arrêt sécurisé
      if (sgp_error_count >= SGP40_MAX_CONSECUTIVE_ERRORS) {
        printf("ERREUR  Erreur I2C répétée (capteur débranché ?)\r\n");
        Error_Handler();                                                  // Seuil atteint — arrêt sécurisé (configurable via SGP40_MAX_CONSECUTIVE_ERRORS)
      }
      HAL_Delay(LOOP_PERIOD_MS);
      continue;
    }
    sgp_error_count = 0U;                                                // Réinitialise le compteur après une mesure SGP40 réussie

    /* 3) Affichage selon phase warmup / nominal */
    if (!SGP40_IsVOCWarmupComplete(&hsgp40)) {
      warmup_count++;
      printf("[%5lu] ENV T=%6.2f°C RH=%5.1f%% | GAS VOC=%5u raw=%5u | Non disponible (warmup)\r\n",
             warmup_count, latest_temp_c, latest_rh_pct, voc_index, voc_raw);
      printf("   Etat algo : warmup (%lu/%lu)\r\n", warmup_count, warmup_samples);  // Progression du blackout (45 éch.)
    } else {
      if (!warmup_done_announced) {
        nominal_count = 0U;                                              // Remet le compteur nominal à 0 à la transition
        warmup_done_announced = 1U;
        printf("INFO  Warmup terminé, relevés remis à zéro\r\n");      // Transition warmup → nominal — première valeur VOC Index valide
      }
      nominal_count++;
      SGP40_VOC_Category cat = SGP40_GetVOCCategory(voc_index);          // Catégorise le VOC Index en 5 niveaux (EXCELLENT → UNHEALTHY)
      printf("[%5lu] ENV T=%6.2f°C RH=%5.1f%% | GAS VOC=%5u raw=%5u | %s\r\n",
             nominal_count, latest_temp_c, latest_rh_pct, voc_index, voc_raw,
             SGP40_VOCCategoryToString(cat));
    }

    HAL_Delay(LOOP_PERIOD_MS);   // Maintient la cadence 1 Hz — doit correspondre à SGP40_SetSampleInterval()
  }
  /* USER CODE END WHILE */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10D19CE4;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    for (volatile uint32_t wait = 0U; wait < 250000U; ++wait) {
      __NOP();
    }
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif

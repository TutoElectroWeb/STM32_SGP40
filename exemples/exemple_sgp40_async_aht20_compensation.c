/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : 'exemple_sgp40_async_aht20_compensation.c'
 * @brief          : [I2C IT] SGP40 async + AHT20 async pour compensation T/RH
 ******************************************************************************
 * @details
 * Exemple non-bloquant sur bus I2C partagé :
 * - AHT20 en async IT fournit température/humidité périodiques
 * - SGP40 en async IT calcule VOC index à 1 Hz
 * - Chaque mesure AHT20 met à jour SGP40_SetCompensation()
 *
 * Configuration matérielle :
 * - I2C choisi dans CubeMX, pull-up externes 4.7kΩ
 * - NVIC I2C EV/ER activés (obligatoire en mode IT)
 * - UART2 115200 pour console
 *
 * Compatible FreeRTOS :
 * ─────────────────────
 *  - SGP40_Init() et AHT20_Init() doivent être appelés avant vTaskStartScheduler()
 *    ou depuis une tâche d'init unique. Ne jamais appeler Init() depuis plusieurs
 *    tâches simultanément.
 *  - HAL_Delay() en FreeRTOS suspend la tâche (pas de spin-wait CPU) ✅
 *  - Les FSM SGP40 et AHT20 utilisent HAL_GetTick() uniquement → compatible.
 *  - Pour plusieurs tâches FreeRTOS accédant aux capteurs sur le même bus I2C :
 *    protéger les appels TriggerEvery/Tick par un mutex osMutexAcquire().
 *  - Les callbacks HAL (OnI2CMasterTxCplt, RxCplt, Error) s'exécutent en IRQ :
 *    ne jamais appeler osDelay() ou toute API FreeRTOS non ISR-safe depuis ces
 *    callbacks. Utiliser osSemaphoreRelease() ou osEventFlagsSet().
 *  - Les champs volatile des handles (async_busy) sont lisibles de n'importe
 *    quelle tâche sans mutex (atomique 8 bits ARM Cortex-M).
 *****************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "STM32_SGP40.h"  // Driver SGP40 — mesure VOC raw + VOC Index avec compensation T/RH
#include "STM32_AHT20.h"  // Driver AHT20 — source T/RH pour la compensation SGP40
#include <stdio.h>        // pour printf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_NAME        "exemple_sgp40_async_aht20_compensation"  ///< Nom pour identification dans les logs
#define SGP40_PERIOD_MS 1000U                                      ///< Période de déclenchement SGP40 = cadence algorithme VOC (ms)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
SGP40_HandleTypeDef hsgp40;             ///< Handle principal du capteur SGP40
SGP40_Async_t sgp40_async;              ///< Contexte machine d'états async SGP40
AHT20_HandleTypeDef haht20;             ///< Handle du capteur AHT20 (source T/RH)
AHT20_Async aht20_async;               ///< Contexte machine d'états async AHT20

static uint32_t sgp40_last_trigger_ms = 0U;  ///< Horodatage du dernier déclenchement SGP40 (pour TriggerEvery)
static uint32_t aht20_last_trigger_ms = 0U;  ///< Horodatage du dernier déclenchement AHT20
static uint8_t  sgp40_error_count     = 0U;  ///< Compteur d'erreurs SGP40 consécutives
static uint8_t  aht20_error_count     = 0U;  ///< Compteur d'erreurs AHT20 consécutives (reset après recovery)

static float latest_temp_c = 25.0f;    ///< Dernière T°C valide lue sur AHT20 (défaut 25°C)
static float latest_rh_pct = 50.0f;    ///< Dernier RH% valide lu sur AHT20 (défaut 50%)
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
  printf("  SGP40 - Async IT + compensation AHT20\r\n");
  printf("========================================\r\n\r\n");

  /* 2) Initialisation des handles (sync) */
  printf("Initialisation AHT20...\r\n");
  AHT20_Status aht_status = AHT20_Init(&haht20, &hi2c3);          // Init AHT20 — vérifie la calibration interne, adresse I2C 0x38
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
  uint64_t serial_num = 0U;                                                       // Récupération du numéro de série via l'API publique
  (void)SGP40_GetSerialNumber(&hsgp40, &serial_num);
  printf("   Serial Number: 0x%04lX%08lX\r\n\r\n",
      (unsigned long)((serial_num >> 32) & 0xFFFFu),
      (unsigned long)(serial_num & 0xFFFFFFFFu));

  // SetSampleInterval et PrimeForVocIndex APRÈS Init() — Init() écrase les champs avec les valeurs par défaut
  sgp_status = SGP40_SetSampleInterval(&hsgp40, SGP40_PERIOD_MS);  // Cadence algorithmique = période de déclenchement async (1 Hz requis)
  if (sgp_status != SGP40_OK) {
    printf("ERREUR  SGP40_SetSampleInterval: %s\r\n", SGP40_StatusToString(sgp_status));
    Error_Handler();
  }

  sgp_status = SGP40_PrimeForVocIndex(&hsgp40);                    // Prime l'algorithme VOC — effectue une mesure de chauffe et jette la première valeur
  if (sgp_status != SGP40_OK) {
    printf("ERREUR  SGP40_PrimeForVocIndex: %s\r\n", SGP40_StatusToString(sgp_status));
    Error_Handler();
  }

  /* 3) Initialisation des contextes asynchrones */
  SGP40_Async_Init(&sgp40_async, &hsgp40);    // Associe le contexte async SGP40 au handle sync — à appeler APRÈS SGP40_Init()
  AHT20_Async_Init(&aht20_async, &haht20);    // Associe le contexte async AHT20 au handle sync

  /* 4) Premier déclenchement — validation MX (I2C/NVIC EV/ER) */
  sgp_status = SGP40_ReadAll_IT(&sgp40_async);   // Lance la 1re transaction IT — valide que I2C + NVIC EV/ER sont actifs
  if (sgp_status != SGP40_OK && sgp_status != SGP40_ERR_BUSY) {
    if (sgp_status == SGP40_ERR_NOT_CONFIGURED) {
      printf("ERREUR  Configuration MX incomplète (I2C/NVIC) pour mode IT\r\n");
      printf("   Action: vérifier I2C choisi + IRQ EV/ER puis regénérer\r\n");
    } else {
      printf("ERREUR  Trigger initial SGP40: %s\r\n", SGP40_StatusToString(sgp_status));
    }
    Error_Handler();
  }

  aht_status = AHT20_ReadAll_IT(&aht20_async);   // Lance la 1re transaction IT AHT20 — même validation MX
  if (aht_status != AHT20_OK && aht_status != AHT20_ERR_BUSY) {
    if (aht_status == AHT20_ERR_NOT_CONFIGURED) {
      printf("ERREUR  Configuration MX incomplète (I2C/NVIC) pour mode IT\r\n");
      printf("   Action: vérifier I2C choisi + IRQ EV/ER puis regénérer\r\n");
    } else {
      printf("ERREUR  Trigger initial AHT20: %s\r\n", AHT20_StatusToString(aht_status));
    }
    Error_Handler();
  }

  sgp40_last_trigger_ms = HAL_GetTick();   // Mémorise l'heure du 1er déclenchement — TriggerEvery n'en redéclenchera un qu'après SGP40_PERIOD_MS
  aht20_last_trigger_ms = HAL_GetTick();   // Idem AHT20

  printf("INFO  Premières mesures lancées (SGP40 + AHT20)...\r\n");
  printf("   Validation MX (I2C/NVIC EV/ER) effectuée par la librairie\r\n\r\n");

  /* 5) Paramètres runtime de la boucle principale */
  const uint32_t warmup_samples  = SGP40_GetVOCWarmupSamples(&hsgp40);  // Durée du blackout algorithme (45 éch. par défaut)
  uint32_t warmup_count          = 0U;   // Compteur mesures phase warmup
  uint32_t nominal_count         = 0U;   // Compteur mesures phase nominale — repart à 1 après warmup
  uint8_t  warmup_done_announced = 0U;   // Flag : 1 = message "Warmup terminé" déjà affiché

  printf("Note: phase warmup algorithme VOC = %lu secondes\r\n\r\n", warmup_samples);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();                                          // Horodatage unique partagé entre les deux TriggerEvery

    (void)SGP40_Async_TriggerEvery(&sgp40_async, now, &sgp40_last_trigger_ms);  // Déclenche SGP40 si IDLE et intervalle écoulé — sans bloquer
    (void)AHT20_Async_TriggerEvery(&aht20_async, now, &aht20_last_trigger_ms);  // Idem AHT20 (cadence indépendante)

    /* AHT20 : tick et mise à jour compensation T/RH */
    AHT20_Data aht_data = {0};
    AHT20_TickResult aht_tick = AHT20_Async_Tick(&aht20_async, now, &aht_data);  // Avance la FSM AHT20 — retourne TICK_DATA_READY si trame complète
    if (aht_tick == AHT20_TICK_DATA_READY) {
      latest_temp_c = aht_data.temperature;                                // Mémorise T°C pour les logs et la compensation SGP40
      latest_rh_pct = aht_data.humidity;                                   // Mémorise RH%
      aht20_error_count = 0U;                                              // Réinitialise le compteur après succès
      (void)SGP40_SetCompensation(&hsgp40, latest_temp_c, latest_rh_pct); // Injecte T/RH dans le calcul VOC — retour ignoré (toujours SGP40_OK)
    } else if (aht_tick == AHT20_TICK_ERROR) {
      aht20_error_count++;
      printf("ERREUR  AHT20 async [%u/%u]\r\n",
             aht20_error_count, AHT20_MAX_CONSECUTIVE_ERRORS);  // Cause I2C — compensation figée aux dernières valeurs valides
      if (aht20_error_count >= AHT20_MAX_CONSECUTIVE_ERRORS) {
        printf("INFO  Recovery AHT20 (DeInit + Init + Async_Init)\r\n");   // Tentative de réinitialisation avant de continuer
        AHT20_DeInit(&haht20);                                             // Remet le handle AHT20 à zéro
        if (AHT20_Init(&haht20, &hi2c3) == AHT20_OK) {
          AHT20_Async_Init(&aht20_async, &haht20);                         // Réassocie le contexte async
          aht20_error_count = 0U;
        }
      }
    }

    /* SGP40 : tick et affichage VOC Index */
    uint16_t voc_raw   = 0U;
    uint16_t voc_index = 0U;
    SGP40_TickResult sgp_tick = SGP40_Async_TickIndex(&sgp40_async, now, &voc_raw, &voc_index);  // Avance la FSM SGP40

    if (sgp_tick == SGP40_TICK_DATA_READY) {
      sgp40_error_count = 0U;                                              // Réinitialise compteur après succès

      if (!SGP40_IsVOCWarmupComplete(&hsgp40)) {
        warmup_count++;
        printf("[%5lu] ENV T=%6.2f°C RH=%5.1f%% | GAS VOC=%5u raw=%5u | Non disponible (warmup)\r\n",
               warmup_count, latest_temp_c, latest_rh_pct, voc_index, voc_raw);
        printf("   Etat algo : warmup (%lu/%lu)\r\n", warmup_count, warmup_samples);  // Progression du blackout (45 éch.)
      } else {
        if (!warmup_done_announced) {
          nominal_count = 0U;                                              // Remet le compteur nominal à 0 à la transition
          warmup_done_announced = 1U;
          printf("INFO  Warmup terminé, relevés remis à zéro\r\n");       // Première valeur VOC Index valide disponible
        }
        nominal_count++;
        SGP40_VOC_Category cat = SGP40_GetVOCCategory(voc_index);          // Catégorise en 5 niveaux (EXCELLENT → UNHEALTHY)
        printf("[%5lu] ENV T=%6.2f°C RH=%5.1f%% | GAS VOC=%5u raw=%5u | %s\r\n",
               nominal_count, latest_temp_c, latest_rh_pct, voc_index, voc_raw,
               SGP40_VOCCategoryToString(cat));
      }
    } else if (sgp_tick == SGP40_TICK_ERROR) {
      sgp40_error_count++;
      printf("ERREUR  SGP40 async [%u/%u]\r\n",
             sgp40_error_count, SGP40_MAX_CONSECUTIVE_ERRORS);  // Cause I2C — compteur vers seuil d'arrêt sécurisé
      if (sgp40_error_count >= SGP40_MAX_CONSECUTIVE_ERRORS) {
        printf("ERREUR  Erreur I2C répétée (capteur débranché ?)\r\n");
        Error_Handler();                                                    // Seuil atteint — arrêt sécurisé (configurable via SGP40_MAX_CONSECUTIVE_ERRORS)
      }
    }

    HAL_Delay(2U);   // Cède le CPU 2 ms — évite la saturation bus I2C entre deux vérifications Tick
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  SGP40_DeInit(&hsgp40);  /* Jamais atteint en nominal — utile bootloader / tests unitaires */
  AHT20_DeInit(&haht20);
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

/* USER CODE BEGIN 4 */
/* Chaque lib filtre par son hi2c interne — appeler TOUTES les libs sans if externe.
   L'arbitrage HAL_BUSY TX → IDLE est géré dans chaque lib ; l'ordre d'appel est sans importance. */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  SGP40_Async_OnI2CMasterTxCplt(&sgp40_async, hi2c);  // SGP40 filtre par son hi2c interne
  AHT20_Async_OnI2CMasterTxCplt(&aht20_async, hi2c);  // AHT20 filtre par son hi2c interne
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  SGP40_Async_OnI2CMasterRxCplt(&sgp40_async, hi2c);
  AHT20_Async_OnI2CMasterRxCplt(&aht20_async, hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  SGP40_Async_OnI2CError(&sgp40_async, hi2c);
  AHT20_Async_OnI2CError(&aht20_async, hi2c);
}
/* USER CODE END 4 */

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

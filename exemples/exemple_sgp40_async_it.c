/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : 'exemple_sgp40_async_it.c'
 * @brief          : [I2C IT] Exemple SGP40 asynchrone non-bloquant
 ******************************************************************************
 * @details
 * Cet exemple illustre l'utilisation asynchrone du SGP40 en mode interruptions
 * I2C (IT). La boucle principale pilote la machine d'état et traite les flags
 * de fin de mesure/erreur remontés par les callbacks.
 *
 * Opérations principales :
 * - Initialisation du capteur et du contexte async
 * - Liaison des callbacks applicatifs (main-loop et IRQ)
 * - Déclenchement des mesures via SGP40_ReadAll_IT()
 * - Traitement périodique via SGP40_Async_Process()
 * - Gestion explicite des erreurs et reprise d'acquisition
 *
 * Configuration matérielle :
 * - I2C choisi dans CubeMX : mode standard/fast avec pull-up externes 4.7kΩ
 * - UART2 : console debug 115200 bauds
 * - NVIC : IRQ I2C EV/ER du bus choisi activés
 *
 * @note
 * - Les callbacks HAL I2C sont implémentés dans USER CODE BEGIN 4.
 * - Les routines IRQ restent courtes (flags/notification), le traitement est fait
 *   dans la boucle principale.
 * - Aucun ajout n'est nécessaire dans stm32l4xx_it.c pour cet exemple.
 * - API couverte : SGP40_Init, SGP40_DeInit, SGP40_SetCompensation,
 *   SGP40_SetSampleInterval, SGP40_PrimeForVocIndex, SGP40_CalculateVOCIndex,
 *   SGP40_GetVOCCategory, SGP40_VOCCategoryToString, SGP40_StatusToString,
 *   SGP40_Async_Init, SGP40_Async_Reset, SGP40_Async_SetCallbacks,
 *   SGP40_Async_SetIrqCallbacks, SGP40_ReadAll_IT, SGP40_Async_Process,
 *   SGP40_Async_DataReadyFlag, SGP40_Async_ErrorFlag, SGP40_Async_ClearFlags,
 *   SGP40_Async_HasData, SGP40_Async_GetData, SGP40_Async_IsIdle,
 *   SGP40_Async_OnI2CMasterTxCplt, SGP40_Async_OnI2CMasterRxCplt, SGP40_Async_OnI2CError.
 * - SGP40_Async_TickIndex reste disponible comme helper compact.
 *   Pour un exemple multi-capteurs sur bus I2C partagé, voir exemple_sgp40_async_multi_capteurs.c.
 *
 * @note Compatibilité FreeRTOS
 * - SGP40_Init() et SGP40_Async_Init() DOIVENT être appelés avant vTaskStartScheduler(),
 *   ou depuis une tâche d'init dédiée (exécutée une seule fois au démarrage).
 * - SGP40_Async_Process() et SGP40_Async_TriggerEvery() peuvent être appelés depuis
 *   n'importe quelle tâche, mais le bus I2C partagé nécessite un mutex applicatif
 *   si plusieurs tâches accèdent à des capteurs différents sur le même hi2c.
 * - Les callbacks HAL (OnI2CMasterTxCplt, OnI2CMasterRxCplt, OnI2CError) sont appelés
 *   depuis le contexte IRQ — ne jamais appeler osDelay(), osMutexAcquire() ou toute
 *   fonction FreeRTOS non ISR-safe depuis ces callbacks.
 * - Les champs volatile du handle (async_busy, consecutive_errors) sont sûrs en lecture
 *   depuis n'importe quelle tâche sans mutex (accès atomique 8 bits sur ARM Cortex-M).
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define SGP40_DEBUG_ENABLE              /* Active SGP40_StatusToString pour cet exemple */
#include "STM32_SGP40.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_NAME "exemple_sgp40_async_it"  ///< Nom pour identification dans les logs
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
SGP40_HandleTypeDef hsgp40;                 ///< Handle principal du capteur SGP40
SGP40_Async_t sgp40_async;                  ///< Contexte de machine d'état asynchrone
static uint8_t error_count = 0U;            ///< Compteur d'erreurs consécutives
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);

/* USER CODE BEGIN PFP */
static void SGP40_App_OnDataReady(void *user_ctx, uint16_t voc_raw, SGP40_Status status);
static void SGP40_App_OnError(void *user_ctx, SGP40_Status status);
static void SGP40_App_OnIrqDataReady(void *user_ctx);
static void SGP40_App_OnIrqError(void *user_ctx);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF); // Pour Envoyer le caractère via UART
    // ITM_SendChar(ch);                 // Option alternative pour envoyer le caractère via ITM
    return ch;
}

static void SGP40_App_OnDataReady(void *user_ctx, uint16_t voc_raw, SGP40_Status status)
{
  (void)user_ctx;
  (void)voc_raw;
  (void)status;
}

static void SGP40_App_OnError(void *user_ctx, SGP40_Status status)
{
  (void)user_ctx;
  (void)status;
}

static void SGP40_App_OnIrqDataReady(void *user_ctx)
{
  (void)user_ctx;
}

static void SGP40_App_OnIrqError(void *user_ctx)
{
  (void)user_ctx;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  MX_USART2_UART_Init();
  MX_I2C3_Init();

  /* USER CODE BEGIN 2 */
  /* 1) Bannière exemple */
  printf("\r\n========================================\r\n");     // Ligne de séparation
  printf("  Fichier: " LOG_NAME "\r\n");                          // Identification de l'exemple
  printf("  SGP40 - Async IT (non-bloquant)\r\n");                // Titre du programme
  printf("========================================\r\n\r\n");     // Ligne de séparation

  /* 2) Initialisation du handle SGP40 */
  // SGP40_Init() configure tous les champs avec les valeurs par défaut :
  // adresse 0x59, timeout 100ms, T=25°C / RH=50%, cadence 1000ms, algo VOC, prime silencieux.
  SGP40_Status status = SGP40_Init(&hsgp40, &hi2c3);                                                       // Initialise le capteur et lit son serial
  if (status != SGP40_OK) {                                                     // Vérifie le succès de l'initialisation capteur
      printf("ERREUR  Erreur init: %s\r\n", SGP40_StatusToString(status));  // Affiche l'erreur d'initialisation
      Error_Handler();
  }

  // Optionnel : compensation T/RH réelle si un 2e capteur (AHT20, BME280...) est disponible.
  // À appeler APRÈS Init() — Init() écrase les champs avec les défauts.
  (void)SGP40_SetCompensation(&hsgp40, 25.0f, 50.0f);                 // Ici : valeurs par défaut (identique Init), remplacer par mesure réelle

  status = SGP40_SetSampleInterval(&hsgp40, 1000U);                                       // Cadence de l'algorithme VOC à 1 Hz
  if (status != SGP40_OK) {                                                          // Vérifie le succès de la configuration de cadence
    printf("ERREUR  Erreur intervalle: %s\r\n", SGP40_StatusToString(status));   // Affiche l'erreur de cadence
    Error_Handler();
  }

  status = SGP40_PrimeForVocIndex(&hsgp40);                                               // Prime l'algorithme avant les mesures asynchrones
  if (status != SGP40_OK) {                                                          // Vérifie le succès de la phase de prime VOC
    printf("ERREUR  Erreur prime VOC: %s\r\n", SGP40_StatusToString(status));    // Affiche l'erreur de prime algorithme
    Error_Handler();
  }

  printf("OK  SGP40 initialisé\r\n");                                 // Confirme l'initialisation réussie
    printf("   Serial: 0x%04lX%08lX\r\n\r\n",                          // Affiche le serial capteur
      (unsigned long)((hsgp40.serial_number >> 32) & 0xFFFFu),
      (unsigned long)(hsgp40.serial_number & 0xFFFFFFFFu));

  /* 3) Initialisation du contexte asynchrone */
  SGP40_Async_Init(&sgp40_async, &hsgp40);                                       // Initialise la machine d'état async
  SGP40_Async_SetCallbacks(&sgp40_async, SGP40_App_OnDataReady, SGP40_App_OnError, NULL);
  SGP40_Async_SetIrqCallbacks(&sgp40_async, SGP40_App_OnIrqDataReady, SGP40_App_OnIrqError, NULL);
  SGP40_Async_ClearFlags(&sgp40_async);

  printf("Mode : Interruptions I2C (IT)\r\n");                                    // Décrit le mode de transfert utilisé
  printf("   Mesure toutes les 1 seconde\r\n");                                   // Informe la cadence cible
  printf("   Compatible multi-lib IT : bus partagé sans blocage mutuel\r\n");    // Mise en avant de la compatibilité multi-lib
  printf("   Horloge logicielle : SysTick (HAL_GetTick / HAL_Delay)\r\n");     // Documente la base de temps utilisée
  printf("   Callbacks HAL I2C inclus dans USER CODE BEGIN 4\r\n");              // Informe l'emplacement des callbacks
  printf("   Validation MX (I2C/IRQ) effectuée par la librairie\r\n\r\n");      // Précise que les contrôles de config sont centralisés dans la lib

  HAL_Delay(1000U);                                                               // Laisse la console/capteur se stabiliser

  /* 5) Paramètres runtime de la boucle asynchrone */
  const uint32_t warmup_samples = SGP40_GetVOCWarmupSamples(&hsgp40);                          // Durée de warmup en échantillons
  uint32_t warmup_count = 0U;                                                           // Compteur de mesures pendant blackout/warmup
  uint32_t nominal_count = 0U;                                                          // Compteur de mesures nominales (remis à zéro après blackout)
  uint8_t warmup_done_announced = 0U;                                                   // Empêche de répéter l'annonce de fin warmup
  printf("Note: phase warmup algorithme VOC = %lu secondes\r\n\r\n", warmup_samples);   // Informe la durée de warmup

  /* 6) Premier déclenchement manuel pour valider la configuration (NVIC/I2C) */
  uint32_t last_trigger = HAL_GetTick();
  SGP40_Status trig_status = SGP40_ReadAll_IT(&sgp40_async);

  if (trig_status != SGP40_OK) {
      if (trig_status == SGP40_ERR_NOT_CONFIGURED) {
          printf("ERREUR  Configuration MX incomplète (I2C/IRQ) pour mode IT\r\n");
          printf("   Action: vérifier I2C choisi + IRQ EV/ER puis régénérer\r\n");
      } else {
          printf("ERREUR  Erreur trigger: %s\r\n", SGP40_StatusToString(trig_status));
      }
      Error_Handler();
  }
  printf("INFO  Première mesure lancée...\r\n\r\n");

  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)                                                            // Boucle principale infinie de supervision async IT
  {
      uint32_t now = HAL_GetTick();                                    // Temps courant système en ms via SysTick

      /* Trigger périodique 1s — TriggerEvery ne déclenche que si FSM IDLE.
       * Si une autre lib async IT (STM32_AHT20, STM32_BME280...) occupe le bus,
       * HAL_BUSY est reçu → FSM reste IDLE → retry automatique au cycle suivant.
       * SGP40_Async_IsIdle() permet de vérifier l'état avant un trigger manuel. */
      if (SGP40_Async_IsIdle(&sgp40_async)) {
          SGP40_Async_TriggerEvery(&sgp40_async, now, &last_trigger);
      }

      /* Avance FSM, lit flags async et consomme les données via API explicite. */
      SGP40_Async_Process(&sgp40_async, now);           /* void depuis v0.9 — statut via ctx->last_status */

      if (SGP40_Async_ErrorFlag(&sgp40_async)) {
        error_count++;
        SGP40_Async_ClearFlags(&sgp40_async);
        SGP40_Async_Reset(&sgp40_async);   // Remet FSM à IDLE (async_busy=0)

        if (error_count >= SGP40_MAX_CONSECUTIVE_ERRORS) {
          // Seuil configurable via STM32_SGP40_conf.h. Réinitialiser le handle.
          printf("ERREUR  Erreur I2C repetee [%u/%u] — DeInit + redemarrage\r\n",
                 error_count, SGP40_MAX_CONSECUTIVE_ERRORS);
          SGP40_DeInit(&hsgp40);
          Error_Handler();
        } else {
          printf("ERREUR  Erreur async [%u/%u] -> reset FSM\r\n",
                 error_count, SGP40_MAX_CONSECUTIVE_ERRORS);
        }
      }

      if (SGP40_Async_DataReadyFlag(&sgp40_async) || SGP40_Async_HasData(&sgp40_async)) {
        uint16_t voc_raw = 0U;
        uint16_t voc_index = 0U;
        SGP40_Status get_status = SGP40_Async_GetData(&sgp40_async, &voc_raw);
        SGP40_Async_ClearFlags(&sgp40_async);

        if (get_status == SGP40_OK && SGP40_CalculateVOCIndex(&hsgp40, voc_raw, &voc_index) == SGP40_OK) {
          error_count = 0U;

          if (!SGP40_IsVOCWarmupComplete(&hsgp40)) {
            warmup_count++;
            printf("[#%lu] OK  VOC raw=%u | index=%u | Non disponible (warmup)\r\n",
                   warmup_count, voc_raw, voc_index);
            printf("Etat algo  : warmup (%lu/%lu)\r\n", warmup_count, warmup_samples);
          } else {
            if (!warmup_done_announced) {
              nominal_count = 0U;
              printf("INFO  Warmup terminé, relevés remis à zéro\r\n");
              warmup_done_announced = 1U;
            }
            nominal_count++;
            SGP40_VOC_Category cat = SGP40_GetVOCCategory(voc_index);
            printf("[#%lu] OK  VOC raw=%u | index=%u | %s\r\n",
                   nominal_count, voc_raw, voc_index, SGP40_VOCCategoryToString(cat));
          }
        }
      }

      /* Zone applicative non bloquante — autres capteurs async IT sur le même bus :
       * appeler leurs Tick() ici, les callbacks se dispatchent automatiquement.
       * Ex: AHT20_Async_TriggerEvery(&haht20_async, now, &last_aht20);  (intervalle via haht20_ctx.sample_interval_ms)
       *     AHT20_Async_TickIndex(&haht20_async, now, &temp_raw, &temp_c);
       */

      /* Simulation d'une tâche applicative bloquante (ex: envoi réseau, calcul lourd) */
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay(1000U); // Délai bloquant de 1s pour prouver que l'IT tourne en tâche de fond
    
  /* USER CODE END WHILE */
  }
  /* USER CODE BEGIN 3 */

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
  /* USER CODE BEGIN I2C3_Init 0 */
  /* USER CODE END I2C3_Init 0 */
  /* USER CODE BEGIN I2C3_Init 1 */
  /* USER CODE END I2C3_Init 1 */
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
  /* USER CODE BEGIN I2C3_Init 2 */
  /* USER CODE END I2C3_Init 2 */
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */
  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* =============================================================================
 * Callbacks HAL I2C — Dispatch vers toutes les libs async IT du bus
 * =============================================================================
 *
 * Chaque fonction STM32_xxx_Async_OnI2Cxxx() filtre par son handle hi2c
 * interne → appeler TOUTES les libs sans if externe.
 *
 * Pour ajouter une 2e lib async IT sur le même bus I2C :
 *   1. Déclarer son contexte async en variable globale (comme sgp40_async)
 *   2. Appeler sa fonction On*Cplt dans chacun des 3 callbacks ci-dessous
 *   3. Appeler son TriggerEvery() + TickIndex() dans le while(1)
 *
 * Les libs STM32_xxx ne se bloqueront jamais mutuellement : si le bus est
 * occupé (HAL_BUSY), leur FSM reste IDLE et retente au cycle suivant.
 * =============================================================================*/

/**
 * @brief Callback HAL — transfert I2C Tx terminé.
 * @note  Appelé depuis IRQ I2C3_EV après que HAL_I2C_Master_Transmit_IT réussit.
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  SGP40_Async_OnI2CMasterTxCplt(&sgp40_async, hi2c);         // SGP40  — filtre hi2c en interne
  // STM32_AHT20_Async_OnI2CMasterTxCplt(&haht20_async, hi2c);     // AHT20  — même pattern (décommenter si utilisé)
  // STM32_BME280_Async_OnI2CMasterTxCplt(&hbme280_async, hi2c);   // BME280 — même pattern (décommenter si utilisé)
}

/**
 * @brief Callback HAL — transfert I2C Rx terminé.
 * @note  Appelé depuis IRQ I2C3_EV après que HAL_I2C_Master_Receive_IT réussit.
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  SGP40_Async_OnI2CMasterRxCplt(&sgp40_async, hi2c);         // SGP40  — filtre hi2c en interne
  // STM32_AHT20_Async_OnI2CMasterRxCplt(&haht20_async, hi2c);     // AHT20  — même pattern (décommenter si utilisé)
  // STM32_BME280_Async_OnI2CMasterRxCplt(&hbme280_async, hi2c);   // BME280 — même pattern (décommenter si utilisé)
}

/**
 * @brief Callback HAL — erreur I2C (NACK, arbitration loss, timeout bus...).
 * @note  Appelé depuis IRQ I2C3_ER.
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  SGP40_Async_OnI2CError(&sgp40_async, hi2c);                 // SGP40  — filtre hi2c en interne
  // STM32_AHT20_Async_OnI2CError(&haht20_async, hi2c);       // AHT20  — même pattern (décommenter si utilisé)
  // STM32_BME280_Async_OnI2CError(&hbme280_async, hi2c);     // BME280 — même pattern (décommenter si utilisé)
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();                                                        // Désactive les interruptions avant la boucle d'erreur
    while (1)                                                               // Boucle d'erreur bloquante
    {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);                           // Fait clignoter la LED d'erreur
      for (volatile uint32_t wait = 0U; wait < 100000U; ++wait) {          // Temporisation locale sans HAL_Delay
        __NOP();                                                            // Occupation CPU minimale pour espacer le clignotement
      }
    }
    
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

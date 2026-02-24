/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : 'exemple_sgp40_polling.c'
 * @brief          : [LectureSimple] Exemple SGP40 lecture simple VOC raw et index
 ******************************************************************************
 * @details
 * Cet exemple présente le chemin d'utilisation le plus simple du driver SGP40 :
 * initialisation, mesure brute ponctuelle et conversion en VOC Index.
 *
 * Opérations principales :
 * - Initialisation du capteur avec paramètres de compensation par défaut
 * - Lecture périodique de la mesure brute (VOC raw)
 * - Calcul et affichage du VOC Index
 * - Affichage des statuts pour diagnostic rapide
 * - Arrêt propre via SGP40_DeInit() en cas d'erreurs répétées
 *
 * Configuration matérielle :
 * - I2C choisi dans CubeMX : mode standard/fast avec pull-up externes 4.7kΩ
 * - UART2 : console debug 115200 bauds
 *
 * @note
 * - Exemple recommandé comme point d'entrée pour valider l'intégration matérielle.
 * - API couverte : SGP40_Init, SGP40_DeInit, SGP40_SetSampleInterval,
 *   SGP40_SetCompensation, SGP40_PrimeForVocIndex, SGP40_MeasureVOCIndex,
 *   SGP40_IsVOCWarmupComplete, SGP40_GetVOCWarmupSamples, SGP40_GetVOCCategory,
 *   SGP40_VOCCategoryToString, SGP40_StatusToString.
 * - SGP40_DeInit() : appeler avant bootloader, en test unitaire, ou après
 *   SGP40_MAX_CONSECUTIVE_ERRORS erreurs consécutives pour réinitialiser le handle.
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "STM32_SGP40.h"  // le SGP40 driver
#include <stdio.h>        // pour printf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_NAME "exemple_sgp40_polling"  ///< Nom pour identification dans les logs
/* #define SGP40_DEBUG_ENABLE */  ///< Décommenter pour activer les traces textuelles — laisser commenté en production
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
SGP40_HandleTypeDef hsgp40;  ///< Handle principal du capteur SGP40
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
  * @brief  Affiche un en-tête visuel pour une phase de l'exemple.
  * @param  title Titre à afficher sur la console UART.
  * @retval None
  */
static void PrintPhase(const char *title) {
  printf("-----------------------\r\n");     // Ligne de séparation
  printf("%s\r\n", title);                   // Titre du programme
  printf("-----------------------\r\n");     // Ligne de séparation
}

/**
  * @brief  Retransmet un caractère via UART pour redirection stdout (printf).
  * @param  ch  Caractère à transmettre.
  * @retval Caractère transmis.
  */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF); // Envoi bloquant vers UART2 (console debug 115200 bauds)
    return ch;
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
  printf("\r\n========================================\r\n");   // Ligne de séparation
  printf("  Fichier: " LOG_NAME "\r\n");                        // Identification de l'exemple dans les logs
  printf("  SGP40 - Lecture simple (VOC Index)\r\n");           // Titre du programme
  printf("========================================\r\n\r\n");   // Ligne de séparation

  /* 2) Initialisation du handle SGP40 */
  // SGP40_Init() configure tous les champs avec les valeurs par défaut :
  // adresse 0x59, timeout 100ms, T=25°C / RH=50%, cadence 1000ms, algo VOC, prime silencieux.

    PrintPhase("Phase 1 - Initialisation");                                               // Titre de la phase d'initialisation

    /* Initialisation SGP40 */
    printf("Initialisation SGP40...\r\n");                                                // Message de début d'initialisation
    SGP40_Status status = SGP40_Init(&hsgp40, &hi2c3);                                                          // Appel de la fonction d'initialisation
    if (status != SGP40_OK) {                                                        // Vérification du statut de l'initialisation
        printf("ERREUR  Erreur init: %s\r\n", SGP40_StatusToString(status));     // Affichage du message d'erreur avec description du code d'erreur
        Error_Handler();                                                                  // Appel du gestionnaire d'erreur (peut faire un reset ou boucle infinie)
    }

  // Optionnel : compensation T/RH réelle si un 2e capteur (AHT20, BME280...) est disponible.
  // À appeler APRÈS Init() — Init() écrase les champs avec les défauts.
  (void)SGP40_SetCompensation(&hsgp40, 22.0f, 50.0f);  // Exemple avec température ambiante réelle (remplacer par mesure capteur T/RH)

    printf("OK  SGP40 initialisé\r\n");                                                   // Message de succès d'initialisation
    uint64_t serial_num = 0U;                                                               // Récupération du numéro de série via l'API publique (pas d'accès direct au champ)
    (void)SGP40_GetSerialNumber(&hsgp40, &serial_num);
        printf("   Serial Number: 0x%04lX%08lX\r\n\r\n",                                // Affichage du numéro de série du capteur pour vérification matérielle
          (unsigned long)((serial_num >> 32) & 0xFFFFu),
          (unsigned long)(serial_num & 0xFFFFFFFFu));

  status = SGP40_SetSampleInterval(&hsgp40, 1000U);                                       // Configure la cadence de l'algorithme VOC à 1 Hz (1000 ms)
  if (status != SGP40_OK) {                                                          // Vérification du statut de la configuration
    printf("ERREUR  Erreur intervalle: %s\r\n", SGP40_StatusToString(status));   // Affichage du message d'erreur avec description du code d'erreur
    Error_Handler();                                                                      // Appel du gestionnaire d'erreur en cas d'échec de la configuration
  }

  status = SGP40_PrimeForVocIndex(&hsgp40);                                               // Effectue la prime algorithmique avant les mesures pour stabiliser l'index VOC
  if (status != SGP40_OK) {                                                          // Vérification du statut de la prime algorithmique
    printf("ERREUR  Erreur prime VOC: %s\r\n", SGP40_StatusToString(status));    // Affichage du message d'erreur avec description du code d'erreur
    Error_Handler();                                                                      // Appel du gestionnaire d'erreur en cas d'échec de la prime algorithmique
  }

  HAL_Delay(1000U);                                                                        // Pause de 1 seconde avant de commencer les mesures pour laisser le temps au capteur de se stabiliser

  /* 3) Paramètres runtime de la boucle principale */
  uint32_t warmup_count = 0U;                                                             // Compteur de mesures pendant blackout/warmup
  uint32_t nominal_count = 0U;                                                            // Compteur de mesures nominales (remis à zéro après blackout)
  const uint32_t warmup_samples = SGP40_GetVOCWarmupSamples(&hsgp40);                              // Nombre d'échantillons de warmup de l'algorithme VOC
  uint8_t warmup_done_announced = 0U;                                                     // Flag pour indiquer si la fin du warmup a été annoncée
  uint8_t error_count = 0U;                                                               // Compteur applicatif d'erreurs consécutives (remis à 0 sur succès)

  PrintPhase("Phase 2 - Warmup algorithme VOC");                                          // Titre de la phase de warmup
  printf("Note: phase warmup algorithme VOC = %lu secondes\r\n\r\n", warmup_samples);     // Affichage du nombre d'échantillons de warmup
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)                                                                                         // Boucle principale infinie de mesure VOC
  {
        /* 1) Acquisition VOC raw + calcul VOC index */
        uint16_t voc_raw;                                                                           // Variable pour stocker la mesure brute VOC
        uint16_t voc_index;                                                                         // Variable pour stocker le VOC Index calculé
        status = SGP40_MeasureVOCIndex(&hsgp40, &voc_raw, &voc_index);                              // Mesure raw + calcul VOC index en un seul appel

        if (status != SGP40_OK) {                                                                              // Vérification du statut de la mesure
            error_count++;                                                                                        // Compteur applicatif d'erreurs consécutives
            printf("ERREUR  Erreur mesure [%u/%u]: %s\r\n\r\n",
                   error_count, SGP40_MAX_CONSECUTIVE_ERRORS,
                   SGP40_StatusToString(status));     // Affichage du message d'erreur avec description du code d'erreur
            if (error_count >= SGP40_MAX_CONSECUTIVE_ERRORS) {
                // Seuil d'erreurs consécutives atteint (configurable via STM32_SGP40_conf.h).
                // SGP40_DeInit() remet le handle à zéro avant d'appeler Error_Handler().
                // En production : appeler SGP40_Init() pour tenter une réinitialisation.
                printf("ERREUR  Erreur I2C repetee (capteur debranche ?)\r\n");
                SGP40_DeInit(&hsgp40);
                Error_Handler();
            }
            HAL_Delay(1000U);                                                                      // Pause de 1 seconde avant la prochaine tentative de mesure
            continue;                                                                              // Passe à l'itération suivante de la boucle
        }

        /* 2) Affichage résultats et état warmup */
        error_count = 0U;                                                                             // Succès : remet le compteur d'erreurs à zéro
        printf("-----------------------\r\n");                                                        // Ligne de séparation pour chaque mesure
        printf("VOC Raw    : %u (0-65535)\r\n", voc_raw);                                           // Affichage de la mesure brute VOC
        printf("VOC Index  : %u (0-500)\r\n", voc_index);                                           // Affichage du VOC Index calculé

        if (!SGP40_IsVOCWarmupComplete(&hsgp40)) {                                                  // Vérifie si on est encore dans la phase de warmup de l'algorithme VOC
          warmup_count++;                                                                            // Incrémente les relevés warmup
          printf("Mesure warmup #%lu\r\n", warmup_count);                                          // Affichage du numéro de mesure warmup
          printf("Etat algo  : warmup (%lu/%lu)\r\n", warmup_count, warmup_samples);               // Affichage de l'état de l'algorithme (warmup en cours avec nombre d'échantillons collectés)
          printf("Qualité    : Non disponible (warmup)\r\n");                                       // Affichage de la qualité de l'air non disponible pendant le warmup
        } else {                                                                                    // Si le warmup est terminé
          if (!warmup_done_announced) {                                                             // Vérifie si la fin du warmup a déjà été annoncée
          nominal_count = 0U;                                                                       // Remise à zéro des relevés nominaux après blackout
          PrintPhase("Phase 3 - Mesures nominales");                                                // Titre de la phase de mesures nominales
          printf("INFO  Warmup terminé, relevés remis à zéro\r\n");                                // Affichage d'information indiquant le reset des relevés
          warmup_done_announced = 1U;                                                               // Met à jour le flag pour indiquer que la fin du warmup a été annoncée
          }

          nominal_count++;                                                                          // Incrémente les relevés nominaux
          printf("Mesure nominale #%lu\r\n", nominal_count);                                       // Affichage du numéro de mesure nominale

          /* Catégorisation qualité air */
          SGP40_VOC_Category category = SGP40_GetVOCCategory(voc_index);                            // Appel de la fonction de catégorisation de la qualité de l'air à partir du VOC Index
          printf("Qualité    : %s\r\n", SGP40_VOCCategoryToString(category));                       // Affichage de la qualité de l'air sous forme de chaîne de caractères
        }

        printf("-----------------------\r\n");                                                        // Ligne de séparation pour chaque mesure

        printf("\r\n");                                                                             // Ligne vide pour séparation visuelle entre les mesures

        /* 3) Cadence fixe à 1 Hz */
        HAL_Delay(1000U);                                                                            // Pause de 1 seconde pour maintenir une cadence de mesure de 1 Hz
    
  /* USER CODE END WHILE */
  }
  /* USER CODE BEGIN 3 */
  SGP40_DeInit(&hsgp40);  /* Jamais atteint en nominal — utile bootloader / tests unitaires */
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
/* Callbacks HAL (IRQ/DMA/IT) et hooks runtime à placer ici si nécessaire. */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* Handler d'erreur bloquant: LED clignotante pour diagnostic visuel. */
    __disable_irq();
    while (1)  // Boucle d'erreur bloquante
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

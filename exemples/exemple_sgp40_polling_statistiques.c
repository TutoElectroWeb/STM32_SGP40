/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : 'exemple_sgp40_polling_statistiques.c'
 * @brief          : [Analyse] Exemple SGP40 VOC Index avec statistiques
 ******************************************************************************
 * @details
 * Cet exemple illustre une exploitation continue des mesures SGP40 avec calcul
 * du VOC Index et accumulation de statistiques (min/max/moyenne et répartition
 * par catégorie de qualité d'air).
 *
 * Opérations principales :
 * - Initialisation capteur et lecture d'identité
 * - Mesure VOC raw puis conversion VOC Index
 * - Classification de la qualité d'air
 * - Mise à jour et affichage des statistiques agrégées
 *
 * Configuration matérielle :
 * - I2C choisi dans CubeMX : mode standard/fast avec pull-up externes 4.7kΩ
 * - UART2 : console debug 115200 bauds
 *
 * @note
 * - Exemple pédagogique pour interprétation et tendance des données VOC.
 * - API couverte : SGP40_Init, SGP40_GetSerialNumber, SGP40_SetCompensation,
 *   SGP40_SetSampleInterval, SGP40_PrimeForVocIndex, SGP40_MeasureVOCIndex,
 *   SGP40_IsVOCWarmupComplete, SGP40_GetVOCWarmupSamples, SGP40_GetVOCCategory,
 *   SGP40_VOCCategoryToString, SGP40_StatusToString.
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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint16_t voc_index_min;
    uint16_t voc_index_max;
    uint16_t voc_index_avg;
    uint32_t count;
    uint32_t count_excellent;
    uint32_t count_good;
    uint32_t count_moderate;
    uint32_t count_poor;
    uint32_t count_unhealthy;
} VOC_Stats;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_NAME "exemple_sgp40_voc_statistiques"  ///< Nom pour identification dans les logs
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
SGP40_HandleTypeDef hsgp40;  ///< Handle principal du capteur SGP40
VOC_Stats stats = {0};       ///< Structure globale d'agrégation statistique
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
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF); // Pour Envoyer le caractère via UART
    // ITM_SendChar(ch);                 // Option alternative pour envoyer le caractère via ITM
    return ch;
}

/**
  * @brief  Met à jour les statistiques VOC avec un nouvel index.
  * @param  voc_index Valeur VOC index à intégrer aux statistiques.
  * @retval None
  */
void UpdateStats(uint16_t voc_index) {
  stats.count++;                                                                                // Incrémente le nombre total de mesures intégrées

  if (stats.count == 1) {                                                                       // Pour la première mesure, initialise min/max/moyenne avec cette valeur
    stats.voc_index_min = voc_index;                                                            // Initialise le minimum avec la première mesure
    stats.voc_index_max = voc_index;                                                            // Initialise le maximum avec la première mesure
    stats.voc_index_avg = voc_index;                                                            // Initialise la moyenne avec la première mesure
    } else {                                                                                    // Pour les mesures suivantes, met à jour min/max/moyenne
    if (voc_index < stats.voc_index_min) stats.voc_index_min = voc_index;                       // Met à jour le minimum si nécessaire
    if (voc_index > stats.voc_index_max) stats.voc_index_max = voc_index;                       // Met à jour le maximum si nécessaire
 
    stats.voc_index_avg = (stats.voc_index_avg * (stats.count - 1) + voc_index) / stats.count;  // Recalcule la moyenne incrémentale (Moyenne mobile (évite overflow sur count élevés))
    }

    // Incrémente compteurs par catégorie
  SGP40_VOC_Category cat = SGP40_GetVOCCategory(voc_index);     // Détermine la catégorie qualité pour cet index
    switch (cat) {                                              // Incrémente le compteur de la catégorie correspondante
    case SGP40_VOC_EXCELLENT:  stats.count_excellent++; break;  // Compte un échantillon en qualité excellente
    case SGP40_VOC_GOOD:       stats.count_good++; break;       // Compte un échantillon en qualité bonne
    case SGP40_VOC_MODERATE:   stats.count_moderate++; break;   // Compte un échantillon en qualité modérée
    case SGP40_VOC_POOR:       stats.count_poor++; break;       // Compte un échantillon en qualité mauvaise
    case SGP40_VOC_UNHEALTHY:  stats.count_unhealthy++; break;  // Compte un échantillon en qualité insalubre
    default: break;                                             // Ignore les catégories non prévues
    }
}

/**
  * @brief  Affiche les statistiques VOC accumulées sur la console.
  * @retval None
  */
void PrintStats(void) {
  printf("\r\n========== Statistiques VOC Index ==========\r\n");                                                             // Affiche l'en-tête du bloc statistiques
  printf("Mesures totales : %lu\r\n", stats.count);                                                                           // Affiche le nombre total de mesures
  printf("VOC Index MIN   : %u\r\n", stats.voc_index_min);                                                                    // Affiche le minimum observé
  printf("VOC Index MAX   : %u\r\n", stats.voc_index_max);                                                                    // Affiche le maximum observé
  printf("VOC Index AVG   : %u\r\n", stats.voc_index_avg);                                                                    // Affiche la moyenne observée
  printf("\r\nRépartition qualité air :\r\n");                                                                                // Introduit la répartition par catégorie
  printf("  Excellent  : %lu (%.1f%%)\r\n", stats.count_excellent, 100.0f * stats.count_excellent / stats.count);  // Part des mesures excellentes
  printf("  Bon        : %lu (%.1f%%)\r\n", stats.count_good, 100.0f * stats.count_good / stats.count);                  // Part des mesures bonnes
  printf("  Modéré     : %lu (%.1f%%)\r\n", stats.count_moderate, 100.0f * stats.count_moderate / stats.count);       // Part des mesures modérées
  printf("  Mauvais    : %lu (%.1f%%)\r\n", stats.count_poor, 100.0f * stats.count_poor / stats.count);              // Part des mesures mauvaises
  printf("  Insalubre  : %lu (%.1f%%)\r\n", stats.count_unhealthy, 100.0f * stats.count_unhealthy / stats.count);  // Part des mesures insalubres
  printf("============================================\r\n\r\n");                                                             // Ferme visuellement le bloc
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
  printf("\r\n==========================================\r\n");       // Ligne de séparation
  printf("  Fichier: " LOG_NAME "\r\n");                              // Identification de l'exemple
  printf("  SGP40 - VOC Index avec Statistiques\r\n");                // Titre du programme
  printf("==========================================\r\n\r\n");       // Ligne de séparation

  /* 2) Configuration du handle SGP40 */
  hsgp40.i2c_timeout = SGP40_DEFAULT_TIMEOUT_MS;  // Timeout I2C en ms

  /* 3) Initialisation du capteur */
  printf("Initialisation SGP40...\r\n");                                            // Message de démarrage de l'init
  SGP40_Status status = SGP40_Init(&hsgp40, &hi2c3);                                                      // Initialisation complète du capteur
  if (status != SGP40_OK) {                                                    // Vérifie le succès de l'initialisation capteur
    printf("ERREUR  Erreur init: %s\r\n", SGP40_StatusToString(status));   // Affiche l'erreur textuelle
    Error_Handler();                                                                // Stop en cas d'échec critique
  }

  printf("OK  SGP40 initialisé\r\n");                                   // Confirme l'initialisation réussie
    printf("   Serial Number: 0x%04lX%08lX\r\n",                         // Affiche le serial capteur pour traçabilité
      (unsigned long)((hsgp40.serial_number >> 32) & 0xFFFFu),
      (unsigned long)(hsgp40.serial_number & 0xFFFFFFFFu));

  /* 4) Compensation T/RH (valeurs fixes de démo) */
  SGP40_SetCompensation(&hsgp40, 22.5f, 45.0f);                                               // Applique une compensation fixe de démonstration
  printf("   Compensation : T=%.1f°C, RH=%.1f%%\r\n\r\n", hsgp40.temp_c, hsgp40.rh_percent);  // Affiche la compensation appliquée

  /* 5) Cadence officielle de l'algorithme VOC: 1 Hz */
  status = SGP40_SetSampleInterval(&hsgp40, 1000U);                                         // Cadence officielle: 1 échantillon/seconde
  if (status != SGP40_OK) {                                                            // Vérifie le succès de la configuration de cadence
    printf("ERREUR  Erreur intervalle: %s\r\n", SGP40_StatusToString(status));     // Affiche l'erreur textuelle
    Error_Handler();                                                                        // Stop en cas d'échec de configuration de cadence
  }

  /* 6) Prime algorithmique avant la boucle */
  status = SGP40_PrimeForVocIndex(&hsgp40);                            // Prime l'algorithme VOC avant la première boucle
  if (status != SGP40_OK) {  // Vérifie le succès de la phase de prime VOC
    printf("ERREUR  Erreur prime VOC: %s\r\n", SGP40_StatusToString(status));
    Error_Handler();
  }

  /* 7) Stabilisation UART/terminal */
  HAL_Delay(1000U);                                                      // Laisse la console/capteur se stabiliser

  /* 8) Paramètres runtime de la boucle principale */
  uint32_t last_stats = HAL_GetTick();                                   // Horodatage de la dernière impression des statistiques
  uint32_t warmup_count = 0U;                                            // Compteur de mesures pendant la phase warmup/blackout
  uint32_t nominal_count = 0U;                                           // Compteur de relevés en régime nominal (redémarre à zéro après blackout)
  uint8_t warmup_done_announced = 0U;                                   // Empêche d'afficher 2 fois le message fin de warmup
  const uint32_t warmup_samples = SGP40_GetVOCWarmupSamples(&hsgp40);                   // Durée de warmup en échantillons

  printf("Note: phase warmup algorithme VOC = %lu secondes\r\n\r\n", warmup_samples);  // Informe la durée de warmup

  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)  // Boucle principale infinie de mesure et statistiques
  {
    /* 1) Acquisition VOC raw + calcul VOC index */
    uint16_t voc_raw;
    uint16_t voc_index;
    status = SGP40_MeasureVOCIndex(&hsgp40, &voc_raw, &voc_index);                // Mesure raw + calcul VOC index en un seul appel
    static uint8_t error_count = 0U;
    if (status != SGP40_OK) {  // Contrôle l'erreur de mesure/calcul
      error_count++;
      printf("ERREUR  Erreur: %s\r\n", SGP40_StatusToString(status));    // Affiche l'erreur
      if (error_count >= 2U) {
          printf("ERREUR  Erreur I2C répétée (capteur débranché ?)\r\n");
          Error_Handler();
      }
      HAL_Delay(1000U);                                                           // Attend 1 seconde avant nouvelle tentative
      continue;                                                                   // Repart à l'itération suivante
    } else {
      error_count = 0U;
    }

    /* 2) Gestion warmup puis affichage métier */
    if (!SGP40_IsVOCWarmupComplete(&hsgp40)) {  // Sépare la phase warmup de la phase nominale
            warmup_count++;  // Compte uniquement les acquisitions de la phase blackout/warmup
            printf("[%5lu] VOC=%5u raw=%5u | Non disponible (warmup)\r\n",  // Affiche la mesure pendant la phase warmup
             warmup_count, voc_index, voc_raw);
            printf("Etat algo  : warmup (%lu/%lu)\r\n", warmup_count, warmup_samples);  // Affiche l'avancement du warmup
    } else {
      if (!warmup_done_announced) {  // N'annonce la fin de warmup qu'une seule fois
        memset(&stats, 0, sizeof(stats));                                // Remise à zéro explicite des statistiques après blackout
        nominal_count = 0U;                                              // Remise à zéro explicite du compteur des relevés nominaux
        last_stats = HAL_GetTick();                                      // Recale la fenêtre d'impression périodique des stats
        printf("INFO  Warmup terminé, relevés/statistiques remis à zéro\r\n");  // Annonce le passage en régime nominal
        warmup_done_announced = 1U;                                     // Marque l'annonce comme déjà faite
      }

      nominal_count++;                                                   // Compte uniquement les relevés après blackout
      SGP40_VOC_Category category = SGP40_GetVOCCategory(voc_index);    // Catégorie qualitative de l'index VOC
      UpdateStats(voc_index);                                           // Met à jour min/max/moyenne et répartitions
      printf("[%5lu] VOC=%5u raw=%5u | %s\r\n",                        // Affiche la mesure nominale avec sa catégorie
             nominal_count, voc_index, voc_raw,
             SGP40_VOCCategoryToString(category));
    }

    /* 3) Affichage stats toutes les 30 secondes */
    if ((stats.count > 0U) && (HAL_GetTick() - last_stats >= 30000U)) {  // Affiche périodiquement les statistiques consolidées
      PrintStats();                                                      // Imprime les statistiques consolidées
      last_stats = HAL_GetTick();                                        // Réarme la fenêtre des 30 secondes
    }

    /* 4) Cadence fixe à 1 Hz */
    HAL_Delay(1000U);                                                    // Maintient la fréquence d'acquisition à 1 Hz
    
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
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();                                                        // Désactive les interruptions avant la boucle d'erreur
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

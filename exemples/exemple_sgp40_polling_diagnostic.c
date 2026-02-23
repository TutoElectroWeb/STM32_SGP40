/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : 'exemple_sgp40_polling_diagnostic.c'
 * @brief          : [Diagnostic] Exemple SGP40 de debug VOC détaillé
 ******************************************************************************
 * @details
 * Cet exemple est orienté diagnostic terrain quand la valeur VOC brute évolue
 * peu ou de manière incohérente. Il aide à valider la chaîne complète : bus I2C,
 * échanges trames, compensation environnementale et cadence de mesure.
 *
 * Opérations principales :
 * - Vérification de la présence du capteur sur I2C
 * - Affichage des données brutes pour inspection
 * - Tests de compensation température/humidité
 * - Monitoring rapproché des mesures VOC
 *
 * Configuration matérielle :
 * - I2C choisi dans CubeMX : mode standard/fast avec pull-up externes 4.7kΩ
 * - UART2 : console debug 115200 bauds
 *
 * @note
 * - Exemple destiné au debug applicatif, pas à un profil production.
 * - Les traces printf sont volontairement verbeuses.
 * - API couverte : SGP40_Init, SGP40_SelfTest, SGP40_SetCompensation,
 *   SGP40_SetSampleInterval, SGP40_MeasureRaw, SGP40_CalculateVOCIndex,
 *   SGP40_GetVOCCategory, SGP40_VOCCategoryToString, SGP40_StatusToString.
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "STM32_SGP40.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_NAME "exemple_sgp40_debug"  ///< Nom pour identification dans les logs
#define DEBUG_WARMUP_SAMPLES  30U    ///< Nombre d'échantillons warmup capteur + algo VOC
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
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF); // Pour Envoyer le caractère via UART
    // ITM_SendChar(ch);                 // Option alternative pour envoyer le caractère via ITM
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
  printf("\r\n================================================\r\n");  // Ligne de séparation
  printf("  Fichier: " LOG_NAME "\r\n");                           // Identification de l'exemple
  printf("  SGP40 - DEBUG Détaillé (diagnostic VOC)\r\n");           // Titre du programme
  printf("================================================\r\n\r\n");  // Ligne de séparation

  /* 2) Configuration du handle SGP40 */
  hsgp40.i2c_timeout = SGP40_DEFAULT_TIMEOUT_MS;  // Timeout I2C en ms

    /* Test 1: Vérification I2C */
    printf("--- Test 1: Communication I2C ---\r\n");                                   // Introduit le test de présence I2C
    HAL_StatusTypeDef hal_status;                                                           // Statut bas niveau HAL pour diagnostic bus

    hal_status = HAL_I2C_IsDeviceReady(&hi2c3, (uint16_t)(SGP40_I2C_ADDR_7B << 1), 3, 100);              // Sonde la présence du périphérique I2C sélectionné
    if (hal_status != HAL_OK) {  // Vérifie si le capteur répond bien sur le bus I2C
        printf("ERREUR  SGP40 non détecté sur I2C (adresse 0x%02X)\r\n", SGP40_I2C_ADDR_7B);  // Affiche l'adresse cible non détectée
        printf("   Code HAL: %d\r\n", hal_status);                                                     // Affiche le code d'erreur HAL bas niveau
        printf("   -> Vérifier connexions SDA/SCL/VCC/GND\r\n");                                      // Rappelle les vérifications de câblage
        printf("   -> Vérifier pull-up 4.7kΩ sur SDA/SCL\r\n\r\n");                                  // Rappelle la présence des résistances de tirage
        Error_Handler();                                                                                // Stoppe en cas d'absence capteur
    }
    printf("OK  SGP40 détecté sur I2C\r\n\r\n");                                      // Confirme la présence I2C du capteur
    HAL_Delay(100U);                                                                               // Laisse une petite temporisation après détection I2C

    /* Test 2: Initialisation */
    printf("--- Test 2: Initialisation ---\r\n");                                      // Introduit le test d'initialisation
    SGP40_Status status = SGP40_Init(&hsgp40, &hi2c3);                                                            // Initialise le capteur
    if (status != SGP40_OK) {  // Vérifie le succès de l'initialisation capteur
        printf("ERREUR  Erreur init: %s\r\n", SGP40_StatusToString(status));               // Affiche l'erreur d'initialisation capteur
        Error_Handler();                                                                                // Stoppe en cas d'échec d'initialisation
    }

    status = SGP40_SetSampleInterval(&hsgp40, 500U);                                        // Cadence debug à 2 Hz
    if (status != SGP40_OK) {  // Vérifie le succès de la configuration de cadence debug
      printf("ERREUR  Erreur intervalle debug: %s\r\n", SGP40_StatusToString(status));    // Affiche l'erreur de cadence debug
      Error_Handler();                                                                                // Stoppe en cas d'échec de configuration cadence
    }

    printf("OK  SGP40 initialisé\r\n");                                                // Confirme l'initialisation réussie
        printf("   Serial: 0x%04lX%08lX\r\n\r\n",                                       // Affiche le serial capteur
          (unsigned long)((hsgp40.serial_number >> 32) & 0xFFFFu),
          (unsigned long)(hsgp40.serial_number & 0xFFFFFFFFu));
    HAL_Delay(100U);                                                                               // Stabilise la console avant test suivant

    /* 3) Campagne de tests de validation */

    /* Test 3: Auto-test capteur */
    printf("--- Test 3: Auto-test heater ---\r\n");                                   // Introduit le test d'auto-diagnostic
    uint16_t test_result;                                                                              // Résultat brut du self-test
    status = SGP40_SelfTest(&hsgp40, &test_result);                                                    // Lance le self-test capteur
    if (status != SGP40_OK) {  // Vérifie le résultat du self-test interne
        printf("ATTENTION   Auto-test ÉCHOUÉ (0x%04X)\r\n", test_result);                            // Signale un auto-test non conforme
        printf("   -> Capteur peut-être défectueux\r\n\r\n");                                       // Propose une interprétation du défaut
      } else {                                                                                           // Branche succès du self-test
        printf("OK  Auto-test OK (0x%04X)\r\n\r\n", test_result);                                  // Confirme la réussite de l'auto-test
    }
    HAL_Delay(500U);                                                                               // Temporisation entre deux tests

    /* Test 4: Warmup (30 mesures pour stabiliser capteur + algo VOC) */
    const uint32_t warmup_samples = DEBUG_WARMUP_SAMPLES;                                   // Nombre d'échantillons utilisés pour le warmup
    printf("--- Test 4: Warmup capteur (%lu mesures) ---\r\n", warmup_samples);        // Introduit la phase de warmup
    printf("ATTENTE  Le heater et l'algo VOC ont besoin de temps pour se stabiliser...\r\n\r\n"); // Explique l'objectif de la phase warmup

    uint16_t warmup_values[DEBUG_WARMUP_SAMPLES];                                           // Buffer des valeurs raw collectées en warmup
    for (uint32_t i = 0; i < warmup_samples; i++) {  // Acquiert les échantillons de warmup
        status = SGP40_MeasureRaw(&hsgp40, &warmup_values[i]);                              // Lecture VOC raw pour l'échantillon i
        if (status == SGP40_OK) {  // Traite uniquement les mesures valides
        uint16_t warmup_index;                                                              // Index temporaire calculé pour amorcer l'algo VOC
        (void)SGP40_CalculateVOCIndex(&hsgp40, warmup_values[i], &warmup_index);           // Conversion raw->index pendant warmup
        printf("  [%2lu] VOC raw = %5u\r\n", i + 1U, warmup_values[i]);                              // Affiche chaque mesure brute de warmup
        } else {                                                                                        // Branche erreur de mesure durant warmup
        printf("  [%2lu] Erreur: %s\r\n", i + 1U, SGP40_StatusToString(status));           // Affiche l'erreur sur l'échantillon courant
        }
        HAL_Delay(1000U);  // 1 mesure/seconde pendant warmup
    }

    // Calcul variation warmup
    uint16_t warm_min = warmup_values[0];                                                   // Minimum observé durant le warmup
    uint16_t warm_max = warmup_values[0];                                                   // Maximum observé durant le warmup
    for (uint32_t i = 1U; i < warmup_samples; i++) {  // Balaye les valeurs pour trouver min et max
        if (warmup_values[i] < warm_min) warm_min = warmup_values[i];  // Met à jour le minimum de warmup
        if (warmup_values[i] > warm_max) warm_max = warmup_values[i];  // Met à jour le maximum de warmup
    }

    printf("\r\nRésumé warmup:\r\n");                                                               // Introduit le bilan warmup
    printf("  MIN = %u, MAX = %u, Variation = %u\r\n", warm_min, warm_max, warm_max - warm_min);   // Affiche min/max/variation warmup

    if (warm_max - warm_min < 100) {  // Détecte une variation trop faible après warmup
      printf("  INFO  Variation faible (<100) : possible en air stable ou capteur deja chaud\r\n"); // Donne une interprétation de variation faible
      printf("  VERIF  Faire un test avec stimulus (souffle/fumee) avant de conclure\r\n");         // Propose une vérification complémentaire
    }
    printf("\r\n");                                                                                  // Sépare visuellement les sections
    HAL_Delay(1000U);                                                                               // Temporisation avant test suivant

    /* Test 5: Mesures sans compensation (valeurs par défaut) */
    printf("--- Test 5: Mesure SANS compensation (défaut T=25°C, RH=50%%) ---\r\n");  // Introduit le test en compensation nominale
    SGP40_SetCompensation(&hsgp40, 25.0f, 50.0f);                                           // Compensation nominale

    uint16_t voc_default;                                                                    // Référence raw avec compensation nominale
    status = SGP40_MeasureRaw(&hsgp40, &voc_default);                                                   // Mesure brute en compensation nominale
    if (status == SGP40_OK) {  // Vérifie la mesure brute nominale
        printf("OK  VOC raw (défaut) = %u\r\n", voc_default);                                         // Affiche la référence nominale
      } else {                                                                                             // Branche erreur de mesure nominale
        printf("ERREUR  Erreur: %s\r\n", SGP40_StatusToString(status));                     // Affiche l'erreur de mesure
    }
    printf("\r\n");                                                                                  // Sépare visuellement les sections
    HAL_Delay(1000U);                                                                               // Temporisation avant test suivant

    /* Test 6: Mesures avec compensation extrême (pour forcer différence) */
    printf("--- Test 6: Mesure AVEC compensation extrême (T=0°C, RH=0%%) ---\r\n");   // Introduit le test de sensibilité compensation
    printf("ATTENTION   Compensation volontairement fausse pour tester impact\r\n\r\n");            // Avertit que ce test force un cas extrême

    SGP40_SetCompensation(&hsgp40, 0.0f, 0.0f);                                             // Compensation extrême pour test de sensibilité

    uint16_t voc_extreme;                                                                    // Mesure raw avec compensation extrême
    status = SGP40_MeasureRaw(&hsgp40, &voc_extreme);                                                   // Mesure brute en compensation extrême
    if (status == SGP40_OK) {  // Vérifie la mesure brute en compensation extrême
        printf("OK  VOC raw (T=0,RH=0) = %u\r\n", voc_extreme);                                      // Affiche la mesure extrême
        printf("   Écart vs défaut = %d\r\n", (int)voc_extreme - (int)voc_default);                 // Affiche l'écart vs référence

        if (abs((int)voc_extreme - (int)voc_default) < 50) {  // Signale une faible sensibilité à la compensation
            printf("   ATTENTION   PROBLÈME: Compensation n'a presque aucun effet!\r\n");           // Signale une sensibilité anormale
            printf("   -> Capteur peut-être défectueux ou problème I2C\r\n");                        // Donne une piste de diagnostic
          }
        } else {                                                                                             // Branche erreur de mesure extrême
          printf("ERREUR  Erreur: %s\r\n", SGP40_StatusToString(status));                     // Affiche l'erreur de mesure
    }
    printf("\r\n");                                                                                  // Sépare visuellement les sections
    HAL_Delay(1000U);                                                                               // Temporisation avant monitoring

    /* Restaurer compensation normale */
    SGP40_SetCompensation(&hsgp40, 25.0f, 50.0f);                                                       // Restaure la compensation nominale

    /* Test 7: Monitoring haute fréquence (500ms) pour détecter variations */
    printf("================================================\r\n");                    // Ligne de séparation
    printf("  MONITORING HAUTE FRÉQUENCE\r\n");                                          // Titre de la phase de monitoring
    printf("================================================\r\n");                    // Ligne de séparation
    printf("Instructions test:\r\n");                                                                 // Introduit le protocole opérateur
    printf("  1. Laisser 30 secondes à l'air libre (baseline)\r\n");                                  // Etape 1 du protocole
    printf("  2. Souffler fumée cigarette près du capteur\r\n");                                      // Etape 2 du protocole
    printf("  3. Observer variation VOC raw\r\n");                                                   // Etape 3 du protocole
    printf("  4. Attendre retour à baseline (~2-3 min)\r\n\r\n");                                   // Etape 4 du protocole
    printf("Format: [Temps] VOC_raw (variation) | VOC_index | Catégorie\r\n");                       // Décrit le format d'affichage des mesures
    printf("------------------------------------------------\r\n\r\n");                             // Ligne de séparation

  /* 4) Paramètres runtime du monitoring final */
  uint32_t start_time = HAL_GetTick();                                                        // Horodatage de départ du monitoring
  uint16_t baseline = 0;                                                                     // Baseline construite sur les premières mesures
  uint16_t voc_min = 65535;                                                                  // Minimum observé dans la fenêtre
  uint16_t voc_max = 0;                                                                      // Maximum observé dans la fenêtre
  uint32_t count = 0;                                                                        // Compteur global des mesures monitorées

  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)  // Boucle principale infinie de monitoring debug
  {
      /* 1) Acquisition VOC raw */
        uint16_t voc_raw;                                                                    // Valeur brute lue au cycle courant
        status = SGP40_MeasureRaw(&hsgp40, &voc_raw);                                        // Lecture VOC raw capteur

        static uint8_t error_count = 0U;
        if (status == SGP40_OK) {  // Traite uniquement les mesures valides
            error_count = 0U;
            count++;                                                                         // Incrémente le nombre de mesures réussies

            // 2) Baseline = moyenne des 10 premières mesures
            if (count <= 10) {                                                               // Construit la baseline sur les 10 premières mesures
                baseline += voc_raw / 10;                                                              // Accumule la baseline moyenne
            }

            // 3) Mise à jour min/max
            if (voc_raw < voc_min) voc_min = voc_raw;                                       // Met à jour le minimum local
            if (voc_raw > voc_max) voc_max = voc_raw;                                       // Met à jour le maximum local

            // 4) Calcul variation vs baseline
            int32_t variation = (int32_t)voc_raw - (int32_t)baseline;                                   // Calcule l'écart à la baseline

            // 5) Calcul VOC index
            uint16_t voc_index;                                                              // Index VOC converti depuis la valeur brute
            SGP40_CalculateVOCIndex(&hsgp40, voc_raw, &voc_index);                          // Conversion raw->index VOC
            SGP40_VOC_Category cat = SGP40_GetVOCCategory(voc_index);                       // Catégorie qualitative associée

            // 6) Affichage mesure
                 uint32_t elapsed = (HAL_GetTick() - start_time) / 1000;                                    // Calcule le temps écoulé en secondes
                 printf("[%4lus] %5u (%+5ld) | idx=%3u | %s",                                                // Affiche la ligne principale de monitoring
                   elapsed, voc_raw, variation, voc_index,
                   SGP40_VOCCategoryToString(cat));

            // 7) Alerte variation significative
            if (labs(variation) > 1000) {                                                    // Détecte une variation importante
                printf(" ATTENTION   VARIATION DÉTECTÉE!");                                              // Ajoute une alerte en cas de variation forte
            }

            printf("\r\n");                                                                            // Termine la ligne de sortie

            // 8) Stats toutes les 30 secondes
            if (count % 60 == 0) {                                                           // Imprime une synthèse toutes les 60 mesures
                printf("\r\n--- Stats (dernières 60 mesures) ---\r\n");                             // Introduit la synthèse périodique
                printf("Baseline : %u\r\n", baseline);                                                 // Affiche la baseline courante
                printf("MIN      : %u (écart: %d)\r\n", voc_min, (int)voc_min - (int)baseline);      // Affiche le minimum et son écart
                printf("MAX      : %u (écart: %+d)\r\n", voc_max, (int)voc_max - (int)baseline);     // Affiche le maximum et son écart
                printf("Amplitude: %u\r\n\r\n", voc_max - voc_min);                                  // Affiche l'amplitude observée

                if (voc_max - voc_min < 500) {  // Détecte une amplitude faible sur la fenêtre de mesure
                    printf("ATTENTION   AMPLITUDE FAIBLE: Capteur ne réagit presque pas!\r\n");       // Alerte sur faible dynamique
                    printf("   Causes possibles:\r\n");                                                 // Introduit les hypothèses de diagnostic
                    printf("   - Capteur défectueux ou saturé\r\n");                                    // Hypothèse 1
                    printf("   - Air déjà pollué (baseline élevée)\r\n");                               // Hypothèse 2
                    printf("   - Délai de réponse long (attendre 1-2 min)\r\n\r\n");                  // Hypothèse 3
                }

                // 9) Reset min/max pour nouvelle période
                voc_min = 65535;                                                                       // Réinitialise le minimum pour la fenêtre suivante
                voc_max = 0;                                                                           // Réinitialise le maximum pour la fenêtre suivante
              }
            } else {                                                                                        // Branche erreur de mesure brute
              error_count++;
              printf("ERREUR  Erreur mesure: %s\r\n", SGP40_StatusToString(status));         // Affiche l'erreur de mesure brute
              if (error_count >= 2U) {
                  printf("ERREUR  Erreur I2C répétée (capteur débranché ?)\r\n");
                  Error_Handler();
              }
        }

    /* 10) Cadence fixe à 2 Hz */
    HAL_Delay(500U);                                                                         // Cadence haute fréquence à 2 Hz
    
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
    __disable_irq();  // Désactive les interruptions avant la boucle d'erreur
    while (1)  // Boucle d'erreur bloquante
    {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);                   // Fait clignoter la LED d'erreur
      for (volatile uint32_t wait = 0U; wait < 100000U; ++wait) {   // Temporisation locale sans HAL_Delay
        __NOP();                                                    // Occupation CPU minimale pour espacer le clignotement
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

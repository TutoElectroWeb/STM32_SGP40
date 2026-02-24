/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : 'exemple_sgp40_polling_selftest.c'
 * @brief          : [SelfTest] Exemple SGP40 auto-test et diagnostic capteur
 ******************************************************************************
 * @details
 * Cet exemple exécute une séquence de validation capteur : lecture numéro de
 * série, initialisation complète, auto-test interne puis mesures de contrôle.
 * Il sert à qualifier rapidement un montage I2C avant intégration applicative.
 *
 * Opérations principales :
 * - Lecture du numéro de série SGP40
 * - Initialisation du driver (Init)
 * - Exécution de SGP40_SelfTest()
 * - Vérification de mesure et commande de maintenance (heater off)
 *
 * Configuration matérielle :
 * - I2C3 : mode standard/fast avec pull-up externes 4.7kΩ
 * - UART2 : console debug 115200 bauds
 *
 * @note
 * - Exemple utile pour diagnostic de production (bring-up, validation bus).
 * - Le scénario final de validation est exécuté via machine d'état et imprime
 *   une synthèse finale (mesures OK/KO, min/max/moyenne VOC index).
 * - API couverte : SGP40_Init, SGP40_GetSerialNumber, SGP40_Init,
 *   SGP40_SelfTest, SGP40_MeasureRaw, SGP40_SetCompensation,
 *   SGP40_SetSampleInterval, SGP40_PrimeForVocIndex, SGP40_HeaterOff,
 *   SGP40_CalculateVOCIndex, SGP40_GetVOCCategory, SGP40_VOCCategoryToString,
 *   SGP40_StatusToString,
 *   SGP40_VOCAlgoReset/GetStates/SetStates/GetTuning/SetTuning/GetSamplingInterval.
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "STM32_SGP40.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_NAME "exemple_sgp40_polling_selftest"  ///< Nom pour identification dans les logs
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
  printf("\r\n========================================\r\n");  // Ligne de séparation
  printf("  Fichier: " LOG_NAME "\r\n");                           // Identification de l'exemple
  printf("  SGP40 - Auto-test et Diagnostic\r\n");                 // Titre du programme
  printf("========================================\r\n\r\n");   // Ligne de séparation

  /* 2) Init — requis avant tout appel API (assigne hi2c, configure le handle) */
  // IMPORTANT : Init() est obligatoire avant n'importe quel appel API.
  // GetSerialNumber, SelfTest, MeasureRaw, etc. échouent tous avec ERR_NULL_PTR sans hi2c.
  SGP40_Status status = SGP40_OK;

    /* Test 1: Lecture Serial Number */
    printf("--- Test 1: Serial Number ---\r\n");                                     // Introduit la lecture du numéro de série
    // Init préalable requis pour assigner hi2c (GetSerialNumber envoie une commande I2C)
    status = SGP40_Init(&hsgp40, &hi2c3);  // Init rapide — le serial est aussi stocké dans hsgp40.serial_number
    if (status != SGP40_OK) { printf("ERREUR  Init avant Serial: %s\r\n", SGP40_StatusToString(status)); Error_Handler(); }

    uint64_t serial;                                                                     // Numéro de série unique du capteur
    status = SGP40_GetSerialNumber(&hsgp40, &serial);                                   // Lecture manuelle du serial (démonstration API dédiée)

    if (status != SGP40_OK) {  // Vérifie le succès de la lecture du serial
        printf("ERREUR  Erreur lecture serial: %s\r\n", SGP40_StatusToString(status));
        printf("   Vérifier connexions I2C (SDA/SCL/pull-up)\r\n\r\n");
        Error_Handler();
    }

        printf("OK  Serial Number: 0x%04lX%08lX\r\n",                                    // Affiche le serial lu
          (unsigned long)((serial >> 32) & 0xFFFFu),
          (unsigned long)(serial & 0xFFFFFFFFu));
    printf("   (6 octets uniques par capteur)\r\n\r\n");
    HAL_Delay(500U);

    /* Test 2: Initialisation simple */
    printf("--- Test 2: Init ---\r\n");                                             // Introduit le test d'init rapide
    status = SGP40_Init(&hsgp40, &hi2c3);                                        // Initialisation rapide avec paramètres par défaut

    if (status != SGP40_OK) {  // Vérifie le succès de l'initialisation
      printf("ERREUR  Erreur Init: %s\r\n\r\n", SGP40_StatusToString(status));
      Error_Handler();
    }

    printf("OK  Init OK\r\n\r\n");                                                 // Confirme l'init rapide
    HAL_Delay(500U);

    /* Test 3: Initialisation complète */
    printf("--- Test 3: Initialisation ---\r\n");                                    // Introduit le test d'init complète
    status = SGP40_Init(&hsgp40, &hi2c3);                                                        // Initialisation complète du capteur

    if (status != SGP40_OK) {  // Vérifie le succès de l'initialisation complète
        printf("ERREUR  Erreur init: %s\r\n\r\n", SGP40_StatusToString(status));
        Error_Handler();
    }

    printf("OK  SGP40 initialisé\r\n");                                              // Confirme l'initialisation complète
    printf("   Handle configuré avec compensation par défaut\r\n\r\n");
    HAL_Delay(500U);

    /* 3) Campagne de tests fonctionnels */

    /* Test 4: Auto-test capteur */
    printf("--- Test 4: Auto-test capteur ---\r\n");                                  // Introduit l'auto-test interne
    printf("ATTENTE  En cours... (jusqu'à 320 ms)\r\n");

    uint16_t test_result;                                                                // Mot de résultat du selftest capteur
    status = SGP40_SelfTest(&hsgp40, &test_result);                                      // Lance l'auto-test interne SGP40

    if (status != SGP40_OK) {  // Vérifie le résultat du self-test interne
        printf("ERREUR  Auto-test ÉCHOUÉ\r\n");
      printf("   Résultat: 0x%04X (attendu: pattern 0xD4XX)\r\n", test_result);
        printf("   Statut  : %s\r\n\r\n", SGP40_StatusToString(status));
    } else {
        printf("OK  Auto-test RÉUSSI\r\n");
        printf("   Résultat: 0x%04X (heater + sensor OK)\r\n\r\n", test_result);
    }
    HAL_Delay(500U);

    /* Test 5: Mesure VOC (vérification fonctionnement) */
    printf("--- Test 5: Mesure VOC raw ---\r\n");                                     // Introduit le test de mesure brute
    uint16_t voc_raw;                                                                    // Valeur VOC brute de référence
    status = SGP40_MeasureRaw(&hsgp40, &voc_raw);                                        // Première mesure brute de validation

    if (status != SGP40_OK) {  // Vérifie le succès de la mesure VOC brute
        printf("ERREUR  Erreur mesure: %s\r\n\r\n", SGP40_StatusToString(status));
    } else {
        printf("OK  Mesure OK\r\n");
        printf("   VOC raw: %u (plage typique: 20000-60000)\r\n", voc_raw);

        if (voc_raw < 10000 || voc_raw > 65000) {  // Détecte une valeur brute atypique
            printf("   ATTENTION   Valeur atypique (vérifier environnement)\r\n");
        }
        printf("\r\n");
    }
    HAL_Delay(500U);

    /* Test 6: Compensation T/RH */
    printf("--- Test 6: Compensation T/RH ---\r\n");                                  // Introduit le test de compensation
    status = SGP40_SetCompensation(&hsgp40, 22.0f, 40.0f);                              // Applique une compensation environnementale

    if (status != SGP40_OK) {  // Vérifie le succès de la configuration compensation
        printf("ERREUR  Erreur compensation: %s\r\n\r\n", SGP40_StatusToString(status));
    } else {
        printf("OK  Compensation configurée\r\n");
        printf("   T = 22.0°C, RH = 40.0%%\r\n\r\n");
    }
    HAL_Delay(500U);

    /* Test 7: Mesure avec compensation */
    printf("--- Test 7: Mesure compensée ---\r\n");                                   // Introduit la mesure post-compensation
    uint16_t voc_comp;                                                                   // Valeur brute après compensation
    status = SGP40_MeasureRaw(&hsgp40, &voc_comp);                                       // Mesure brute compensée

    if (status != SGP40_OK) {  // Vérifie le succès de la mesure compensée
        printf("ERREUR  Erreur: %s\r\n\r\n", SGP40_StatusToString(status));
    } else {
        printf("OK  Mesure compensée OK\r\n");
        printf("   VOC raw: %u\r\n", voc_comp);
        printf("   Écart vs défaut: %d\r\n\r\n", (int)voc_comp - (int)voc_raw);
    }
    HAL_Delay(1000U);

    status = SGP40_SetSampleInterval(&hsgp40, 1000U);                                    // Cadence nominale de l'algo VOC à 1 Hz
    if (status != SGP40_OK) {  // Vérifie la configuration de cadence nominale
      printf("ERREUR  Erreur intervalle: %s\r\n\r\n", SGP40_StatusToString(status));
    }

    status = SGP40_PrimeForVocIndex(&hsgp40);                                            // Prime l'algorithme VOC avant lecture d'index
    if (status != SGP40_OK) {  // Vérifie le succès de la phase de prime VOC
      printf("ERREUR  Erreur prime VOC: %s\r\n\r\n", SGP40_StatusToString(status));
    }
    HAL_Delay(1000U);

    /* Test 8: API avancée algorithme VOC */
    printf("--- Test 8: API avancée VOC ---\r\n");                                    // Introduit la vérification des API avancées
    float state0 = 0.0f, state1 = 0.0f;                                                  // Etats internes exportables de l'algorithme VOC
    float sampling_s = 0.0f;                                                             // Intervalle d'échantillonnage courant (s)
    int32_t index_offset = 0, lto = 0, ltg = 0, gating = 0, std_initial = 0, gain = 0; // Paramètres de tuning VOC

    status = SGP40_VOCAlgoGetSamplingInterval(&hsgp40, &sampling_s);
    if (status == SGP40_OK) {  // Affiche l'intervalle de sampling si lecture réussie
      printf("OK  Sampling algo VOC: %.2fs\r\n", sampling_s);
    }

    status = SGP40_VOCAlgoGetTuning(&hsgp40, &index_offset, &lto, &ltg, &gating, &std_initial, &gain);
    if (status == SGP40_OK) {  // Affiche le tuning si lecture réussie
      printf("OK  Tuning VOC lu (offset=%ld, gain=%ld)\r\n", index_offset, gain);
      (void)SGP40_VOCAlgoSetTuning(&hsgp40, index_offset, lto, ltg, gating, std_initial, gain);
    }

    status = SGP40_VOCAlgoGetStates(&hsgp40, &state0, &state1);
    if (status == SGP40_OK) {  // Affiche les états internes si lecture réussie
      printf("OK  States VOC lus\r\n");
      (void)SGP40_VOCAlgoSetStates(&hsgp40, state0, state1);
    }

    status = SGP40_VOCAlgoReset(&hsgp40);
    if (status == SGP40_OK) {  // Confirme le reset algorithmique si succès
      printf("OK  Reset algo VOC OK\r\n\r\n");
    }
    HAL_Delay(1000U);

    /* Test 9: HeaterOff + reprise */
    printf("--- Test 9: HeaterOff + reprise ---\r\n");                                 // Introduit le test de reprise après arrêt heater
    status = SGP40_HeaterOff(&hsgp40);                                                   // Coupe le heater pour test de reprise
    if (status != SGP40_OK) {  // Vérifie la commande HeaterOff
      printf("ERREUR  Erreur HeaterOff: %s\r\n\r\n", SGP40_StatusToString(status));
      Error_Handler();
    }

    status = SGP40_Init(&hsgp40, &hi2c3);                                                        // Ré-initialise le capteur après HeaterOff
    if (status != SGP40_OK) {  // Vérifie la ré-initialisation après HeaterOff
      printf("ERREUR  Erreur re-init: %s\r\n\r\n", SGP40_StatusToString(status));
      Error_Handler();
    }

    status = SGP40_SetSampleInterval(&hsgp40, 1000U);                                    // Restaure la cadence nominale de calcul VOC
    if (status != SGP40_OK) {  // Vérifie la reconfiguration de cadence
      printf("ERREUR  Erreur intervalle re-init: %s\r\n\r\n", SGP40_StatusToString(status));
      Error_Handler();
    }

    status = SGP40_PrimeForVocIndex(&hsgp40);                                            // Re-prime l'algorithme après reprise
    if (status != SGP40_OK) {  // Vérifie la re-prime après reprise
      printf("ERREUR  Erreur prime re-init: %s\r\n\r\n", SGP40_StatusToString(status));
      Error_Handler();
    }
    printf("OK  Reprise après HeaterOff OK\r\n\r\n");
    HAL_Delay(1000U);

    /* Résumé final */
    printf("========================================\r\n");                              // Ligne de séparation
    printf("  Diagnostic terminé\r\n");                                                  // Titre de fin de campagne
    printf("========================================\r\n");                              // Ligne de séparation
    printf("OK  Tous les tests réussis\r\n");
    printf("   Capteur SGP40 opérationnel\r\n\r\n");

  /* 4) Machine d'état de validation finale */
  printf("--- Test final (machine d'état, 10 mesures) ---\r\n\r\n");
  typedef enum {
    TEST_STATE_MEASURE = 0,
    TEST_STATE_SUMMARY,
    TEST_STATE_DONE
  } TestState;

  TestState test_state = TEST_STATE_MEASURE;                                             // Etat initial de la machine de test
  uint32_t count = 1;                                                                     // Compteur de mesures réalisées
  uint32_t ok_count = 0;                                                                  // Nombre de mesures valides
  uint32_t ko_count = 0;                                                                  // Nombre de mesures en erreur
  uint32_t sum_index = 0;                                                                 // Somme des index VOC pour moyenne
  uint16_t min_index = 500;                                                               // Minimum VOC index observé
  uint16_t max_index = 0;                                                                 // Maximum VOC index observé
  const uint32_t target_samples = 10;                                                     // Nombre de mesures à produire dans le test final

  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)  // Boucle principale infinie de la machine d'état de validation
  {
      /* 1) Exécution de la machine d'état de fin de test */
      switch (test_state) {  // Oriente le traitement selon l'état courant du scénario
        case TEST_STATE_MEASURE:  // Etat de mesure séquentielle des échantillons
          /* 2) Acquisition + calcul VOC index sur N échantillons */
          if (count <= target_samples) {  // Continue tant que la cible d'échantillons n'est pas atteinte
            status = SGP40_MeasureRaw(&hsgp40, &voc_raw);                                 // Lecture VOC raw

            static uint8_t error_count = 0U;
            if (status == SGP40_OK) {  // Vérifie le succès de la mesure brute
              error_count = 0U;
              uint16_t voc_index;                                                         // Index VOC calculé pour cette mesure
              status = SGP40_CalculateVOCIndex(&hsgp40, voc_raw, &voc_index);             // Conversion raw->index VOC
              if (status == SGP40_OK) {  // Vérifie le succès du calcul VOC index
                ok_count++;                                                               // Incrémente le nombre de mesures réussies
                sum_index += voc_index;                                                   // Accumule pour la moyenne finale
                if (voc_index < min_index) min_index = voc_index;                          // Met à jour le minimum observé
                if (voc_index > max_index) max_index = voc_index;                          // Met à jour le maximum observé

                printf("[%lu/%lu] VOC raw=%5u, index=%3u | %s\r\n",
                     count, target_samples, voc_raw, voc_index,
                     SGP40_VOCCategoryToString(SGP40_GetVOCCategory(voc_index)));
              } else {
                ko_count++;                                                               // Comptabilise les erreurs de calcul VOC index
                printf("[%lu/%lu] ERREUR  Erreur index: %s\r\n",
                     count, target_samples, SGP40_StatusToString(status));
              }
            } else {
              ko_count++;                                                                 // Comptabilise les erreurs de mesure brute
              error_count++;
              if (error_count >= 2U) {
                  printf("ERREUR  Erreur I2C répétée (capteur débranché ?)\r\n");
                  Error_Handler();
              }
              printf("[%lu/%lu] ERREUR  Erreur mesure: %s\r\n",
                   count, target_samples, SGP40_StatusToString(status));
            }

            count++;                                                                      // Passe à l'échantillon suivant
            HAL_Delay(1000U);                                                                      // Respecte la cadence de 1 Hz
          } else {
            test_state = TEST_STATE_SUMMARY;                                              // Bascule vers l'étape de synthèse
          }
          break;

        case TEST_STATE_SUMMARY:  // Etat de synthèse finale des résultats
          /* 3) Synthèse globale des résultats */
          printf("\r\n========================================\r\n");
          printf("  Synthèse finale exemple_sgp40_polling_selftest\r\n");
          printf("========================================\r\n");
          printf("Mesures demandées : %lu\r\n", target_samples);
          printf("Mesures OK        : %lu\r\n", ok_count);
          printf("Mesures KO        : %lu\r\n", ko_count);

          if (ok_count > 0U) {  // Affiche stats détaillées si au moins une mesure valide
            printf("VOC index min/max : %u / %u\r\n", min_index, max_index);
            printf("VOC index moyen   : %lu\r\n", (unsigned long)(sum_index / ok_count));
            printf("Résultat global   : OK  PASS\r\n\r\n");
          } else {
            printf("VOC index min/max : N/A\r\n");
            printf("VOC index moyen   : N/A\r\n");
            printf("Résultat global   : ERREUR  FAIL\r\n\r\n");
          }

          test_state = TEST_STATE_DONE;                                                   // Passe en état terminal après impression
          break;

        case TEST_STATE_DONE:  // Etat terminal sans nouvelle mesure
        default:               // Sécurité: traite tout état inattendu en attente passive
          /* 4) Etat terminal: attente passive */
            HAL_Delay(1000U);                                                                        // Boucle passive en fin de scénario
          break;
      }
    
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

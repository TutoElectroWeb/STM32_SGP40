/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : 'exemple_sgp40_async_multi_capteurs.c'
 * @brief          : [I2C IT] SGP40 + BME280 sur le même bus I2C — partage coopératif
 ******************************************************************************
 * @details
 * Cet exemple montre comment faire cohabiter le SGP40 (mode async IT non-bloquant)
 * et un deuxième capteur (ex: BME280, SHT31...) sur le MÊME bus I2C, sans modifier
 * ni stm32l4xx_it.c ni le handler I2C.
 *
 * Pattern de partage de bus (coopératif, sans arbitre dédié) :
 * ─────────────────────────────────────────────────────────────
 *
 *  Timeline SGP40 (1 mesure = ~31.5ms à 400kHz) :
 *  │──── TX 8B (~0.6ms) ─────│── WAIT 30ms (bus libre) ──│── RX 3B (~0.4ms) ─│
 *
 *  Pendant la fenêtre WAIT 30ms, le bus I2C est LIBRE.
 *  Le BME280 peut effectuer sa lecture bloquante (~5ms) dans cette fenêtre.
 *
 *  Si le BME280 n'a pas fini quand SGP40 tente le RX :
 *    → HAL_I2C_Master_Receive_IT retourne HAL_BUSY
 *    → SGP40 ne passe PAS en erreur : il réessaie dans 2ms automatiquement
 *    → Au cycle suivant, le bus est libre → RX réussit
 *
 * Configuration matérielle :
 * ──────────────────────────
 *  - Un seul bus I2C (hi2c3) : SGP40 (0x59) + BME280 (0x76 ou 0x77)
 *  - Vitesse recommandée : 400kHz (Fast Mode) avec pull-up 4.7kΩ
 *  - NVIC : I2C3_EV_IRQn + I2C3_ER_IRQn activés (OBLIGATOIRE pour mode IT)
 *  - PAS de DMA configuré pour I2C (aucun gain sur les trames 3-8 octets du SGP40)
 *  - UART2 : console debug 115200 bauds
 *
 * Compatible FreeRTOS :
 * ─────────────────────
 *  - SGP40_Init() doit être appelé avant vTaskStartScheduler() ou depuis une tâche
 *    d'init unique. Ne jamais appeler Init() depuis plusieurs tâches simultanément.
 *  - HAL_Delay() en FreeRTOS suspend la tâche (pas de spin-wait CPU) ✅
 *  - La FSM SGP40 utilise HAL_GetTick() uniquement → compatible toutes configs.
 *  - Pour plusieurs tâches FreeRTOS accédant aux capteurs sur le même bus I2C :
 *    protéger les appels TriggerEvery/TickIndex par un mutex osMutexAcquire().
 *  - Les callbacks HAL (OnI2CMasterTxCplt, RxCplt, Error) s'exécutent en IRQ :
 *    ne jamais appeler osDelay() ou toute API FreeRTOS non ISR-safe depuis ces
 *    callbacks. Utiliser uniquement osSemaphoreRelease() ou osEventFlagsSet().
 *  - Les champs volatile du handle (async_busy, consecutive_errors) sont lisibles
 *    depuis n'importe quelle tâche sans mutex (atomique 8 bits ARM Cortex-M).
 *
 * API couverte :
 * ──────────────
 *  SGP40_Init, SGP40_SetCompensation, SGP40_SetSampleInterval,
 *  SGP40_PrimeForVocIndex, SGP40_CalculateVOCIndex, SGP40_GetVOCCategory,
 *  SGP40_VOCCategoryToString, SGP40_StatusToString,
 *  SGP40_Async_Init, SGP40_ReadAll_IT, SGP40_Async_TickIndex,
 *  SGP40_Async_TriggerEvery, SGP40_Async_IsIdle,
 *  SGP40_Async_OnI2CMasterTxCplt, SGP40_Async_OnI2CMasterRxCplt,
 *  SGP40_Async_OnI2CError.
 *
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
#define LOG_NAME             "exemple_sgp40_multi_capteurs"  ///< Nom pour identification dans les logs
#define SGP40_MEASURE_PERIOD_MS  1000U  ///< Période mesure SGP40 (ms)
#define BME280_MEASURE_PERIOD_MS  500U  ///< Période mesure BME280 (ms)
#define BME280_I2C_ADDR_7B       0x76   ///< Adresse BME280 (0x76 ou 0x77 selon SDO)
#define BME280_I2C_TIMEOUT_MS    50U    ///< Timeout I2C BME280
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* --- SGP40 (VOC, async IT non-bloquant) --- */
SGP40_HandleTypeDef hsgp40;     ///< Handle principal SGP40
SGP40_Async_t       sgp40_async; ///< Contexte FSM async SGP40

/* --- BME280 (T/P/RH, async IT non-bloquant) --- */
/* Note : remplacer ces variables par les handles de votre lib BME280 si applicable */
static float bme280_temp_c   = 25.0f;  ///< Dernière température lue BME280 (°C)
static float bme280_rh_pct   = 50.0f;  ///< Dernière humidité lue BME280 (%)
static uint32_t bme280_last_ms = 0U;   ///< Timestamp dernier trigger BME280

/* --- Résultats SGP40 --- */
static uint16_t last_voc_raw   = 0U;   ///< Dernier VOC raw SGP40
static uint16_t last_voc_index = 0U;   ///< Dernier VOC index (0-500)
static uint32_t sgp40_last_ms  = 0U;   ///< Timestamp dernier trigger SGP40
static uint8_t  sgp40_error_count = 0U; ///< Compteur d'erreurs consécutives SGP40

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
 * @brief Redirige printf vers UART2.
 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, 0xFFFF);
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

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();

  /* USER CODE BEGIN 2 */

  /* ===========================================================================
   * Initialisation SGP40
   * ===========================================================================
   *
   * SGP40_Init configure les défauts (addr, timeout, T/RH 25°C/50%),
   * lit le serial number et prépare l'algorithme VOC. Bloquant ~30ms une
   * seule fois. Si FreeRTOS : appeler depuis une tâche init (non IRQ).
   */
  SGP40_Status init_status = SGP40_Init(&hsgp40, &hi2c3);
  if (init_status != SGP40_OK) {
      printf("[%s] ERREUR Init SGP40: %s\r\n", LOG_NAME, SGP40_StatusToString(init_status));
      Error_Handler();
  }

  /* Compensation initiale avec les valeurs par défaut (sera mise à jour via BME280) */
  SGP40_SetCompensation(&hsgp40, 25.0f, 50.0f);
  SGP40_SetSampleInterval(&hsgp40, SGP40_MEASURE_PERIOD_MS);
  SGP40_PrimeForVocIndex(&hsgp40);   // Discard première mesure (warmup heater)

  /* Initialise le contexte de machine d'état async (IT) */
  SGP40_Async_Init(&sgp40_async, &hsgp40); // Pas de callbacks : Tick() suffit

  printf("[%s] SGP40 prêt — Serial: 0x%012llX\r\n",
         LOG_NAME, (unsigned long long)hsgp40.serial_number);
  printf("[%s] Bus I2C partagé : SGP40 (0x%02X) + BME280 (0x%02X)\r\n",
         LOG_NAME, SGP40_I2C_ADDR_7B, BME280_I2C_ADDR_7B);
  printf("[%s] Mode : SGP40 async IT non-bloquant, BME280 async IT non-bloquant\r\n", LOG_NAME);

  /* Premier déclenchement manuel pour valider la configuration (NVIC/I2C) */
  sgp40_last_ms = HAL_GetTick();
  SGP40_Status trig_status = SGP40_ReadAll_IT(&sgp40_async);

  if (trig_status != SGP40_OK) {
      if (trig_status == SGP40_ERR_NOT_CONFIGURED) {
          printf("[%s] ERREUR Configuration MX incomplète (I2C/IRQ) pour mode IT\r\n", LOG_NAME);
          printf("   Action: vérifier I2C choisi + IRQ EV/ER puis régénérer\r\n");
      } else {
          printf("[%s] ERREUR Erreur trigger: %s\r\n", LOG_NAME, SGP40_StatusToString(trig_status));
      }
      Error_Handler();
  }
  printf("[%s] INFO Première mesure lancée...\r\n\r\n", LOG_NAME);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* =========================================================================
     * CAPTEUR 1 — SGP40 : trigger périodique (IT, non-bloquant)
     *
     * TriggerEvery lance ReadAll_IT() seulement si :
     *   1. La FSM est IDLE (aucune mesure en cours)
     *   2. L'intervalle sgp40_last_ms + 1000ms est écoulé
     *
     * Si le I2C est occupé par le BME280 au moment du TX,
     * TriggerEvery retourne SGP40_ERR_BUSY → sgp40_last_ms n'est pas mis à
     * jour → retentée au cycle suivant (1ms plus tard).
     * =========================================================================*/
    SGP40_Async_TriggerEvery(&sgp40_async, now, &sgp40_last_ms);

    /* =========================================================================
     * CAPTEUR 1 — SGP40 : avancement FSM + récupération index VOC
     *
     * TickIndex = Tick() + CalculateVOCIndex() en un seul appel.
     * Retourne TICK_DATA_READY uniquement quand l'index est calculé.
     * Si le bus est occupé par BME280 au moment du TX, la FSM reste IDLE
     * (fix HAL_BUSY) → pas de TICK_ERROR parasite → retry transparent.
     * =========================================================================*/
    uint16_t voc_raw_this_tick = 0U;
    SGP40_TickResult tick = SGP40_Async_TickIndex(&sgp40_async, now,
                                                   &voc_raw_this_tick, &last_voc_index);

    if (tick == SGP40_TICK_DATA_READY) {
        sgp40_error_count = 0U;
        last_voc_raw = voc_raw_this_tick;

        /* Mettre à jour la compensation avec la dernière T/RH mesurée par BME280 */
        SGP40_SetCompensation(&hsgp40, bme280_temp_c, bme280_rh_pct);

        SGP40_VOC_Category cat = SGP40_GetVOCCategory(last_voc_index);
        printf("[SGP40] VOC raw=%5u  index=%3u  %s   T=%.1f°C  RH=%.1f%%\r\n",
               last_voc_raw, last_voc_index,
               SGP40_VOCCategoryToString(cat),
               bme280_temp_c, bme280_rh_pct);
    }
    else if (tick == SGP40_TICK_ERROR) {
        sgp40_error_count++;
        if (sgp40_error_count >= SGP40_MAX_CONSECUTIVE_ERRORS) {
            // Seuil configurable via STM32_SGP40_conf.h
            printf("[%s] ERREUR I2C repetee [%u/%u] (capteur debranche ?)\r\n",
                   LOG_NAME, sgp40_error_count, SGP40_MAX_CONSECUTIVE_ERRORS);
            SGP40_DeInit(&hsgp40);
            Error_Handler();
        }
        printf("[SGP40] Erreur async [%u/%u] — FSM reset, retentative au prochain cycle\r\n",
               sgp40_error_count, SGP40_MAX_CONSECUTIVE_ERRORS);
    }

    /* =========================================================================
     * CAPTEUR 2 — BME280 : trigger périodique (IT, non-bloquant)
     *
     * Stratégie cooperative de partage de bus :
     *  - On déclenche le BME280 de la même manière que le SGP40.
     *  - Si le bus est occupé par le SGP40, la FSM du BME280 restera IDLE
     *    et retentera au cycle suivant.
     * =========================================================================*/
    /* Exemple d'appel pour une lib BME280 async IT :
    STM32_BME280_Async_TriggerEvery(&hbme280_async, now, BME280_MEASURE_PERIOD_MS, &bme280_last_ms);
    STM32_BME280_TickResult bme_tick = STM32_BME280_Async_Tick(&hbme280_async, now, &bme280_temp_c, &bme280_rh_pct);
    if (bme_tick == STM32_BME280_TICK_DATA_READY) {
        printf("[BME280] T=%.1f°C  RH=%.1f%%\r\n", bme280_temp_c, bme280_rh_pct);
    }
    */

    /* Simulation d'une tâche applicative bloquante (ex: envoi réseau, calcul lourd) */
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(4000U); // Délai bloquant de 4s pour prouver que l'IT tourne en tâche de fond

    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */

/* =============================================================================
 * Callbacks HAL I2C — Dispatch vers toutes les libs async IT du bus
 * =============================================================================
 *
 * Ces callbacks sont appelés depuis les IRQ I2C3_EV et I2C3_ER.
 * Chaque fonction STM32_xxx_Async_OnI2Cxxx() filtre par son handle hi2c
 * interne → appeler TOUTES les libs sans if externe.
 *
 * Compatibilité multi-lib IT :
 *   - Si deux libs tentent de démarrer un transfert simultanément, HAL retourne
 *     HAL_BUSY pour la seconde → grâce au fix TriggerMeasure_Common, la FSM
 *     reste IDLE au lieu de passer ERROR → retry transparent au cycle suivant.
 *   - Le BME280 (polling bloquant) ne génère pas de callbacks HAL → aucune
 *     modification nécessaire pour lui.
 *
 * Pour ajouter une lib async IT (ex: STM32_AHT20) :
 *   1. Déclarer `STM32_AHT20_Async_t haht20_async;` en variable globale
 *   2. Décommenter les appels ci-dessous
 *   3. Ajouter `STM32_AHT20_Async_TriggerEvery()` + `TickIndex()` dans while(1)
 * =============================================================================*/

/**
 * @brief Callback HAL — transfert I2C Tx terminé.
 * @note  Appelé depuis IRQ I2C3_EV après que HAL_I2C_Master_Transmit_IT réussit.
 */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    SGP40_Async_OnI2CMasterTxCplt(&sgp40_async, hi2c);                         // SGP40 — filtre hi2c en interne
    // STM32_AHT20_Async_OnI2CMasterTxCplt(&haht20_async, hi2c);                     // AHT20 — même pattern (décommenter si utilisé)
}

/**
 * @brief Callback HAL — transfert I2C Rx terminé.
 * @note  Appelé depuis IRQ I2C3_EV après que HAL_I2C_Master_Receive_IT réussit.
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    SGP40_Async_OnI2CMasterRxCplt(&sgp40_async, hi2c);                         // SGP40 — filtre hi2c en interne
    // STM32_AHT20_Async_OnI2CMasterRxCplt(&haht20_async, hi2c);                     // AHT20 — même pattern (décommenter si utilisé)
}

/**
 * @brief Callback HAL — erreur I2C (NACK, arbitration loss, timeout bus...).
 * @note  Appelé depuis IRQ I2C3_ER.
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    SGP40_Async_OnI2CError(&sgp40_async, hi2c);                                 // SGP40 — filtre hi2c en interne
    // STM32_AHT20_Async_OnI2CError(&haht20_async, hi2c);                       // AHT20 — même pattern (décommenter si utilisé)
}

/* USER CODE END 4 */

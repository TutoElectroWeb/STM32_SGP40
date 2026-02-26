/**
  ******************************************************************************
  * @file    STM32_SGP40.h
  * @brief   Driver STM32 HAL pour capteur VOC SGP40 (Sensirion)
  * @author  Generated via STM32_LIB_STYLE_GUIDE.md
  * @date    2026-02-11
  * @version 0.9.0
  * @copyright Libre sous licence MIT.
  ******************************************************************************
  * @attention
  *
  * Capteur : SGP40 (Sensirion) - VOC (Volatile Organic Compounds)
  * Bus : I2C (adresse fixe 0x59)
  * Mesure : VOC raw signal (0-65535) → VOC index (0-500) via algorithme
  *
  * Conformité : STM32_LIB_STYLE_GUIDE.md v2.0 (volatile sur champs IRQ,
  * DeInit, consecutive_errors, conf.h, guard polling/async, 0 malloc)
  *
  * @note  Délais bloquants : SGP40_MEASURE_WAIT_MS (~30 ms) dans MeasureRaw(),
  *        SGP40_SELFTEST_WAIT_MS (~320 ms) dans SelfTest(). Le mode IT (async)
  *        élimine ces blocages dans la boucle principale.
  *
  * @note  Compatibilité FreeRTOS : SGP40_Init() doit être appelé AVANT
  *        vTaskStartScheduler(). En contexte tâche, protéger l'accès au bus
  *        I2C partagé par un mutex. Ne jamais appeler HAL_Delay() depuis une IRQ.
  *
  * @note  Contraintes CPU/FPU : l'algorithme VOC Sensirion (GasIndexAlgorithm)
  *        utilise des calculs float en interne. Sur Cortex-M0/M0+/M3 sans FPU
  *        matérielle (STM32G0, L0, F1, F2) les opérations sont émulées via
  *        softfp (~8–20 cycles/op) — overhead notable à 1 Hz. Sur Cortex-M4F/M7
  *        avec FPU activée (-mfpu=fpv4-sp-d16 -mfloat-abi=hard), impact
  *        négligeable (<2 cycles/op). Vérifier le réglage FPU dans CubeMX :
  *        Project Manager → Advanced Settings → Floating Point Unit.
  *
  * @note  HAL_BUSY en mode synchrone (polling) : si HAL_I2C_Master_Transmit/
  *        Receive retourne HAL_BUSY en mode polling, le périphérique I2C est
  *        matériellement occupé — état anormal en accès exclusif (guard
  *        async_busy en place). consecutive_errors est incrémenté intentionnellement.
  *        En mode asynchrone IT (SGP40_ReadAll_IT, bus I2C partagé multi-capteurs),
  *        HAL_BUSY est une contention normale (autre lib en transfert) et
  *        n'incrémente pas consecutive_errors — la FSM reste IDLE et TriggerEvery
  *        retentera automatiquement au prochain cycle.
  *
  * @note  sizeof(SGP40_Handle_t) ≈ 220 bytes (164 opaque algo VOC + contexte).
  *        sizeof(SGP40_Async_t)  ≈  68 bytes.
  *
  ******************************************************************************
  */

#ifndef STM32_SGP40_H
#define STM32_SGP40_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Includes
 * ============================================================================ */
#include "main.h"                  ///< HAL multi-famille STM32 via CubeMX
#include "STM32_SGP40_conf.h"      ///< Configuration overridable (timeouts, seuils, debug)
#include <stdint.h>                ///< Types uint8_t, uint16_t, int32_t…
#include <stdbool.h>               ///< Type bool, true, false

/* =============================================================================
 * Constantes
 * ============================================================================= */

/** @brief Taille du buffer opaque interne pour l'algorithme VOC Sensirion.
 *  Correspond à sizeof(GasIndexAlgorithmParams) — vérifié par _Static_assert dans STM32_SGP40.c.
 *  @note Le header vendor sensirion_gas_index_algorithm.h est inclus dans STM32_SGP40.c uniquement.
 */
#define SGP40_VOC_ALGO_OPAQUE_SIZE   164U

#define SGP40_LIB_VERSION_MAJOR      0         ///< Version majeure de la lib
#define SGP40_LIB_VERSION_MINOR      9         ///< Version mineure de la lib
#define SGP40_LIB_VERSION_PATCH      0         ///< Version patch de la lib

/** Adresse I2C SGP40 (profil SINGLE_ADDRESS) */
#define SGP40_I2C_ADDR_7B            0x59U

/** Cadence recommandée pour l'algorithme VOC Index (ms) */
#define SGP40_DEFAULT_SAMPLE_INTERVAL_MS  1000U

/** Commandes SGP40 — datasheet Sensirion SGP40 Rev. 1.1 */
#define SGP40_CMD_MEASURE_RAW        0x260FU   ///< Mesure VOC raw (avec compensation T/RH)
#define SGP40_CMD_MEASURE_TEST       0x280EU   ///< Auto-test capteur
#define SGP40_CMD_HEATER_OFF         0x3615U   ///< Extinction heater (power-down)
#define SGP40_CMD_GET_SERIAL         0x3682U   ///< Lecture serial number (48 bits)

/** Valeurs par défaut compensation T/RH */
#define SGP40_DEFAULT_TEMP_C         25.0f     ///< Température par défaut (°C)
#define SGP40_DEFAULT_RH_PERCENT     50.0f     ///< Humidité relative par défaut (%)

/** CRC-8 polynomial (Sensirion) */
#define SGP40_CRC8_POLYNOMIAL        0x31U     ///< Polynôme CRC8 propriétaire Sensirion
#define SGP40_CRC8_INIT              0xFFU     ///< Valeur initiale CRC8 (standard Sensirion)

/** Tailles internes nommées */
#define SGP40_CMD_DATA_BUF_SIZE      20U       ///< Buffer max pour WriteCommandWithData
#define SGP40_ASYNC_CMD_BUF_SIZE     8U        ///< Buffer commande async (cmd 2B + T/RH 6B)

/** VOC Index (algorithme Sensirion) - plages typiques */
#define SGP40_VOC_INDEX_MIN          0U        ///< Air pur
#define SGP40_VOC_INDEX_TYPICAL      100U      ///< Valeur baseline typique
#define SGP40_VOC_INDEX_MAX          500U      ///< Concentration maximale

/** Résultats auto-test SGP40 — datasheet §5.2 (octet bas toujours 0x00) */
#define SGP40_TEST_RESULT_OK         0xD400U   ///< Auto-test réussi  (heater + sensor OK)
#define SGP40_TEST_RESULT_FAIL       0x4B00U   ///< Auto-test échoué (heater ou sensor défectueux)

/* =============================================================================
 * Types exportés
 * ============================================================================= */

/**
 * @brief Codes d'erreur SGP40
 */
typedef enum {
    SGP40_OK = 0,              ///< Opération réussie
    SGP40_ERR_NULL_PTR,        ///< Pointeur NULL
    SGP40_ERR_INVALID_PARAM,   ///< Paramètre invalide
    SGP40_ERR_NOT_CONFIGURED,  ///< Configuration HAL/NVIC requise absente
    SGP40_ERR_BUSY,            ///< Bus occupé ou cadence non respectée
    SGP40_ERR_I2C,             ///< Erreur I2C (HAL_ERROR/TIMEOUT/BUSY)
    SGP40_ERR_CRC,             ///< CRC8 invalide
    SGP40_ERR_SELF_TEST,       ///< Auto-test échoué
    SGP40_ERR_NOT_READY,       ///< Capteur pas prêt (heater off / non init)
    SGP40_ERR_TIMEOUT,         ///< Timeout mesure ou I2C
    SGP40_ERR_NOT_INITIALIZED, ///< Handle non initialisé (SGP40_Init non appelé)
    SGP40_ERR_OVERFLOW,        ///< File de mesures async pleine (future use)
    SGP40_ERR_UNKNOWN          ///< Erreur inconnue
} SGP40_Status;

/**
 * @brief Handle SGP40 (configuration + état)
 *
 * @note  Base de temps : HAL_GetTick() / HAL_Delay() (SysTick).
 * @note  Champs préfixés async_* modifiés depuis les callbacks HAL (IRQ) →
 *        déclarés volatile pour garantir la cohérence cache/registre ARM.
 * @note  sizeof(SGP40_Handle_t) ≈ 220 bytes — allouer statiquement.
 */
typedef struct {
    /* Configuration I2C --------------------------------------------------- */
    I2C_HandleTypeDef *hi2c;          ///< Handle I2C HAL
    uint8_t            i2c_addr_7b;   ///< Adresse I2C 7-bit (défaut: SGP40_I2C_ADDR_7B)
    uint16_t           i2c_timeout;   ///< Timeout I2C ms (défaut: SGP40_DEFAULT_TIMEOUT_MS)

    /* Compensation T/RH --------------------------------------------------- */
    float temp_c;                     ///< Température (°C) pour compensation
    float rh_percent;                 ///< Humidité relative (%) pour compensation

    /* État capteur --------------------------------------------------------- */
    bool     initialized;             ///< true après SGP40_Init() réussi
    uint64_t serial_number;           ///< Serial number 48 bits (lu au init)
    bool     measurement_mode_active; ///< true après première mesure, false après HeaterOff
    uint32_t sample_interval_ms;      ///< Intervalle minimal entre mesures (ms)
    uint32_t last_measure_tick_ms;    ///< Timestamp (ms) de la dernière mesure

    /* Algorithme VOC Sensirion (opaque) ------------------------------------ */
    /** @brief État interne algorithme VOC Sensirion (opaque — ne pas accéder directement).
     *  Alloué par valeur pour éviter toute allocation dynamique.
     *  Taille vérifiée par _Static_assert dans STM32_SGP40.c.
     */
    uint8_t _voc_algo_opaque[SGP40_VOC_ALGO_OPAQUE_SIZE] __attribute__((aligned(4)));
    bool    voc_algo_initialized;     ///< true si l'algo VOC est initialisé

    /* Mesures -------------------------------------------------------------- */
    uint16_t last_voc_raw;            ///< Dernière valeur VOC raw (0-65535)
    uint32_t measure_count;           ///< Compteur de mesures (pour tracking warmup)

    /* Suivi erreurs -------------------------------------------------------- */
    volatile uint8_t  consecutive_errors; ///< Erreurs I2C consécutives (reset à 0 au succès)
    volatile SGP40_Status last_error;     ///< Dernier code d'erreur retourné (modifié depuis IRQ → volatile)
    uint32_t          last_hal_error;     ///< Dernier code HAL_I2C_GetError() (debug)

    /* Sync async/polling --------------------------------------------------- */
    /** @brief 1 = une opération IT async est en cours sur ce handle.
     *  Modifié depuis les callbacks HAL (IRQ) → volatile obligatoire.
     *  Les fonctions polling vérifient ce flag avant d'accéder au bus I2C.
     */
    volatile uint8_t async_busy;
} SGP40_Handle_t;


/**
 * @brief Résultat de SGP40_Async_Tick() — remplace le polling manuel dans while(1)
 *
 * @note  Valeurs fixes (ABI contract inter-lib) :
 *        IDLE=0  BUSY=1  DATA_READY=2  ERROR=3
 */
typedef enum {
    SGP40_TICK_IDLE       = 0,   ///< Aucune mesure en cours
    SGP40_TICK_BUSY       = 1,   ///< Mesure en cours (non-bloquant)
    SGP40_TICK_DATA_READY = 2,   ///< Données disponibles dans *voc_raw_out
    SGP40_TICK_ERROR      = 3    ///< Erreur — FSM réinitialisée automatiquement
} SGP40_TickResult;

/**
 * @brief Catégories VOC Index (interprétation qualité air)
 */
typedef enum {
    SGP40_VOC_EXCELLENT = 0,   ///< 0-100 : Air excellent
    SGP40_VOC_GOOD,            ///< 101-150 : Bon
    SGP40_VOC_MODERATE,        ///< 151-200 : Modéré
    SGP40_VOC_POOR,            ///< 201-300 : Mauvais
    SGP40_VOC_UNHEALTHY,       ///< 301-500 : Insalubre
    SGP40_VOC_INVALID          ///< Hors plage / erreur
} SGP40_VOC_Category;

/**
 * @brief Données métier SGP40 — résultat d'une mesure complète
 *
 * Regroupe le signal brut et le VOC Index calculé pour un usage simplifié
 * dans la boucle principale (pattern @c PREFIX_ReadAll).
 */
typedef struct {
    uint16_t voc_raw;    ///< Signal brut SRAW_VOC [0..65535]
    uint16_t voc_index;  ///< VOC Index [0..500] (0 pendant blackout algo)
} SGP40_Data;

/* =============================================================================
 * Async (non-bloquant IT)
 * ============================================================================= */

/**
 * @brief Machine d'état pour mesure asynchrone
 *
 * Permet de déclencher une mesure VOC sans bloquer la boucle principale.
 * Utilise le mode IT (interruptions I2C) pour les transferts.
 * DMA non implémenté : aucun gain sur les trames 8B/3B du SGP40.
 */
typedef enum {
    SGP40_ASYNC_IDLE       = 0, ///< Aucune mesure en cours
    SGP40_ASYNC_WRITE_CMD,      ///< Écriture commande + params T/RH
    SGP40_ASYNC_WAIT_MEAS,      ///< Attente délai mesure (30ms)
    SGP40_ASYNC_READ_DATA,      ///< Lecture VOC raw (3 octets: MSB, LSB, CRC)
    SGP40_ASYNC_DONE,           ///< Mesure terminée, données disponibles
    SGP40_ASYNC_ERROR           ///< Erreur survenue
} SGP40_AsyncState;

/**
 * @brief Callback utilisateur - données prêtes
 * @param user_ctx Contexte utilisateur (passé à Init)
 * @param voc_raw  Valeur VOC raw mesurée
 * @param status   Code statut de la mesure
 * @note  Appelé depuis SGP40_Async_Process() (contexte main loop)
 */
typedef void (*SGP40_Async_OnDataReadyCb)(void *user_ctx, uint16_t voc_raw, SGP40_Status status);

/**
 * @brief Callback utilisateur - erreur
 * @param user_ctx Contexte utilisateur
 * @param status   Code erreur
 * @note  Appelé depuis SGP40_Async_Process() (contexte main loop)
 */
typedef void (*SGP40_Async_OnErrorCb)(void *user_ctx, SGP40_Status status);

/**
 * @brief Callback IRQ-safe - données prêtes (depuis interruption I2C)
 * @param user_ctx Contexte utilisateur
 * @note  Appelé depuis HAL I2C callback (interruption) — DOIT être ultra-court.
 * @warning Pas de printf, HAL_Delay, ni opération bloquante ici !
 */
typedef void (*SGP40_Async_OnIrqDataReadyCb)(void *user_ctx);

/**
 * @brief Callback IRQ-safe - erreur (depuis interruption I2C)
 * @param user_ctx Contexte utilisateur
 * @note  Appelé depuis HAL I2C callback (interruption) — DOIT être ultra-court.
 */
typedef void (*SGP40_Async_OnIrqErrorCb)(void *user_ctx);

/**
 * @brief Contexte pour mesure asynchrone SGP40
 *
 * @note  Base de temps : HAL_GetTick() (SysTick).
 * @note  sizeof(SGP40_Async_t) ≈ 68 bytes — allouer statiquement.
 * @warning Modèle de thread unique : SGP40_Async_Process(), SGP40_Async_Tick()
 *          et SGP40_Async_TriggerEvery() DOIVENT être appelées depuis un seul
 *          contexte d'exécution (main loop ou une seule tâche FreeRTOS).
 *          Les callbacks on_irq_* sont les SEULS points d'entrée autorisés
 *          depuis les IRQ I2C HAL (MasterTxCplt, MasterRxCplt, Error).
 *          Sur ARM Cortex-M, les écritures uint8_t/bool alignées sont atomiques
 *          par architecture — volatile suffit, __disable_irq() n'est pas requis.
 *          Avec FreeRTOS : placer Process/Tick dans une seule tâche low-priority
 *          dédiée aux capteurs, ou protéger ctx par un mutex si partagé.
 */
typedef struct {
    SGP40_Handle_t    *hsgp40;               ///< Handle SGP40 synchrone (i2c, compensation)
    I2C_HandleTypeDef *hi2c;                 ///< Handle I2C (copié depuis hsgp40)

    volatile SGP40_AsyncState state;         ///< État machine d'état (modifié depuis IRQ)
    volatile SGP40_Status     last_status;   ///< Dernier code erreur (modifié depuis IRQ)

    uint32_t meas_deadline_ms;               ///< Deadline mesure (WAIT_MEAS)
    uint32_t i2c_deadline_ms;               ///< Deadline transfert I2C

    uint8_t  cmd_buf[SGP40_ASYNC_CMD_BUF_SIZE]; ///< Buffer commande (cmd 2B + params 6B)
    uint8_t  rx_buf[3];                      ///< Buffer réception (MSB, LSB, CRC)
    uint16_t voc_raw;                        ///< Dernière valeur VOC raw lue

    volatile bool data_ready_flag;           ///< Flag données prêtes (set depuis ISR)
    volatile bool error_flag;                ///< Flag erreur (set depuis ISR)
    volatile bool notify_data_pending;       ///< Notify data à envoyer (Process) — écrit en IRQ → volatile
    volatile bool notify_error_pending;      ///< Notify error à envoyer (Process) — écrit en IRQ → volatile

    /* Callbacks utilisateur ----------------------------------------------- */
    SGP40_Async_OnDataReadyCb    on_data_ready;     ///< Données prêtes (main loop, peut être NULL)
    SGP40_Async_OnErrorCb        on_error;           ///< Erreur (main loop, peut être NULL)
    SGP40_Async_OnIrqDataReadyCb on_irq_data_ready; ///< Données prêtes depuis IRQ (ultra-court)
    SGP40_Async_OnIrqErrorCb     on_irq_error;      ///< Erreur depuis IRQ (ultra-court)
    void                        *user_ctx;           ///< Contexte callbacks main-loop (SetCallbacks)
    void                        *irq_user_ctx;       ///< Contexte callbacks IRQ-safe (SetIrqCallbacks) — NE PAS écraser user_ctx
} SGP40_Async_t;

/* =============================================================================
 * API synchrone (bloquante)
 * ============================================================================= */

/**
 * @brief Initialise le capteur SGP40 (config + serial + algo VOC + prime)
 * @param hsgp40 Handle SGP40 (non NULL)
 * @param hi2c   Handle I2C HAL (non NULL)
 * @retval SGP40_Status
 * @note  Configure tous les défauts (addr, timeout, T/RH, intervalle).
 *        Effectue une mesure de chauffe silencieuse (prime) automatiquement.
 *        Configure les valeurs par défaut, cadence et amorçage VOC en une seule init.
 */
SGP40_Status SGP40_Init(SGP40_Handle_t *hsgp40, I2C_HandleTypeDef *hi2c);

/**
 * @brief Réinitialise le handle SGP40 (libère ressources logicielles)
 * @param hsgp40 Handle SGP40 (non NULL)
 * @retval SGP40_Status
 * @note  Remet h à zéro (initialized=false, async_busy=0, hi2c=NULL).
 *        À appeler en cas d'erreur fatale, avant bootloader, ou en test unitaire.
 *        Retourne SGP40_ERR_BUSY si une opération IT async est en cours.
 */
SGP40_Status SGP40_DeInit(SGP40_Handle_t *hsgp40);

/**
 * @brief Lecture serial number 48 bits
 * @param hsgp40 Handle SGP40
 * @param serial Pointeur pour stocker serial (6 octets)
 * @retval SGP40_Status
 */
SGP40_Status SGP40_GetSerialNumber(SGP40_Handle_t *hsgp40, uint64_t *serial);

/**
 * @brief Auto-test capteur (heater + sensor)
 * @param hsgp40      Handle SGP40
 * @param test_result Résultat test brut — comparer avec SGP40_TEST_RESULT_OK (0xD400) :
 *                    - 0xD400 : tous les tests réussis (heater + sensor OK)
 *                    - 0x4B00 : au moins un test en échec
 *                    - Datasheet §5.2 : l'octet bas est toujours 0x00
 * @retval SGP40_OK            si test_result == 0xD400
 * @retval SGP40_ERR_SELF_TEST si test_result != 0xD400
 * @note  Bloquant jusqu'à SGP40_SELFTEST_WAIT_MS (320 ms).
 */
SGP40_Status SGP40_SelfTest(SGP40_Handle_t *hsgp40, uint16_t *test_result);

/**
 * @brief Mesure VOC raw (signal brut 0-65535)
 * @param hsgp40  Handle SGP40
 * @param voc_raw Pointeur pour stocker signal VOC raw
 * @retval SGP40_OK                 si mesure réussie
 * @retval SGP40_ERR_NULL_PTR       si hsgp40 ou voc_raw est NULL
 * @retval SGP40_ERR_NOT_INITIALIZED si SGP40_Init() non appelé (ou HeaterOff() effectué)
 * @retval SGP40_ERR_BUSY           si async en cours ou cadence non respectée
 * @retval SGP40_ERR_I2C            si erreur de communication I2C
 * @retval SGP40_ERR_CRC            si CRC-8 invalide sur la réponse
 * @note  Bloquant ~SGP40_MEASURE_WAIT_MS (30 ms), utilise compensation T/RH configurée.
 */
SGP40_Status SGP40_MeasureRaw(SGP40_Handle_t *hsgp40, uint16_t *voc_raw);

/**
 * @brief Prime le capteur pour usage VOC Index (discard première mesure)
 * @param hsgp40 Handle SGP40
 * @retval SGP40_Status
 * @note  Recommandé après Init() avant d'alimenter l'algorithme VOC Index.
 */
SGP40_Status SGP40_PrimeForVocIndex(SGP40_Handle_t *hsgp40);

/**
 * @brief Configure l'intervalle minimal entre mesures
 * @param hsgp40      Handle SGP40
 * @param interval_ms Intervalle en ms (plage supportée: 500..10000)
 * @retval SGP40_Status
 * @note  ⚠️  Réinitialise l'algorithme VOC Sensirion si déjà actif :
 *        l'état d'apprentissage accumulé (baseline, gain, offset) est perdu.
 *        Appeler uniquement en phase d'initialisation, avant toute acquisition,
 *        jamais en cours de mesure nominale.
 */
SGP40_Status SGP40_SetSampleInterval(SGP40_Handle_t *hsgp40, uint32_t interval_ms);

/**
 * @brief Configure compensation température/humidité
 * @param hsgp40     Handle SGP40
 * @param temp_c     Température (°C), plage -45 à +130
 * @param rh_percent Humidité relative (%), plage 0 à 100
 * @retval SGP40_Status
 */
SGP40_Status SGP40_SetCompensation(SGP40_Handle_t *hsgp40, float temp_c, float rh_percent);

/**
 * @brief Extinction heater (mode power-down)
 * @param hsgp40 Handle SGP40
 * @retval SGP40_Status
 * @note  Réduit consommation ~0.1µA, nécessite SGP40_Init() après réutilisation.
 */
SGP40_Status SGP40_HeaterOff(SGP40_Handle_t *hsgp40);

/**
 * @brief Soft Reset via I2C General Call (réinitialise TOUS les périphériques I2C du bus)
 * @param hsgp40 Handle SGP40 (pour accéder à hi2c et i2c_timeout)
 * @retval SGP40_OK                 Reset envoyé (ou NACK GC normal — reset probable)
 * @retval SGP40_ERR_NULL_PTR       hsgp40 NULL
 * @retval SGP40_ERR_NOT_CONFIGURED hi2c non configuré
 * @retval SGP40_ERR_BUSY           async en cours
 * @retval SGP40_ERR_TIMEOUT        timeout I2C (bus bloqué)
 * @warning ⚠️  Reset I2C General Call (addr 0x00, data 0x06 — NXP I2C spec §3.1.12) :
 *          TOUS les périphériques I2C du bus sont réinitialisés, pas uniquement le SGP40.
 * @note   Identique à sensirion_i2c_general_call_reset() du driver officiel Sensirion.
 *         Après appel, le SGP40 est en état POR → appeler SGP40_Init() avant usage.
 *         NACK sur General Call est normal pour certains périphériques → retour SGP40_OK
 *         même si HAL retourne HAL_ERROR avec flag AF.
 */
SGP40_Status SGP40_SoftReset(SGP40_Handle_t *hsgp40);

/**
 * @brief Calcule VOC Index depuis VOC raw (algorithme officiel Sensirion)
 * @param hsgp40    Handle SGP40 (contient l'état interne de l'algorithme)
 * @param voc_raw   Signal VOC raw (0-65535)
 * @param voc_index Pointeur pour stocker VOC index (0-500)
 * @retval SGP40_Status
 * @note  Conserver une cadence constante (recommandé 1s).
 */
SGP40_Status SGP40_CalculateVOCIndex(SGP40_Handle_t *hsgp40, uint16_t voc_raw, uint16_t *voc_index);

/**
 * @brief Reset des états internes de l'algorithme VOC Sensirion
 * @param hsgp40 Handle SGP40
 * @retval SGP40_Status
 */
SGP40_Status SGP40_VOCAlgoReset(SGP40_Handle_t *hsgp40);

/**
 * @brief Récupère les états VOC (pour persistance / reprise)
 * @param hsgp40 Handle SGP40
 * @param state0 Sortie état 0
 * @param state1 Sortie état 1
 * @retval SGP40_Status
 * @note  Usage avancé — persistance de l'état d'apprentissage en EEPROM/Flash
 *        pour reprendre l'algo VOC sans phase de warmup après une coupure courte.
 *        Ne pas appeler en boucle nominale.
 */
SGP40_Status SGP40_VOCAlgoGetStates(const SGP40_Handle_t *hsgp40, float *state0, float *state1);

/**
 * @brief Restaure les états VOC (reprise après interruption courte)
 * @param hsgp40 Handle SGP40
 * @param state0 État 0
 * @param state1 État 1
 * @retval SGP40_Status
 * @note  Usage avancé — restaurer des états issus de SGP40_VOCAlgoGetStates().
 *        Invalide si la coupure dépasse ~30 min (baseline dérivée, recommencer).
 */
SGP40_Status SGP40_VOCAlgoSetStates(SGP40_Handle_t *hsgp40, float state0, float state1);

/**
 * @brief Configure les paramètres de tuning de l'algorithme VOC
 * @param hsgp40                       Handle SGP40
 * @param index_offset                 Offset index (1..250)
 * @param learning_time_offset_hours   Temps apprentissage offset (1..1000h)
 * @param learning_time_gain_hours     Temps apprentissage gain (1..1000h)
 * @param gating_max_duration_minutes  Durée max gating (0..3000min)
 * @param std_initial                  Écart-type initial (10..5000)
 * @param gain_factor                  Gain (1..1000)
 * @retval SGP40_Status
 * @note  Usage avancé — paramètres documentés dans Sensirion AN#1 et AP_SGP40.
 *        Ne modifier qu'avec une validation expérimentale complète. Les valeurs
 *        par défaut Sensirion sont optimales pour 98 % des applications.
 */
SGP40_Status SGP40_VOCAlgoSetTuning(
    SGP40_Handle_t *hsgp40,
    int32_t index_offset,
    int32_t learning_time_offset_hours,
    int32_t learning_time_gain_hours,
    int32_t gating_max_duration_minutes,
    int32_t std_initial,
    int32_t gain_factor
);

/**
 * @brief Lit les paramètres de tuning de l'algorithme VOC
 * @param hsgp40                       Handle SGP40
 * @param index_offset                 Sortie
 * @param learning_time_offset_hours   Sortie
 * @param learning_time_gain_hours     Sortie
 * @param gating_max_duration_minutes  Sortie
 * @param std_initial                  Sortie
 * @param gain_factor                  Sortie
 * @retval SGP40_Status
 * @note  Usage avancé — utile pour vérifier les paramètres actifs ou les
 *        sauvegarder avant un SGP40_SetSampleInterval() (resets l'algo).
 */
SGP40_Status SGP40_VOCAlgoGetTuning(
    const SGP40_Handle_t *hsgp40,
    int32_t *index_offset,
    int32_t *learning_time_offset_hours,
    int32_t *learning_time_gain_hours,
    int32_t *gating_max_duration_minutes,
    int32_t *std_initial,
    int32_t *gain_factor
);

/**
 * @brief Lit l'intervalle d'échantillonnage utilisé par l'algorithme (secondes)
 * @param hsgp40              Handle SGP40
 * @param sampling_interval_s Sortie intervalle (s)
 * @retval SGP40_Status
 * @note  Usage avancé — valeur miroir de sample_interval_ms/1000. Utile pour
 *        vérifier la cohérence entre le handle et l'algo après SetSampleInterval.
 */
SGP40_Status SGP40_VOCAlgoGetSamplingInterval(const SGP40_Handle_t *hsgp40, float *sampling_interval_s);

/**
 * @brief Catégorise VOC Index (qualité air)
 * @param voc_index VOC index (0-500)
 * @retval SGP40_VOC_Category
 */
SGP40_VOC_Category SGP40_GetVOCCategory(uint16_t voc_index);

/**
 * @brief Conversion VOC category → texte (description)
 * @param category Catégorie VOC
 * @retval Chaîne statique (ne pas libérer)
 */
const char *SGP40_VOCCategoryToString(SGP40_VOC_Category category);

/**
 * @brief Conversion code erreur → texte (debug uniquement)
 * @param code Code erreur SGP40
 * @retval Chaîne statique (ne pas libérer), "" si SGP40_DEBUG_ENABLE non défini
 * @note  Toujours déclarée (compilable sans SGP40_DEBUG_ENABLE) — les exemples
 *        peuvent appeler cette fonction sans définir de flag localement.
 *        Corps conditionnel dans STM32_SGP40.c : strings Flash présentes uniquement
 *        si SGP40_DEBUG_ENABLE est actif (conf.h ou -D compilateur).
 */
const char *SGP40_StatusToString(SGP40_Status code);

/* =============================================================================
 * Fonctions convenience (warmup + mesure tout-en-un)
 * ============================================================================= */

/**
 * @brief Vérifie si la phase de warmup de l'algorithme VOC est terminée
 * @param hsgp40 Handle SGP40 (non NULL)
 * @retval true si measure_count >= GasIndexAlgorithm_INITIAL_BLACKOUT
 * @note  Évite l'accès direct à la constante Sensirion dans le code utilisateur.
 */
bool SGP40_IsVOCWarmupComplete(const SGP40_Handle_t *hsgp40);

/**
 * @brief Retourne le nombre d'échantillons nécessaires au warmup de l'algo VOC
 * @param hsgp40 Handle SGP40 (non NULL) — intervalle utilisé pour le calcul
 * @retval ceil(45 s / sample_interval_s) — ex. 45 @ 1 Hz, 90 @ 500 ms, 23 @ 2 s
 * @note   Retourne 45 (valeur 1 Hz) si hsgp40 est NULL.
 */
uint32_t SGP40_GetVOCWarmupSamples(const SGP40_Handle_t *hsgp40);

/**
 * @brief Mesure VOC raw + calcul VOC Index en un seul appel
 * @param hsgp40    Handle SGP40 (non NULL, initialisé)
 * @param voc_raw   Pointeur pour stocker signal VOC raw (peut être NULL)
 * @param voc_index Pointeur pour stocker VOC index 0-500 (non NULL)
 * @retval SGP40_Status
 * @note  Bloquant ~SGP40_MEASURE_WAIT_MS. Combine MeasureRaw() + CalculateVOCIndex().
 */
SGP40_Status SGP40_MeasureVOCIndex(SGP40_Handle_t *hsgp40, uint16_t *voc_raw, uint16_t *voc_index);

/**
 * @brief Mesure complète VOC (raw + index) stockée dans une struct métier
 * @param hsgp40   Handle SGP40 (non NULL, initialisé)
 * @param data_out Pointeur vers SGP40_Data à remplir (non NULL)
 * @retval SGP40_Status
 * @note  Bloquant ~SGP40_MEASURE_WAIT_MS. Wrapper de SGP40_MeasureVOCIndex().
 * @pre   hsgp40 non NULL, data_out non NULL
 * @post  data_out->voc_raw et data_out->voc_index remplis si SGP40_OK
 */
SGP40_Status SGP40_ReadAll(SGP40_Handle_t *hsgp40, SGP40_Data *data_out);

/* =============================================================================
 * API Asynchrone (IT)
 * ============================================================================= */

/**
 * @brief Initialise contexte asynchrone
 * @param ctx    Contexte async à initialiser
 * @param hsgp40 Handle SGP40 synchrone (doit être déjà initialisé)
 * @note  Réinitialise tous les flags et callbacks.
 */
void SGP40_Async_Init(SGP40_Async_t *ctx, SGP40_Handle_t *hsgp40);

/**
 * @brief Reset la machine d'état async sans perdre les callbacks/timebase
 * @param ctx Contexte async (non NULL)
 * @pre   ctx non NULL (comportement indéfini sinon)
 * @post  ctx->state == SGP40_ASYNC_IDLE
 * @post  ctx->hsgp40->async_busy == 0  (libère le guard polling)
 * @note  Préserve : hsgp40, hi2c, tous les callbacks, user_ctx.
 *        Remet : state=IDLE, flags=false, buffers=0, deadlines=0.
 *        Utile pour récupération d'erreur sans ré-appeler Init+SetCallbacks.
 * @warning ⚠️ Abandonne toute mesure en cours — appeler uniquement depuis main loop.
 */
void SGP40_Async_Reset(SGP40_Async_t *ctx);

/**
 * @brief Configure callbacks utilisateur (contexte main loop)
 * @param ctx          Contexte async
 * @param on_data_ready Callback données prêtes (peut être NULL)
 * @param on_error     Callback erreur (peut être NULL)
 * @param user_ctx     Pointeur utilisateur passé aux callbacks
 */
void SGP40_Async_SetCallbacks(SGP40_Async_t *ctx,
                               SGP40_Async_OnDataReadyCb on_data_ready,
                               SGP40_Async_OnErrorCb on_error,
                               void *user_ctx);

/**
 * @brief Configure callbacks IRQ-safe (depuis interruptions I2C)
 * @param ctx              Contexte async
 * @param on_irq_data_ready Callback IRQ données prêtes (peut être NULL)
 * @param on_irq_error     Callback IRQ erreur (peut être NULL)
 * @param user_ctx         Pointeur utilisateur passé aux callbacks
 * @warning Ces callbacks sont appelés depuis interruption — garder ultra-courts !
 */
void SGP40_Async_SetIrqCallbacks(SGP40_Async_t *ctx,
                                  SGP40_Async_OnIrqDataReadyCb on_irq_data_ready,
                                  SGP40_Async_OnIrqErrorCb on_irq_error,
                                  void *user_ctx);

/**
 * @brief Lecture flag données prêtes
 * @param ctx Contexte async
 * @retval true si données disponibles
 */
bool SGP40_Async_DataReadyFlag(const SGP40_Async_t *ctx);

/**
 * @brief Lecture flag erreur
 * @param ctx Contexte async
 * @retval true si erreur survenue
 */
bool SGP40_Async_ErrorFlag(const SGP40_Async_t *ctx);

/**
 * @brief Clear flags data_ready et error
 * @param ctx Contexte async
 */
void SGP40_Async_ClearFlags(SGP40_Async_t *ctx);

/**
 * @brief Vérifie si machine d'état idle
 * @param ctx Contexte async
 * @retval true si IDLE (pas de mesure en cours)
 */
bool SGP40_Async_IsIdle(const SGP40_Async_t *ctx);

/**
 * @brief Vérifie si données disponibles
 * @param ctx Contexte async
 * @retval true si état DONE avec données valides
 */
bool SGP40_Async_HasData(const SGP40_Async_t *ctx);

/**
 * @brief Déclenche mesure VOC en mode IT (non-bloquant)
 * @param ctx Contexte async (non NULL, SGP40_Async_Init() préalablement appelé)
 * @pre   ctx->hsgp40->initialized == true  (SGP40_Init() réussi)
 * @pre   NVIC activé pour le périphérique I2C utilisé
 * @pre   HAL_GetTick() monotone (non réinitialisé entre deux appels)
 * @post  ctx->state == SGP40_ASYNC_TX (ou SGP40_ASYNC_IDLE si HAL_BUSY — normal sur bus partagé)
 * @retval SGP40_OK             mesure lancée
 * @retval SGP40_ERR_BUSY       FSM non idle (mesure déjà en cours)
 * @retval SGP40_ERR_I2C        erreur HAL I2C (hors HAL_BUSY)
 * @note  Utilise HAL_I2C_Master_Transmit_IT. DMA non implémenté : aucun gain
 *        pour les trames SGP40 (8B TX / 3B RX) — IT suffit sans surcoût DMA.
 */
SGP40_Status SGP40_ReadAll_IT(SGP40_Async_t *ctx);

/**
 * @brief Machine d'état async — à appeler dans main loop
 * @param ctx    Contexte async (non NULL)
 * @param now_ms Temps actuel HAL_GetTick()
 * @pre   now_ms monotone sur Cortex-M (uint32_t, wraparound géré par soustraction non signée)
 * @pre   Appelé exclusivement depuis la main loop (jamais depuis IRQ)
 * @retval None — consulter ctx->last_status ou SGP40_Async_GetTickResult() pour le statut
 * @note  Gère transitions d'état, timeouts, et appelle callbacks utilisateur.
 *        Ne remet pas automatiquement l'état à IDLE après DONE.
 */
void SGP40_Async_Process(SGP40_Async_t *ctx, uint32_t now_ms);

/**
 * @brief Récupère dernière mesure VOC raw
 * @param ctx     Contexte async
 * @param voc_raw Pointeur pour stocker VOC raw
 * @retval SGP40_OK              si données valides (état DONE)
 * @retval SGP40_ERR_NOT_CONFIGURED  si IDLE (aucune mesure lancée)
 * @retval SGP40_ERR_BUSY        si transfert en cours (état non-DONE, non-IDLE)
 * @retval ctx->last_status      si état ERROR (propage l'erreur réelle)
 * @note  Consomme la mesure et remet l'état à IDLE après lecture.
 */
SGP40_Status SGP40_Async_GetData(SGP40_Async_t *ctx, uint16_t *voc_raw);

/**
 * @brief Callback HAL - Tx I2C terminé (mode IT)
 * @param ctx  Contexte async (non NULL)
 * @param hi2c Handle I2C ayant déclenché l'IRQ
 * @pre   Appelé exclusivement depuis le contexte IRQ HAL (HAL_I2C_MasterTxCpltCallback)
 * @post  ctx->state == SGP40_ASYNC_RX si hi2c correspond, inchangé sinon
 * @note  À appeler depuis HAL_I2C_MasterTxCpltCallback.
 *        Filtre sur hi2c : ignore si différent du handle configuré.
 */
void SGP40_Async_OnI2CMasterTxCplt(SGP40_Async_t *ctx, I2C_HandleTypeDef *hi2c);

/**
 * @brief Callback HAL - Rx I2C terminé (mode IT)
 * @param ctx  Contexte async (non NULL)
 * @param hi2c Handle I2C ayant déclenché l'IRQ
 * @pre   Appelé exclusivement depuis le contexte IRQ HAL (HAL_I2C_MasterRxCpltCallback)
 * @pre   ctx->rx_buf dimensionné à ≥ 3 octets (2B data + 1B CRC — §4.5 SGP40 datasheet)
 * @post  notify_data_pending == true (si CRC OK) ou notify_error_pending == true
 * @note  À appeler depuis HAL_I2C_MasterRxCpltCallback.
 *        Filtre sur hi2c : ignore si différent du handle configuré.
 */
void SGP40_Async_OnI2CMasterRxCplt(SGP40_Async_t *ctx, I2C_HandleTypeDef *hi2c);

/**
 * @brief Callback HAL - Erreur I2C
 * @param ctx  Contexte async (non NULL)
 * @param hi2c Handle I2C ayant déclenché l'IRQ
 * @pre   Appelé exclusivement depuis le contexte IRQ HAL (HAL_I2C_ErrorCallback)
 * @post  notify_error_pending == true si hi2c correspond
 * @note  À appeler depuis HAL_I2C_ErrorCallback.
 *        Filtre sur hi2c : ignore si différent du handle configuré.
 */
void SGP40_Async_OnI2CError(SGP40_Async_t *ctx, I2C_HandleTypeDef *hi2c);

/* =============================================================================
 * Helpers haut niveau (réduisent le boilerplate utilisateur)
 * ============================================================================= */

/**
 * @brief Avance la FSM, récupère les données si prêtes, remet à IDLE si erreur.
 *        Remplace Process() + flag polling + GetData() + ClearFlags() + Reset().
 * @param ctx         Contexte async
 * @param now_ms      HAL_GetTick()
 * @param voc_raw_out Sortie VOC raw si TICK_DATA_READY (peut être NULL)
 * @retval SGP40_TICK_DATA_READY  données prêtes dans *voc_raw_out
 *         SGP40_TICK_ERROR       erreur — FSM remise à IDLE automatiquement
 *         SGP40_TICK_BUSY        mesure en cours
 *         SGP40_TICK_IDLE        rien en cours
 */
SGP40_TickResult SGP40_Async_Tick(SGP40_Async_t *ctx, uint32_t now_ms, uint16_t *voc_raw_out);

/**
 * @brief Déclenche ReadAll_IT() si IDLE && intervalle écoulé (stocké dans hsgp40).
 *        Met à jour *last_ms uniquement si le trigger réussit.
 * @note  Compatible partage de bus I2C multi-capteurs : si le bus est occupé
 *        (autre capteur en transfert), retente automatiquement sur les appels suivants.
 * @param ctx     Contexte async
 * @param now_ms  HAL_GetTick()
 * @param last_ms Pointeur vers le timestamp du dernier trigger (mis à jour)
 * @retval SGP40_OK          trigger lancé, OU délai non encore écoulé (pas une erreur)
 *         SGP40_ERR_BUSY    FSM non idle (mesure déjà en cours)
 *         autre             erreur I2C/config
 */
SGP40_Status SGP40_Async_TriggerEvery(SGP40_Async_t *ctx, uint32_t now_ms,
                                       uint32_t *last_ms);

/**
 * @brief Tick + calcul VOC Index en un seul appel — réduit le boilerplate while(1).
 * @details Remplace le pattern `SGP40_Async_Tick() + SGP40_CalculateVOCIndex()` par
 *          un seul appel. Si la donnée n'est pas encore prête, retourne TICK_IDLE ou
 *          TICK_BUSY sans effet de bord.
 * @note  Compatible multi-lib IT : grâce au fix HAL_BUSY TX, la FSM reste IDLE si
 *        le bus est occupé par une autre lib (STM32_AHT20, STM32_BME280...).
 *        TICK_ERROR n'est donc émis que pour de vraies erreurs I2C ou de timeout.
 * @param ctx           Contexte async (non NULL)
 * @param now_ms        HAL_GetTick()
 * @param voc_raw_out   Sortie VOC brut en ticks (peut être NULL)
 * @param voc_index_out Sortie VOC Index 0–500 (non NULL)
 * @retval SGP40_TICK_DATA_READY  index valide dans *voc_index_out
 *         SGP40_TICK_IDLE        FSM idle, pas de mesure en cours
 *         SGP40_TICK_BUSY        mesure en cours, résultat pas encore prêt
 *         SGP40_TICK_ERROR       erreur I2C ou algo VOC, voir ctx->last_status
 */
SGP40_TickResult SGP40_Async_TickIndex(SGP40_Async_t *ctx, uint32_t now_ms,
                                       uint16_t *voc_raw_out, uint16_t *voc_index_out);

#ifdef __cplusplus
}
#endif

#endif /* STM32_SGP40_H */

/**
  ******************************************************************************
  * @file    STM32_SGP40.c
  * @brief   Implémentation driver STM32 HAL pour capteur VOC SGP40 (Sensirion)
  * @author  Generated via STM32_LIB_STYLE_GUIDE.md
  * @date    2026-02-11
  * @version 0.9.0
  * @copyright Libre sous licence MIT.
  ******************************************************************************
  */

#include "STM32_SGP40.h"
#include "sensirion_gas_index_algorithm.h"  ///< Header vendor — inclus ici uniquement, pas dans le .h public
#include <math.h>
#include <string.h>

/* ============================================================================
 * Private defines
 * ============================================================================ */
#define SGP40_SERIAL_SIZE       9       ///< Serial: 3 words (2 bytes + CRC chaque)
#define SGP40_TEST_OK_VALUE     0xD400U ///< Valeur attendue auto-test (datasheet §5.2 : octet bas toujours 0x00)
#define SGP40_TEST_FAIL_VALUE   0x4B00U ///< Valeur échec auto-test (datasheet §5.2)

/** @brief Accès typé au buffer opaque algo VOC dans un handle SGP40.
 *  Évite l'exposition de GasIndexAlgorithmParams dans le .h public.
 */
#define SGP40_VOC_ALGO(h) ((GasIndexAlgorithmParams *)(void *)((h)->_voc_algo_opaque))

/* Vérifie à la compilation que SGP40_VOC_ALGO_OPAQUE_SIZE == sizeof(GasIndexAlgorithmParams).
 * En cas d'erreur : mettre à jour SGP40_VOC_ALGO_OPAQUE_SIZE dans STM32_SGP40.h. */
_Static_assert(sizeof(GasIndexAlgorithmParams) == SGP40_VOC_ALGO_OPAQUE_SIZE,
               "SGP40_VOC_ALGO_OPAQUE_SIZE ne correspond pas a sizeof(GasIndexAlgorithmParams) — mettre a jour STM32_SGP40.h");

/* ============================================================================
 * Private function prototypes
 * ============================================================================ */
static uint8_t SGP40_CalculateCRC8(const uint8_t *data, size_t len);
static SGP40_Status SGP40_MapHalStatus(HAL_StatusTypeDef hal_status);
static SGP40_Status SGP40_WriteCommand(SGP40_Handle_t *hsgp40, uint16_t cmd);
static SGP40_Status SGP40_WriteCommandWithData(SGP40_Handle_t *hsgp40, uint16_t cmd, const uint8_t *data, size_t data_len);
static SGP40_Status SGP40_ReadData(SGP40_Handle_t *hsgp40, uint8_t *data, size_t len);
static uint16_t SGP40_TempToRaw(float temp_c);
static uint16_t SGP40_RHToRaw(float rh_percent);
static SGP40_Status SGP40_Async_TriggerMeasure_Common(SGP40_Async_t *ctx);

/* ============================================================================
 * Private functions
 * ============================================================================ */

/**
 * @brief Calcule CRC-8 (polynomial Sensirion 0x31)
 * @param data Données à vérifier
 * @param len  Longueur
 * @retval CRC8
 */
static uint8_t SGP40_CalculateCRC8(const uint8_t *data, size_t len) {
    uint8_t crc = SGP40_CRC8_INIT;                    // Valeur initiale 0xFF (Sensirion)

    for (size_t i = 0; i < len; i++) {                // Itère sur chaque octet d'entrée
        crc ^= data[i];                                // XOR avec l'octet courant
        for (uint8_t bit = 0; bit < 8; bit++) {        // Traite les 8 bits de l'octet
            if (crc & 0x80) {                          // Si bit de poids fort = 1
                crc = (crc << 1) ^ SGP40_CRC8_POLYNOMIAL; // Décale + applique le polynôme
            } else {                                   // Sinon
                crc <<= 1;                             // Décale sans XOR
            }
        }
    }

    return crc;                                        // Retourne le CRC-8 calculé
}

/**
 * @brief Convertit un statut HAL I2C en code statut SGP40
 * @param hal_status Code de retour HAL
 * @retval SGP40_Status
 */
static SGP40_Status SGP40_MapHalStatus(HAL_StatusTypeDef hal_status) {
    if (hal_status == HAL_BUSY) {
        return SGP40_ERR_BUSY;
    }
    if (hal_status == HAL_TIMEOUT) {
        return SGP40_ERR_TIMEOUT;
    }
    return SGP40_ERR_I2C;
}

/**
 * @brief Envoie commande 16 bits (MSB first)
 * @param hsgp40 Handle SGP40
 * @param cmd    Commande 16 bits
 * @retval SGP40_Status
 * @note  Met à jour consecutive_errors et last_error dans le handle.
 */
static SGP40_Status SGP40_WriteCommand(SGP40_Handle_t *hsgp40, uint16_t cmd) {
    if (!hsgp40 || !hsgp40->hi2c) {                   // Vérifie handle valide
        return SGP40_ERR_NULL_PTR;
    }

    uint8_t buf[2];                                    // Buffer 2 octets pour la commande
    buf[0] = (cmd >> 8) & 0xFF;                        // Octet de poids fort (MSB)
    buf[1] = cmd & 0xFF;                               // Octet de poids faible (LSB)

    HAL_StatusTypeDef hal_status = HAL_I2C_Master_Transmit(  // Envoi I2C bloquant
        hsgp40->hi2c,                                  // Handle I2C HAL
        (uint16_t)(hsgp40->i2c_addr_7b << 1),          // Adresse esclave (7-bit convertie HAL)
        buf,                                           // Données à envoyer
        2,                                             // 2 octets = 1 commande Sensirion
        hsgp40->i2c_timeout                            // Timeout configuré dans le handle
    );

    if (hal_status != HAL_OK) {                        // Erreur de transmission
        hsgp40->last_hal_error = HAL_I2C_GetError(hsgp40->hi2c); // Code HAL bas niveau
        hsgp40->consecutive_errors++;                  // Erreur consécutive comptabilisée
        hsgp40->last_error = SGP40_MapHalStatus(hal_status);
        return hsgp40->last_error;
    }

    hsgp40->last_hal_error = 0U;
    hsgp40->consecutive_errors = 0;                    // Succès : remet le compteur à zéro
    return SGP40_OK;
}

/**
 * @brief Envoie commande + données avec CRC
 * @param hsgp40   Handle SGP40
 * @param cmd      Commande 16 bits
 * @param data     Données (format: MSB, LSB, CRC, MSB, LSB, CRC, ...)
 * @param data_len Longueur données (multiple de 3: 2 bytes + CRC)
 * @retval SGP40_Status
 * @note  Met à jour consecutive_errors et last_error dans le handle.
 */
static SGP40_Status SGP40_WriteCommandWithData(SGP40_Handle_t *hsgp40, uint16_t cmd, const uint8_t *data, size_t data_len) {
    if (!hsgp40 || !hsgp40->hi2c || !data) {           // Vérifie tous les pointeurs
        return SGP40_ERR_NULL_PTR;
    }

    if (data_len % 3 != 0) {                           // Format Sensirion: multiples de 3 (word + CRC)
        return SGP40_ERR_INVALID_PARAM;
    }

    uint8_t buf[SGP40_CMD_DATA_BUF_SIZE];              // Buffer local : commande + données
    if (data_len + 2 > sizeof(buf)) {                  // Vérifie que les données rentrent dans le buffer
        return SGP40_ERR_INVALID_PARAM;
    }

    buf[0] = (cmd >> 8) & 0xFF;                        // Commande MSB en premier
    buf[1] = cmd & 0xFF;                               // Commande LSB
    memcpy(&buf[2], data, data_len);                   // Copie les paramètres (words + CRCs)

    HAL_StatusTypeDef hal_status = HAL_I2C_Master_Transmit(  // Envoi commande + paramètres
        hsgp40->hi2c,
        (uint16_t)(hsgp40->i2c_addr_7b << 1),
        buf,
        2 + data_len,
        hsgp40->i2c_timeout
    );

    if (hal_status != HAL_OK) {                        // Erreur de transmission
        hsgp40->last_hal_error = HAL_I2C_GetError(hsgp40->hi2c);
        hsgp40->consecutive_errors++;
        hsgp40->last_error = SGP40_MapHalStatus(hal_status);
        return hsgp40->last_error;
    }

    hsgp40->last_hal_error = 0U;
    hsgp40->consecutive_errors = 0;                    // Succès TX
    return SGP40_OK;
}

/**
 * @brief Lit données avec vérification CRC (format: word0_MSB, word0_LSB, CRC0, ...)
 * @param hsgp40 Handle SGP40
 * @param data   Buffer réception (doit contenir mots + CRCs)
 * @param len    Longueur totale (multiple de 3)
 * @retval SGP40_Status
 * @note  Met à jour consecutive_errors et last_error dans le handle.
 */
static SGP40_Status SGP40_ReadData(SGP40_Handle_t *hsgp40, uint8_t *data, size_t len) {
    if (!hsgp40 || !hsgp40->hi2c || !data) {           // Vérifie tous les pointeurs
        return SGP40_ERR_NULL_PTR;
    }

    if (len % 3 != 0) {                                // Format Sensirion: multiples de 3 (word + CRC)
        return SGP40_ERR_INVALID_PARAM;
    }

    HAL_StatusTypeDef hal_status = HAL_I2C_Master_Receive(  // Réception I2C bloquante
        hsgp40->hi2c,
        (uint16_t)(hsgp40->i2c_addr_7b << 1),
        data,
        len,
        hsgp40->i2c_timeout
    );

    if (hal_status != HAL_OK) {                        // Erreur de réception
        hsgp40->last_hal_error = HAL_I2C_GetError(hsgp40->hi2c);
        hsgp40->consecutive_errors++;
        hsgp40->last_error = SGP40_MapHalStatus(hal_status);
        return hsgp40->last_error;
    }

    // Vérification CRC pour chaque mot (2 bytes + CRC)
    for (size_t i = 0; i < len; i += 3) {             // Avance de 3 en 3 (word + CRC)
        uint8_t crc_calc = SGP40_CalculateCRC8(&data[i], 2);  // CRC sur les 2 octets du mot
        if (crc_calc != data[i + 2]) {                 // Compare avec le CRC reçu
            hsgp40->consecutive_errors++;
            hsgp40->last_error = SGP40_ERR_CRC;
            return SGP40_ERR_CRC;
        }
    }

    hsgp40->last_hal_error = 0U;
    hsgp40->consecutive_errors = 0;                    // Toutes données valides : reset erreurs
    return SGP40_OK;
}

/**
 * @brief Conversion température °C → raw Sensirion
 * @param temp_c Température (°C)
 * @retval Raw value (uint16_t)
 * @note  Format: raw = (T + 45) * 65535 / 175
 */
static uint16_t SGP40_TempToRaw(float temp_c) {
    // Clamp sur la plage physique capteur [-45°C, +130°C]
    if (temp_c < -45.0f) temp_c = -45.0f;             // Limite basse datasheet
    if (temp_c > 130.0f) temp_c = 130.0f;             // Limite haute datasheet

    return (uint16_t)((temp_c + 45.0f) * 65535.0f / 175.0f);  // Formule Sensirion => [0, 65535]
}

/**
 * @brief Conversion humidité % → raw Sensirion
 * @param rh_percent Humidité relative (%)
 * @retval Raw value (uint16_t)
 * @note  Format: raw = RH% * 65535 / 100
 */
static uint16_t SGP40_RHToRaw(float rh_percent) {
    // Clamp sur la plage physique [0%, 100%]
    if (rh_percent < 0.0f)   rh_percent = 0.0f;       // Limite basse (0% HR)
    if (rh_percent > 100.0f) rh_percent = 100.0f;     // Limite haute (100% HR)

    return (uint16_t)(rh_percent * 65535.0f / 100.0f);  // Formule Sensirion => [0, 65535]
}

/* ============================================================================
 * Public functions
 * ============================================================================ */

/**
 * @brief Initialise le handle SGP40 avec les valeurs par défaut
 * @param hsgp40 Handle SGP40 (non NULL)
 * @param hi2c   Handle I2C HAL (non NULL)
 * @retval SGP40_Status
 * @note  Initialise tous les champs du handle avec les valeurs par défaut,
 *        lance l'algo VOC et effectue une première mesure silencieuse (prime).
 */
SGP40_Status SGP40_Init(SGP40_Handle_t *hsgp40, I2C_HandleTypeDef *hi2c) {
    if (!hsgp40 || !hi2c) {                            // Vérifie les deux pointeurs obligatoires
        return SGP40_ERR_NULL_PTR;
    }

    // Remplissage complet du handle avec les valeurs par défaut
    hsgp40->hi2c                    = hi2c;
    hsgp40->i2c_addr_7b             = SGP40_I2C_ADDR_7B;
    hsgp40->i2c_timeout             = SGP40_DEFAULT_TIMEOUT_MS;
    hsgp40->temp_c                  = SGP40_DEFAULT_TEMP_C;
    hsgp40->rh_percent              = SGP40_DEFAULT_RH_PERCENT;
    hsgp40->sample_interval_ms      = SGP40_DEFAULT_SAMPLE_INTERVAL_MS;
    hsgp40->measurement_mode_active = false;
    hsgp40->last_measure_tick_ms    = 0U;
    hsgp40->measure_count           = 0U;
    hsgp40->voc_algo_initialized    = false;
    hsgp40->last_voc_raw            = 0U;
    // Suivi erreurs — remis à zéro au démarrage
    hsgp40->consecutive_errors      = 0U;
    hsgp40->last_error              = SGP40_OK;
    hsgp40->last_hal_error          = 0U;
    hsgp40->async_busy              = 0U;

    // Initialise l'algorithme VOC Sensirion
    GasIndexAlgorithm_init_with_sampling_interval(
        SGP40_VOC_ALGO(hsgp40),
        GasIndexAlgorithm_ALGORITHM_TYPE_VOC,
        ((float)hsgp40->sample_interval_ms) / 1000.0f
    );
    hsgp40->voc_algo_initialized = true;

    // Vérification présence capteur (lecture serial number)
    SGP40_Status status = SGP40_GetSerialNumber(hsgp40, &hsgp40->serial_number);
    if (status != SGP40_OK) {
        return status;
    }

    hsgp40->initialized = true;

    // Première mesure silencieuse ("prime") pour activer l'algo VOC dès le démarrage
    uint16_t raw_discard = 0U;
    SGP40_MeasureRaw(hsgp40, &raw_discard);            // Échec ignoré volontairement
    // Remet les deux champs à l'état initial : la prime interne ne doit pas bloquer
    // PrimeForVocIndex() ni la première mesure utilisateur via la garde de cadence.
    hsgp40->measurement_mode_active = false;
    hsgp40->last_measure_tick_ms    = 0U;

    return SGP40_OK;
}

/**
 * @brief Réinitialise le handle SGP40 (libère ressources logicielles)
 * @param hsgp40 Handle SGP40 (non NULL)
 * @retval SGP40_Status
 */
SGP40_Status SGP40_DeInit(SGP40_Handle_t *hsgp40) {
    if (!hsgp40) {
        return SGP40_ERR_NULL_PTR;
    }

    if (hsgp40->async_busy) {                          // Refuse si un transfert IT est en cours
        return SGP40_ERR_BUSY;
    }

    memset(hsgp40, 0, sizeof(SGP40_Handle_t));         // Remet tout à zéro : initialized=false, hi2c=NULL, etc.
    return SGP40_OK;
}

/**
 * @brief Lecture serial number 48 bits
 * @param hsgp40 Handle SGP40
 * @param serial Pointeur pour stocker serial (6 octets)
 * @retval SGP40_Status
 */
SGP40_Status SGP40_GetSerialNumber(SGP40_Handle_t *hsgp40, uint64_t *serial) {
    if (!hsgp40 || !serial) {                          // Vérifie les deux pointeurs
        return SGP40_ERR_NULL_PTR;
    }

    *serial = 0U;

    if (hsgp40->async_busy) {                          // Guard : interdit si async en cours
        return SGP40_ERR_BUSY;
    }

    // Envoi commande Get Serial Number
    SGP40_Status status = SGP40_WriteCommand(hsgp40, SGP40_CMD_GET_SERIAL);
    if (status != SGP40_OK) {
        return status;
    }

    HAL_Delay(SGP40_SERIAL_READ_DELAY_MS);  // datasheet §5.3 : délai post-commande < 1 ms

    // Lecture 9 bytes: 3 words (2 bytes + CRC chaque)
    uint8_t buf[SGP40_SERIAL_SIZE];
    status = SGP40_ReadData(hsgp40, buf, SGP40_SERIAL_SIZE);
    if (status != SGP40_OK) {
        return status;
    }

    // Reconstruction serial 48 bits (6 octets utiles, octets CRC ignorés)
    *serial = 0;
    *serial |= ((uint64_t)buf[0] << 40);               // Word 0 : MSB (bits 47-40)
    *serial |= ((uint64_t)buf[1] << 32);               // Word 0 : LSB (bits 39-32)
    *serial |= ((uint64_t)buf[3] << 24);               // Word 1 : MSB (bits 31-24) [buf[2]=CRC]
    *serial |= ((uint64_t)buf[4] << 16);               // Word 1 : LSB (bits 23-16)
    *serial |= ((uint64_t)buf[6] << 8);                // Word 2 : MSB (bits 15-8)  [buf[5]=CRC]
    *serial |= ((uint64_t)buf[7]);                     // Word 2 : LSB (bits 7-0)

    return SGP40_OK;
}

/**
 * @brief Auto-test capteur (heater + sensor)
 * @param hsgp40      Handle SGP40
 * @param test_result Résultat brut — SGP40_TEST_RESULT_OK (0xD400) = OK,
 *                    SGP40_TEST_RESULT_FAIL (0x4B00) = échec.
 *                    Datasheet §5.2 : octet bas toujours 0x00.
 * @retval SGP40_OK            si *test_result == 0xD400
 * @retval SGP40_ERR_SELF_TEST si *test_result != 0xD400
 * @note  Bloquant jusqu'à SGP40_SELFTEST_WAIT_MS (320 ms).
 */
SGP40_Status SGP40_SelfTest(SGP40_Handle_t *hsgp40, uint16_t *test_result) {
    if (!hsgp40 || !test_result) {
        return SGP40_ERR_NULL_PTR;
    }

    *test_result = 0U;

    if (hsgp40->async_busy) {                          // Guard : interdit si async en cours
        return SGP40_ERR_BUSY;
    }

    SGP40_Status status = SGP40_WriteCommand(hsgp40, SGP40_CMD_MEASURE_TEST);
    if (status != SGP40_OK) {
        return status;
    }

    HAL_Delay(SGP40_SELFTEST_WAIT_MS);                 // Attente 320ms (durée max datasheet)

    uint8_t buf[3];                                    // MSB, LSB, CRC
    status = SGP40_ReadData(hsgp40, buf, 3);
    if (status != SGP40_OK) {
        return status;
    }

    *test_result = (uint16_t)((buf[0] << 8) | buf[1]); // Assemble résultat 16 bits

    if (*test_result != SGP40_TEST_OK_VALUE) {              // vérifie valeur exacte 0xD400 (datasheet §5.2 : octet bas toujours 0x00)
        hsgp40->consecutive_errors++;
        hsgp40->last_error = SGP40_ERR_SELF_TEST;
        return SGP40_ERR_SELF_TEST;
    }

    hsgp40->consecutive_errors = 0;                    // Auto-test réussi
    return SGP40_OK;
}

/**
 * @brief Mesure VOC raw (signal brut 0-65535)
 * @param hsgp40  Handle SGP40
 * @param voc_raw Pointeur pour stocker signal VOC raw
 * @retval SGP40_Status
 * @note  Bloquant ~SGP40_MEASURE_WAIT_MS (30 ms), utilise compensation T/RH configurée dans handle.
 *        Retourne SGP40_ERR_BUSY si async en cours ou cadence non respectée.
 */
SGP40_Status SGP40_MeasureRaw(SGP40_Handle_t *hsgp40, uint16_t *voc_raw) {
    if (!hsgp40 || !voc_raw) {
        return SGP40_ERR_NULL_PTR;
    }

    *voc_raw = 0U;

    if (!hsgp40->initialized) {
        return SGP40_ERR_NOT_INITIALIZED;              // SGP40_Init() non appelé ou HeaterOff() effectué
    }

    if (hsgp40->async_busy) {                          // Guard : interdit si async en cours
        return SGP40_ERR_BUSY;
    }

    uint32_t now_ms = HAL_GetTick();
    if (hsgp40->measurement_mode_active) {             // Si déjà en mode mesure (au moins 1 sample)
        uint32_t elapsed = now_ms - hsgp40->last_measure_tick_ms;
        if (elapsed < hsgp40->sample_interval_ms) {    // Trop tôt : cadence non respectée
            return SGP40_ERR_BUSY;
        }
    }

    // Préparation données de compensation : RH (word + CRC), T° (word + CRC)
    uint16_t rh_raw   = SGP40_RHToRaw(hsgp40->rh_percent);
    uint16_t temp_raw = SGP40_TempToRaw(hsgp40->temp_c);

    uint8_t data[6];
    data[0] = (rh_raw >> 8) & 0xFF;                   // HR MSB
    data[1] = rh_raw & 0xFF;                           // HR LSB
    data[2] = SGP40_CalculateCRC8(&data[0], 2);        // CRC sur les 2 octets HR
    data[3] = (temp_raw >> 8) & 0xFF;                  // T° MSB
    data[4] = temp_raw & 0xFF;                         // T° LSB
    data[5] = SGP40_CalculateCRC8(&data[3], 2);        // CRC sur les 2 octets T°

    /* Mémoriser le tick AVANT le délai bloquant : cohérence avec le mode async
     * où last_measure_tick_ms est pris avant le transfert IT.
     * L'intervalle de cadence est mesuré depuis le déclenchement de la mesure,
     * pas depuis sa fin, pour un résultat identique quel que soit le mode. */
    hsgp40->measurement_mode_active = true;            // Active la garde de cadence
    hsgp40->last_measure_tick_ms    = now_ms;          // Tick pris AVANT HAL_Delay (cohérence async)

    SGP40_Status status = SGP40_WriteCommandWithData(hsgp40, SGP40_CMD_MEASURE_RAW, data, 6);
    if (status != SGP40_OK) {
        hsgp40->measurement_mode_active = false;       // Annule la garde si TX échoue
        hsgp40->last_measure_tick_ms    = 0U;
        return status;
    }

    HAL_Delay(SGP40_MEASURE_WAIT_MS);                  // Attente conversion ~30ms (datasheet)

    uint8_t buf[3];
    status = SGP40_ReadData(hsgp40, buf, 3);
    if (status != SGP40_OK) {
        return status;
    }

    *voc_raw = (uint16_t)((buf[0] << 8) | buf[1]);     // Assemble valeur VOC raw 16 bits
    hsgp40->last_voc_raw = *voc_raw;                   // Sauvegarde pour accès rapide
    hsgp40->measure_count++;                           // Incrémente compteur warmup algo

    return SGP40_OK;
}

/**
 * @brief Prime le capteur pour usage VOC Index (discard première mesure)
 * @param hsgp40 Handle SGP40
 * @retval SGP40_Status
 */
SGP40_Status SGP40_PrimeForVocIndex(SGP40_Handle_t *hsgp40) {
    if (!hsgp40) {
        return SGP40_ERR_NULL_PTR;
    }

    if (hsgp40->async_busy) {                          // Guard : interdit si async en cours
        return SGP40_ERR_BUSY;
    }

    // Bypass de la garde de cadence : PrimeForVocIndex est une opération d'init,
    // pas une mesure de boucle. Elle peut être appelée immédiatement après Init().
    hsgp40->measurement_mode_active = false;
    hsgp40->last_measure_tick_ms    = 0U;

    uint16_t discarded = 0;
    SGP40_Status status = SGP40_MeasureRaw(hsgp40, &discarded);

    // Après prime : remet à zéro pour ne pas bloquer la première mesure utilisateur.
    hsgp40->measurement_mode_active = false;
    hsgp40->last_measure_tick_ms    = 0U;

    return status;
}

/**
 * @brief Configure l'intervalle minimal entre mesures
 * @param hsgp40      Handle SGP40
 * @param interval_ms Intervalle en ms (plage supportée: 500..10000)
 * @retval SGP40_Status
 * @note  ⚠️  Réinitialise l'algorithme VOC Sensirion si déjà actif :
 *        l'état d'apprentissage accumulé (baseline, gain, offset) est perdu.
 *        Appeler uniquement en phase d'initialisation, avant toute acquisition.
 */
SGP40_Status SGP40_SetSampleInterval(SGP40_Handle_t *hsgp40, uint32_t interval_ms) {
    if (!hsgp40) {
        return SGP40_ERR_NULL_PTR;
    }

    if (interval_ms < 500U || interval_ms > 10000U) {  // Plage validée : 0.5s à 10s
        return SGP40_ERR_INVALID_PARAM;
    }

    hsgp40->sample_interval_ms = interval_ms;

    if (hsgp40->voc_algo_initialized) {                // Si l'algo est déjà actif : reconfigurer
        GasIndexAlgorithm_init_with_sampling_interval(
            SGP40_VOC_ALGO(hsgp40),
            GasIndexAlgorithm_ALGORITHM_TYPE_VOC,
            ((float)hsgp40->sample_interval_ms) / 1000.0f
        );
        /* L'algo VOC est ré-initialisé → le warmup doit recommencer.
         * Sans ce reset, IsVOCWarmupComplete() retournerait true à tort. */
        hsgp40->measure_count = 0U;
    }

    return SGP40_OK;
}

/**
 * @brief Configure compensation température/humidité
 * @param hsgp40     Handle SGP40
 * @param temp_c     Température (°C), plage -45 à +130
 * @param rh_percent Humidité relative (%), plage 0 à 100
 * @retval SGP40_Status
 */
SGP40_Status SGP40_SetCompensation(SGP40_Handle_t *hsgp40, float temp_c, float rh_percent) {
    if (!hsgp40) {
        return SGP40_ERR_NULL_PTR;
    }

    if (temp_c < -45.0f || temp_c > 130.0f) {
        return SGP40_ERR_INVALID_PARAM;
    }
    if (rh_percent < 0.0f || rh_percent > 100.0f) {
        return SGP40_ERR_INVALID_PARAM;
    }

    hsgp40->temp_c     = temp_c;
    hsgp40->rh_percent = rh_percent;

    return SGP40_OK;
}

/**
 * @brief Extinction heater (mode power-down)
 * @param hsgp40 Handle SGP40
 * @retval SGP40_Status
 * @note  Réduit consommation ~0.1µA, nécessite SGP40_Init() après réutilisation.
 */
SGP40_Status SGP40_HeaterOff(SGP40_Handle_t *hsgp40) {
    if (!hsgp40) {
        return SGP40_ERR_NULL_PTR;
    }

    if (hsgp40->async_busy) {                          // Guard : interdit si async en cours
        return SGP40_ERR_BUSY;
    }

    SGP40_Status status = SGP40_WriteCommand(hsgp40, SGP40_CMD_HEATER_OFF);
    if (status == SGP40_OK) {
        hsgp40->initialized           = false;         // Nécessite un SGP40_Init() après réutilisation
        hsgp40->measurement_mode_active = false;
        hsgp40->last_measure_tick_ms  = 0U;
        hsgp40->measure_count         = 0U;
        hsgp40->voc_algo_initialized  = false;
    }

    return status;
}

/**
 * @brief Soft Reset via I2C General Call (réinitialise TOUS les périphériques I2C du bus)
 * @param hsgp40 Handle SGP40 (pour accéder à hi2c et i2c_timeout)
 * @retval SGP40_OK              Reset envoyé avec succès
 * @retval SGP40_ERR_NULL_PTR    hsgp40 NULL
 * @retval SGP40_ERR_NOT_CONFIGURED  hi2c non configuré
 * @retval SGP40_ERR_BUSY        async en cours
 * @retval SGP40_ERR_TIMEOUT     timeout I2C (bus bloqué)
 * @warning ⚠️  Reset I2C General Call (addr 0x00) : TOUS les périphériques I2C
 *          du bus sont réinitialisés, pas uniquement le SGP40.
 * @note   Implémente sensirion_i2c_general_call_reset() du driver Sensirion RPi.
 *         Certains périphériques n'ACK pas la General Call → HAL_ERROR peut
 *         survenir même si le reset a été exécuté (retour SGP40_OK conservatoire).
 *         Après appel, le SGP40 est en état POR → appeler SGP40_Init() avant usage.
 */
SGP40_Status SGP40_SoftReset(SGP40_Handle_t *hsgp40) {
    if (!hsgp40) {
        return SGP40_ERR_NULL_PTR;
    }

    if (!hsgp40->hi2c) {
        return SGP40_ERR_NOT_CONFIGURED;
    }

    if (hsgp40->async_busy) {                          // Guard : interdit si async en cours
        return SGP40_ERR_BUSY;
    }

    /* General Call Reset : I2C addr=0x00 (7-bit) → DevAddress=0x00, data=0x06
     * Conforme NXP I2C spec §3.1.12 et driver Sensirion sensirion_i2c_general_call_reset().
     * Remet tous les périphériques I2C sur le bus à leur état POR. */
    const uint8_t reset_cmd = 0x06U;
    HAL_StatusTypeDef hal_status = HAL_I2C_Master_Transmit(
        hsgp40->hi2c,
        (uint16_t)0x00U,                               // General Call addr 7-bit = 0x00, HAL attend addr << 1 (déjà 0)
        (uint8_t *)&reset_cmd,
        1U,
        hsgp40->i2c_timeout
    );

    /* Invalide l'état du handle : le SGP40 a redémarré (POR) */
    hsgp40->initialized             = false;
    hsgp40->measurement_mode_active = false;
    hsgp40->last_measure_tick_ms    = 0U;
    hsgp40->measure_count           = 0U;
    hsgp40->voc_algo_initialized    = false;

    /* NACK sur General Call est normal (certains périphériques ne l'ACK pas).
     * HAL retourne HAL_ERROR avec I2C AF flag même si la trame a été émise.
     * On retourne SGP40_OK dès que le bus n'était pas en timeout. */
    if (hal_status == HAL_OK || hal_status == HAL_ERROR) {
        hsgp40->consecutive_errors = 0U;
        return SGP40_OK;
    }

    return SGP40_MapHalStatus(hal_status);             // Propage HAL_BUSY ou HAL_TIMEOUT
}

/**
 * @brief Calcule VOC Index depuis VOC raw (algorithme officiel Sensirion)
 * @param hsgp40    Handle SGP40 (contient l'état interne de l'algorithme)
 * @param voc_raw   Signal VOC raw (0-65535)
 * @param voc_index Pointeur pour stocker VOC index (0-500)
 * @retval SGP40_Status
 */
SGP40_Status SGP40_CalculateVOCIndex(SGP40_Handle_t *hsgp40, uint16_t voc_raw, uint16_t *voc_index) {
    if (!hsgp40 || !voc_index) {
        return SGP40_ERR_NULL_PTR;
    }

    if (!hsgp40->voc_algo_initialized) {
        return SGP40_ERR_NOT_READY;
    }

    int32_t gas_index = 0;
    GasIndexAlgorithm_process(SGP40_VOC_ALGO(hsgp40), (int32_t)voc_raw, &gas_index);

    if (gas_index < 0)   gas_index = 0;               // Clamp bas
    if (gas_index > 500) gas_index = 500;              // Clamp haut

    *voc_index = (uint16_t)gas_index;

    return SGP40_OK;
}

/**
 * @brief Reset des états internes de l'algorithme VOC Sensirion
 * @param hsgp40 Handle SGP40
 * @retval SGP40_Status
 */
SGP40_Status SGP40_VOCAlgoReset(SGP40_Handle_t *hsgp40) {
    if (!hsgp40) {
        return SGP40_ERR_NULL_PTR;
    }
    if (!hsgp40->voc_algo_initialized) {
        return SGP40_ERR_NOT_READY;
    }

    GasIndexAlgorithm_reset(SGP40_VOC_ALGO(hsgp40));
    return SGP40_OK;
}

/**
 * @brief Récupère les états VOC (pour persistance / reprise)
 */
SGP40_Status SGP40_VOCAlgoGetStates(const SGP40_Handle_t *hsgp40, float *state0, float *state1) {
    if (!hsgp40 || !state0 || !state1) {
        return SGP40_ERR_NULL_PTR;
    }
    if (!hsgp40->voc_algo_initialized) {
        return SGP40_ERR_NOT_READY;
    }

    GasIndexAlgorithm_get_states(SGP40_VOC_ALGO(hsgp40), state0, state1);
    return SGP40_OK;
}

/**
 * @brief Restaure les états VOC (reprise après interruption courte)
 */
SGP40_Status SGP40_VOCAlgoSetStates(SGP40_Handle_t *hsgp40, float state0, float state1) {
    if (!hsgp40) {
        return SGP40_ERR_NULL_PTR;
    }
    if (!hsgp40->voc_algo_initialized) {
        return SGP40_ERR_NOT_READY;
    }

    GasIndexAlgorithm_set_states(SGP40_VOC_ALGO(hsgp40), state0, state1);
    return SGP40_OK;
}

/**
 * @brief Configure les paramètres de tuning de l'algorithme VOC
 */
SGP40_Status SGP40_VOCAlgoSetTuning(
    SGP40_Handle_t *hsgp40,
    int32_t index_offset,
    int32_t learning_time_offset_hours,
    int32_t learning_time_gain_hours,
    int32_t gating_max_duration_minutes,
    int32_t std_initial,
    int32_t gain_factor
) {
    if (!hsgp40) {
        return SGP40_ERR_NULL_PTR;
    }
    if (!hsgp40->voc_algo_initialized) {
        return SGP40_ERR_NOT_READY;
    }

    GasIndexAlgorithm_set_tuning_parameters(
        SGP40_VOC_ALGO(hsgp40),
        index_offset,
        learning_time_offset_hours,
        learning_time_gain_hours,
        gating_max_duration_minutes,
        std_initial,
        gain_factor
    );
    return SGP40_OK;
}

/**
 * @brief Lit les paramètres de tuning de l'algorithme VOC
 */
SGP40_Status SGP40_VOCAlgoGetTuning(
    const SGP40_Handle_t *hsgp40,
    int32_t *index_offset,
    int32_t *learning_time_offset_hours,
    int32_t *learning_time_gain_hours,
    int32_t *gating_max_duration_minutes,
    int32_t *std_initial,
    int32_t *gain_factor
) {
    if (!hsgp40 || !index_offset || !learning_time_offset_hours || !learning_time_gain_hours ||
        !gating_max_duration_minutes || !std_initial || !gain_factor) {
        return SGP40_ERR_NULL_PTR;
    }
    if (!hsgp40->voc_algo_initialized) {
        return SGP40_ERR_NOT_READY;
    }

    GasIndexAlgorithm_get_tuning_parameters(
        SGP40_VOC_ALGO(hsgp40),
        index_offset,
        learning_time_offset_hours,
        learning_time_gain_hours,
        gating_max_duration_minutes,
        std_initial,
        gain_factor
    );
    return SGP40_OK;
}

/**
 * @brief Lit l'intervalle d'échantillonnage utilisé par l'algorithme (secondes)
 */
SGP40_Status SGP40_VOCAlgoGetSamplingInterval(const SGP40_Handle_t *hsgp40, float *sampling_interval_s) {
    if (!hsgp40 || !sampling_interval_s) {
        return SGP40_ERR_NULL_PTR;
    }
    if (!hsgp40->voc_algo_initialized) {
        return SGP40_ERR_NOT_READY;
    }

    GasIndexAlgorithm_get_sampling_interval(SGP40_VOC_ALGO(hsgp40), sampling_interval_s);
    return SGP40_OK;
}

/**
 * @brief Catégorise VOC Index (qualité air)
 */
SGP40_VOC_Category SGP40_GetVOCCategory(uint16_t voc_index) {
    if (voc_index <= 100) {
        return SGP40_VOC_EXCELLENT;
    } else if (voc_index <= 150) {
        return SGP40_VOC_GOOD;
    } else if (voc_index <= 200) {
        return SGP40_VOC_MODERATE;
    } else if (voc_index <= 300) {
        return SGP40_VOC_POOR;
    } else if (voc_index <= 500) {
        return SGP40_VOC_UNHEALTHY;
    } else {
        return SGP40_VOC_INVALID;
    }
}

/**
 * @brief Conversion VOC category → texte
 */
const char* SGP40_VOCCategoryToString(SGP40_VOC_Category category) {
    switch (category) {
        case SGP40_VOC_EXCELLENT:  return "Excellent (0-100)";
        case SGP40_VOC_GOOD:       return "Bon (101-150)";
        case SGP40_VOC_MODERATE:   return "Modere (151-200)";
        case SGP40_VOC_POOR:       return "Mauvais (201-300)";
        case SGP40_VOC_UNHEALTHY:  return "Insalubre (301-500)";
        case SGP40_VOC_INVALID:
        default:                   return "Invalide";
    }
}

/**
 * @brief Conversion code erreur → texte
 * @note  Compilé uniquement si SGP40_DEBUG_ENABLE est défini (économie Flash).
 *        Retourne chaîne vide en production.
 */
const char* SGP40_StatusToString(SGP40_Status code) {
#ifdef SGP40_DEBUG_ENABLE
    switch (code) {
        case SGP40_OK:                 return "OK";
        case SGP40_ERR_NULL_PTR:       return "Erreur: pointeur NULL";
        case SGP40_ERR_INVALID_PARAM:  return "Erreur: parametre invalide";
        case SGP40_ERR_NOT_CONFIGURED: return "Erreur: configuration HAL/NVIC manquante";
        case SGP40_ERR_BUSY:           return "Erreur: bus occupe ou cadence";
        case SGP40_ERR_I2C:            return "Erreur: I2C communication";
        case SGP40_ERR_CRC:            return "Erreur: CRC invalide";
        case SGP40_ERR_SELF_TEST:      return "Erreur: auto-test echoue";
        case SGP40_ERR_NOT_READY:      return "Erreur: capteur pas pret";
        case SGP40_ERR_NOT_INITIALIZED: return "Erreur: capteur non initialise";
        case SGP40_ERR_TIMEOUT:        return "Erreur: timeout";
        case SGP40_ERR_OVERFLOW:       return "Erreur: file pleine";
        case SGP40_ERR_UNKNOWN:
        default:                       return "Erreur: inconnue";
    }
#else
    (void)code;
    return "";
#endif
}

/* =============================================================================
 * Fonctions convenience (warmup + mesure tout-en-un)
 * ============================================================================= */

/**
 * @brief  Vérifie si la phase de warmup de l'algorithme VOC est terminée.
 * @param  hsgp40 Handle SGP40 (non NULL)
 * @retval true si measure_count >= ceil(INITIAL_BLACKOUT_s / sample_interval_s)
 * @note   GasIndexAlgorithm_INITIAL_BLACKOUT est en SECONDES. La conversion en
 *         nombre d'échantillons dépend de sample_interval_ms. Correct à tout intervalle.
 */
bool SGP40_IsVOCWarmupComplete(const SGP40_Handle_t *hsgp40) {
    if (!hsgp40) {
        return false;
    }
    float samples_needed = GasIndexAlgorithm_INITIAL_BLACKOUT
                         * 1000.0f / (float)hsgp40->sample_interval_ms;
    return hsgp40->measure_count >= (uint32_t)ceilf(samples_needed);
}

/**
 * @brief  Retourne le nombre d'échantillons nécessaires au warmup de l'algo VOC.
 * @param  hsgp40 Handle SGP40 (non NULL) — intervalle utilisé pour le calcul
 * @retval Nombre d'échantillons = ceil(45 s / sample_interval_s)
 * @note   Retourne 45 (valeur 1 Hz) si hsgp40 est NULL.
 */
uint32_t SGP40_GetVOCWarmupSamples(const SGP40_Handle_t *hsgp40) {
    if (!hsgp40) {
        return (uint32_t)GasIndexAlgorithm_INITIAL_BLACKOUT; /* fallback : 45 @ 1 Hz */
    }
    float samples_needed = GasIndexAlgorithm_INITIAL_BLACKOUT
                         * 1000.0f / (float)hsgp40->sample_interval_ms;
    return (uint32_t)ceilf(samples_needed);
}

/**
 * @brief  Mesure VOC raw + calcul VOC Index en un seul appel.
 * @param  hsgp40    Handle SGP40 (non NULL, initialisé)
 * @param  voc_raw   Pointeur pour stocker signal VOC raw (peut être NULL)
 * @param  voc_index Pointeur pour stocker VOC index 0-500 (non NULL)
 * @retval SGP40_Status
 */
SGP40_Status SGP40_MeasureVOCIndex(SGP40_Handle_t *hsgp40, uint16_t *voc_raw, uint16_t *voc_index) {
    if (!hsgp40 || !voc_index) {
        return SGP40_ERR_NULL_PTR;
    }

    uint16_t raw = 0;
    SGP40_Status status = SGP40_MeasureRaw(hsgp40, &raw);
    if (status != SGP40_OK) {
        return status;
    }

    if (voc_raw) {
        *voc_raw = raw;
    }

    return SGP40_CalculateVOCIndex(hsgp40, raw, voc_index);
}

SGP40_Status SGP40_ReadAll(SGP40_Handle_t *hsgp40, SGP40_Data *data_out) {
    if (!hsgp40 || !data_out) {
        return SGP40_ERR_NULL_PTR;
    }
    return SGP40_MeasureVOCIndex(hsgp40, &data_out->voc_raw, &data_out->voc_index);
}

/* =============================================================================
 * API Asynchrone (IT)
 * ============================================================================= */

/**
 * @brief  Vérifie si une IRQ NVIC est activée.
 */
static bool SGP40_IsIrqEnabledInternal(IRQn_Type irqn) {
    if (irqn < 0) {
        return false;
    }

    uint32_t irq_index = ((uint32_t)irqn) >> 5U;
    uint32_t irq_mask  = 1UL << (((uint32_t)irqn) & 0x1FU);
    return (NVIC->ISER[irq_index] & irq_mask) != 0U;
}

/**
 * @brief  Résout la paire IRQn (EV+ER) pour un périphérique I2C.
 */
static bool SGP40_GetI2CIrqPair(const I2C_HandleTypeDef *hi2c, IRQn_Type *ev_irq, IRQn_Type *er_irq) {
    if (!hi2c || !ev_irq || !er_irq) {
        return false;
    }

#ifdef I2C1
    if (hi2c->Instance == I2C1) { *ev_irq = I2C1_EV_IRQn; *er_irq = I2C1_ER_IRQn; return true; }
#endif
#ifdef I2C2
    if (hi2c->Instance == I2C2) { *ev_irq = I2C2_EV_IRQn; *er_irq = I2C2_ER_IRQn; return true; }
#endif
#ifdef I2C3
    if (hi2c->Instance == I2C3) { *ev_irq = I2C3_EV_IRQn; *er_irq = I2C3_ER_IRQn; return true; }
#endif
#ifdef I2C4
    if (hi2c->Instance == I2C4) { *ev_irq = I2C4_EV_IRQn; *er_irq = I2C4_ER_IRQn; return true; }
#endif

    return false;
}

/**
 * @brief  Vérifie configuration NVIC pour mode IT.
 */
static SGP40_Status SGP40_Async_CheckConfig_IT(const SGP40_Async_t *ctx) {
    if (!ctx || !ctx->hi2c) {
        return SGP40_ERR_NULL_PTR;
    }

    IRQn_Type ev_irq, er_irq;
    if (!SGP40_GetI2CIrqPair(ctx->hi2c, &ev_irq, &er_irq)) {
        return SGP40_ERR_NOT_CONFIGURED;
    }

    if (!SGP40_IsIrqEnabledInternal(ev_irq) || !SGP40_IsIrqEnabledInternal(er_irq)) {
        return SGP40_ERR_NOT_CONFIGURED;
    }

    return SGP40_OK;
}

/**
 * @brief Initialise contexte asynchrone
 * @param ctx    Contexte async à initialiser
 * @param hsgp40 Handle SGP40 synchrone (doit être déjà initialisé)
 */
void SGP40_Async_Init(SGP40_Async_t *ctx, SGP40_Handle_t *hsgp40) {
    if (!ctx || !hsgp40) return;

    memset(ctx, 0, sizeof(SGP40_Async_t));
    ctx->hsgp40      = hsgp40;
    ctx->hi2c        = hsgp40->hi2c;
    ctx->state       = SGP40_ASYNC_IDLE;
    ctx->last_status = SGP40_OK;
    // hsgp40->async_busy déjà à 0 (Init ou memset)
}

/**
 * @brief  Reset la machine d'état async sans perdre les callbacks/timebase.
 * @param  ctx Contexte async (non NULL)
 * @note   Préserve : hsgp40, hi2c, tous les callbacks, user_ctx.
 *         Remet : state=IDLE, flags=false, buffers=0, deadlines=0, async_busy=0.
 */
void SGP40_Async_Reset(SGP40_Async_t *ctx) {
    if (!ctx) return;

    ctx->state              = SGP40_ASYNC_IDLE;
    ctx->last_status        = SGP40_OK;
    ctx->meas_deadline_ms   = 0U;
    ctx->i2c_deadline_ms    = 0U;
    memset(ctx->cmd_buf, 0, sizeof(ctx->cmd_buf));
    memset(ctx->rx_buf,  0, sizeof(ctx->rx_buf));
    ctx->voc_raw             = 0U;
    ctx->data_ready_flag     = false;
    ctx->error_flag          = false;
    ctx->notify_data_pending = false;
    ctx->notify_error_pending = false;

    if (ctx->hsgp40) {
        ctx->hsgp40->async_busy = 0U;                 // Libère le verrou polling/async
    }
    /* hsgp40, hi2c, callbacks, user_ctx, irq_user_ctx : préservés */
}

/**
 * @brief Configure callbacks utilisateur (contexte main loop)
 */
void SGP40_Async_SetCallbacks(SGP40_Async_t *ctx,
                               SGP40_Async_OnDataReadyCb on_data_ready,
                               SGP40_Async_OnErrorCb on_error,
                               void *user_ctx) {
    if (!ctx) return;
    ctx->on_data_ready = on_data_ready;
    ctx->on_error      = on_error;
    ctx->user_ctx      = user_ctx;
}

/**
 * @brief Configure callbacks IRQ-safe (depuis interruptions I2C)
 */
void SGP40_Async_SetIrqCallbacks(SGP40_Async_t *ctx,
                                  SGP40_Async_OnIrqDataReadyCb on_irq_data_ready,
                                  SGP40_Async_OnIrqErrorCb on_irq_error,
                                  void *user_ctx) {
    if (!ctx) return;
    ctx->on_irq_data_ready = on_irq_data_ready;
    ctx->on_irq_error      = on_irq_error;
    ctx->irq_user_ctx      = user_ctx;   /* F1 : contexte IRQ distinct — ne pas écraser user_ctx main-loop */
}

/**
 * @brief Lecture flag données prêtes
 */
bool SGP40_Async_DataReadyFlag(const SGP40_Async_t *ctx) {
    return ctx ? ctx->data_ready_flag : false;
}

/**
 * @brief Lecture flag erreur
 */
bool SGP40_Async_ErrorFlag(const SGP40_Async_t *ctx) {
    return ctx ? ctx->error_flag : false;
}

/**
 * @brief Clear flags data_ready et error
 */
void SGP40_Async_ClearFlags(SGP40_Async_t *ctx) {
    if (!ctx) return;
    ctx->data_ready_flag      = false;
    ctx->error_flag           = false;
    ctx->notify_data_pending  = false;
    ctx->notify_error_pending = false;
}

/**
 * @brief Vérifie si machine d'état idle
 */
bool SGP40_Async_IsIdle(const SGP40_Async_t *ctx) {
    return ctx ? (ctx->state == SGP40_ASYNC_IDLE) : false;
}

/**
 * @brief Vérifie si données disponibles
 */
bool SGP40_Async_HasData(const SGP40_Async_t *ctx) {
    return ctx ? (ctx->state == SGP40_ASYNC_DONE && ctx->last_status == SGP40_OK) : false;
}

/**
 * @brief  Prépare et lance le transfert I2C en mode IT (non-bloquant).
 * @note   DMA non implémenté : aucun gain réel pour les trames SGP40
 *         (8 octets TX / 3 octets RX) — IT suffit et simplifie la configuration CubeMX.
 */
static SGP40_Status SGP40_Async_TriggerMeasure_Common(SGP40_Async_t *ctx) {
    uint32_t now_ms = HAL_GetTick();
    if (ctx->hsgp40->measurement_mode_active) {        // Si au moins une mesure déjà faite
        uint32_t elapsed = now_ms - ctx->hsgp40->last_measure_tick_ms;
        if (elapsed < ctx->hsgp40->sample_interval_ms) {
            return SGP40_ERR_BUSY;
        }
    }

    ctx->data_ready_flag      = false;
    ctx->error_flag           = false;
    ctx->notify_data_pending  = false;
    ctx->notify_error_pending = false;

    /* Préparer commande 0x260F + compensation T/RH (cmd 2B + RH 2B+CRC + T 2B+CRC = 8B) */
    uint16_t cmd = SGP40_CMD_MEASURE_RAW;
    ctx->cmd_buf[0] = (cmd >> 8) & 0xFF;
    ctx->cmd_buf[1] = cmd & 0xFF;

    uint16_t rh_raw = SGP40_RHToRaw(ctx->hsgp40->rh_percent);
    uint16_t t_raw  = SGP40_TempToRaw(ctx->hsgp40->temp_c);

    ctx->cmd_buf[2] = (rh_raw >> 8) & 0xFF;
    ctx->cmd_buf[3] = rh_raw & 0xFF;
    ctx->cmd_buf[4] = SGP40_CalculateCRC8(&ctx->cmd_buf[2], 2);
    ctx->cmd_buf[5] = (t_raw >> 8) & 0xFF;
    ctx->cmd_buf[6] = t_raw & 0xFF;
    ctx->cmd_buf[7] = SGP40_CalculateCRC8(&ctx->cmd_buf[5], 2);

    /* Lancer transfert I2C en mode IT (non-bloquant) */
    HAL_StatusTypeDef hal_status = HAL_I2C_Master_Transmit_IT(
        ctx->hi2c,
        (uint16_t)(ctx->hsgp40->i2c_addr_7b << 1),
        ctx->cmd_buf, SGP40_ASYNC_CMD_BUF_SIZE);

    if (hal_status == HAL_BUSY) {
        /* Bus I2C occupé par une autre lib async IT sur le même hi2c.
         * La FSM reste IDLE : TriggerEvery() retentera automatiquement au prochain appel.
         * HAL_BUSY ne compte pas dans consecutive_errors (contention de bus normale). */
        ctx->state       = SGP40_ASYNC_IDLE;
        ctx->last_status = SGP40_ERR_BUSY;
        return SGP40_ERR_BUSY;
    }
    if (hal_status != HAL_OK) {                        // Vraie erreur I2C (NACK, timeout HW...)
        ctx->hsgp40->last_hal_error = HAL_I2C_GetError(ctx->hi2c);
        ctx->hsgp40->consecutive_errors++;
        ctx->hsgp40->last_error = SGP40_ERR_I2C;
        ctx->state       = SGP40_ASYNC_ERROR;
        ctx->last_status = (hal_status == HAL_TIMEOUT) ? SGP40_ERR_TIMEOUT : SGP40_ERR_I2C;
        return ctx->last_status;
    }

    /* Transfert lancé — marque le handle comme occupé en async */
    ctx->hsgp40->async_busy              = 1U;
    ctx->state                           = SGP40_ASYNC_WRITE_CMD;
    ctx->i2c_deadline_ms                 = now_ms + ctx->hsgp40->i2c_timeout;
    ctx->last_status                     = SGP40_OK;
    ctx->hsgp40->measurement_mode_active = true;
    ctx->hsgp40->last_measure_tick_ms    = now_ms;

    return SGP40_OK;
}

/**
 * @brief Déclenche mesure VOC en mode IT (non-bloquant)
 */
SGP40_Status SGP40_ReadAll_IT(SGP40_Async_t *ctx) {
    if (!ctx || !ctx->hsgp40 || !ctx->hi2c) return SGP40_ERR_NULL_PTR;
    if (ctx->state != SGP40_ASYNC_IDLE) return SGP40_ERR_NOT_READY;

    SGP40_Status cfg_status = SGP40_Async_CheckConfig_IT(ctx);
    if (cfg_status != SGP40_OK) {
        ctx->state       = SGP40_ASYNC_ERROR;
        ctx->last_status = cfg_status;
        return cfg_status;
    }

    return SGP40_Async_TriggerMeasure_Common(ctx);
}

/**
 * @brief Machine d'état async — à appeler dans main loop
 */
void SGP40_Async_Process(SGP40_Async_t *ctx, uint32_t now_ms) {
    if (!ctx) return;

    // Délivrer les notifications utilisateur pendantes (déclenchées depuis les ISR)
    if (ctx->notify_data_pending && ctx->on_data_ready) {
        ctx->on_data_ready(ctx->user_ctx, ctx->voc_raw, ctx->last_status);
        ctx->notify_data_pending = false;
    }

    if (ctx->notify_error_pending && ctx->on_error) {
        ctx->on_error(ctx->user_ctx, ctx->last_status);
        ctx->notify_error_pending = false;
    }

    switch (ctx->state) {
        case SGP40_ASYNC_IDLE:
        case SGP40_ASYNC_DONE:
        case SGP40_ASYNC_ERROR:
            break;

        case SGP40_ASYNC_WRITE_CMD:                    // En attente du TxCplt I2C
            if ((int32_t)(now_ms - ctx->i2c_deadline_ms) >= 0) {  // Timeout I2C dépassé
                ctx->hsgp40->consecutive_errors++;
                ctx->hsgp40->last_error = SGP40_ERR_TIMEOUT;
                ctx->state              = SGP40_ASYNC_ERROR;
                ctx->last_status        = SGP40_ERR_TIMEOUT;
                ctx->error_flag         = true;
                ctx->notify_error_pending = true;
                ctx->hsgp40->async_busy = 0U;          // Libère le verrou : timeout = fin de transaction
            }
            break;

        case SGP40_ASYNC_WAIT_MEAS:                    // Attente délai mesure 30ms
            if ((int32_t)(now_ms - ctx->meas_deadline_ms) >= 0) {
                HAL_StatusTypeDef hal_status = HAL_I2C_Master_Receive_IT(
                    ctx->hi2c,
                    (uint16_t)(ctx->hsgp40->i2c_addr_7b << 1),
                    ctx->rx_buf, 3);

                if (hal_status == HAL_BUSY) {
                    // Bus occupé par un autre capteur : réessai dans SGP40_ASYNC_BUSY_RETRY_MS (multi-capteurs bus partagé)
                    // HAL_BUSY RX : async_busy reste à 1, on réessaiera
                    ctx->meas_deadline_ms = now_ms + SGP40_ASYNC_BUSY_RETRY_MS;
                } else if (hal_status != HAL_OK) {
                    ctx->hsgp40->last_hal_error = HAL_I2C_GetError(ctx->hi2c);
                    ctx->hsgp40->consecutive_errors++;
                    ctx->hsgp40->last_error = SGP40_ERR_I2C;
                    ctx->state              = SGP40_ASYNC_ERROR;
                    ctx->last_status        = (hal_status == HAL_TIMEOUT) ? SGP40_ERR_TIMEOUT : SGP40_ERR_I2C;
                    ctx->error_flag         = true;
                    ctx->notify_error_pending = true;
                    ctx->hsgp40->async_busy = 0U;      // Libère le verrou : erreur = fin de transaction
                } else {
                    ctx->state           = SGP40_ASYNC_READ_DATA;
                    ctx->i2c_deadline_ms = now_ms + ctx->hsgp40->i2c_timeout;
                }
            }
            break;

        case SGP40_ASYNC_READ_DATA:                    // En attente du RxCplt I2C
            if ((int32_t)(now_ms - ctx->i2c_deadline_ms) >= 0) {  // Timeout I2C dépassé
                ctx->hsgp40->consecutive_errors++;
                ctx->hsgp40->last_error = SGP40_ERR_TIMEOUT;
                ctx->state              = SGP40_ASYNC_ERROR;
                ctx->last_status        = SGP40_ERR_TIMEOUT;
                ctx->error_flag         = true;
                ctx->notify_error_pending = true;
                ctx->hsgp40->async_busy = 0U;          // Libère le verrou
            }
            break;
    }
    return;
}

/**
 * @brief Récupère dernière mesure VOC raw
 * @param ctx     Contexte async
 * @param voc_raw Pointeur pour stocker VOC raw
 * @retval SGP40_OK              si données valides (état DONE)
 * @retval SGP40_ERR_NULL_PTR    si ctx ou voc_raw est NULL
 * @retval SGP40_ERR_NOT_CONFIGURED si IDLE (aucune mesure lancée)
 * @retval SGP40_ERR_BUSY        si transfert en cours (état non-DONE, non-IDLE)
 * @retval ctx->last_status      si état ERROR (propage l'erreur réelle)
 * @note  Consomme la mesure et remet l'état à IDLE après lecture.
 */
SGP40_Status SGP40_Async_GetData(SGP40_Async_t *ctx, uint16_t *voc_raw) {
    if (!ctx || !voc_raw) return SGP40_ERR_NULL_PTR;

    if (ctx->state == SGP40_ASYNC_IDLE)  return SGP40_ERR_NOT_CONFIGURED;
    if (ctx->state == SGP40_ASYNC_ERROR) return ctx->last_status;
    if (ctx->state != SGP40_ASYNC_DONE)  return SGP40_ERR_BUSY;

    *voc_raw   = ctx->voc_raw;
    ctx->state = SGP40_ASYNC_IDLE;
    ctx->data_ready_flag     = false;
    ctx->notify_data_pending = false;

    if (ctx->hsgp40) {
        ctx->hsgp40->async_busy = 0U;                 // Données consommées → libère le verrou
    }

    return ctx->last_status;
}

/**
 * @brief Callback HAL - Tx I2C terminé (mode IT)
 * @note  À appeler depuis HAL_I2C_MasterTxCpltCallback.
 *        Filtre sur hi2c : ignore si différent du handle configuré.
 */
void SGP40_Async_OnI2CMasterTxCplt(SGP40_Async_t *ctx, I2C_HandleTypeDef *hi2c) {
    if (!ctx || ctx->hi2c != hi2c) return;             // Filtre : ignore si handle I2C différent

    if (ctx->state == SGP40_ASYNC_WRITE_CMD) {
        // Commande + compensation envoyées, lancer temporisation SGP40_MEASURE_WAIT_MS avant lecture
        ctx->state            = SGP40_ASYNC_WAIT_MEAS;
        ctx->meas_deadline_ms = HAL_GetTick() + SGP40_MEASURE_WAIT_MS;
        // async_busy reste à 1 : on va lancer un RX ensuite
    }
}

/**
 * @brief Callback HAL - Rx I2C terminé (mode IT)
 * @note  À appeler depuis HAL_I2C_MasterRxCpltCallback.
 *        Filtre sur hi2c : ignore si différent du handle configuré.
 */
void SGP40_Async_OnI2CMasterRxCplt(SGP40_Async_t *ctx, I2C_HandleTypeDef *hi2c) {
    if (!ctx || ctx->hi2c != hi2c) return;             // Filtre : ignore si handle I2C différent

    if (ctx->state == SGP40_ASYNC_READ_DATA) {
        // Vérifier CRC du mot reçu
        uint8_t crc_calc = SGP40_CalculateCRC8(ctx->rx_buf, 2);
        if (crc_calc != ctx->rx_buf[2]) {              // CRC invalide : donnée corrompue
            ctx->hsgp40->consecutive_errors++;
            ctx->hsgp40->last_error = SGP40_ERR_CRC;
            ctx->state              = SGP40_ASYNC_ERROR;
            ctx->last_status        = SGP40_ERR_CRC;
            ctx->error_flag         = true;
            ctx->notify_error_pending = true;
            ctx->hsgp40->async_busy = 0U;              // Libère le verrou

            if (ctx->on_irq_error) {
                ctx->on_irq_error(ctx->irq_user_ctx);
            }
            return;
        }

        // Extraire VOC raw (2 octets utiles, CRC validé)
        ctx->voc_raw = (uint16_t)((ctx->rx_buf[0] << 8) | ctx->rx_buf[1]);
        ctx->hsgp40->last_voc_raw = ctx->voc_raw;
        ctx->hsgp40->measure_count++;
        ctx->hsgp40->consecutive_errors = 0;           // Mesure réussie : remet compteur à zéro

        ctx->state               = SGP40_ASYNC_DONE;
        ctx->last_status         = SGP40_OK;
        ctx->data_ready_flag     = true;
        ctx->notify_data_pending = true;
        // async_busy reste à 1 jusqu'à GetData() ou Reset()
        // (données disponibles mais pas encore consommées — polling toujours bloqué)

        if (ctx->on_irq_data_ready) {
            ctx->on_irq_data_ready(ctx->irq_user_ctx);
        }
    }
}

/**
 * @brief Callback HAL - Erreur I2C
 * @note  À appeler depuis HAL_I2C_ErrorCallback.
 *        Filtre sur hi2c : ignore si différent du handle configuré.
 */
void SGP40_Async_OnI2CError(SGP40_Async_t *ctx, I2C_HandleTypeDef *hi2c) {
    if (!ctx || ctx->hi2c != hi2c) return;             // Filtre : ignore si handle I2C différent

    ctx->hsgp40->consecutive_errors++;
    ctx->hsgp40->last_error    = SGP40_ERR_I2C;
    ctx->state                 = SGP40_ASYNC_ERROR;
    ctx->last_status           = SGP40_ERR_I2C;
    ctx->error_flag            = true;
    ctx->notify_error_pending  = true;
    ctx->hsgp40->async_busy    = 0U;                   // Erreur = fin de transaction HAL IT

    if (ctx->on_irq_error) {
        ctx->on_irq_error(ctx->irq_user_ctx);
    }
}

/* =============================================================================
 * Helpers haut niveau : Tick / TriggerEvery
 * ============================================================================= */

/**
 * @brief Avance la machine d'état async et retourne un résultat actionnable
 * @note  TICK_ERROR est émis UNIQUEMENT via ErrorFlag (vraie erreur HW : CRC, NACK,
 *        timeout). ERR_BUSY dans last_status (contention de bus normale sur bus
 *        partagé) ne déclenche PAS TICK_ERROR — la FSM reste IDLE et TriggerEvery
 *        retentera automatiquement au prochain cycle.
 */
SGP40_TickResult SGP40_Async_Tick(SGP40_Async_t *ctx, uint32_t now_ms, uint16_t *voc_raw_out) {
    if (!ctx) return SGP40_TICK_IDLE;

    SGP40_Async_Process(ctx, now_ms);
    /* Ne pas tester ctx->last_status ici : last_status == ERR_BUSY si HAL_BUSY
     * (contention de bus) — ce n'est pas une erreur → ne pas retourner TICK_ERROR. */

    if (SGP40_Async_ErrorFlag(ctx)) {                  // Vraie erreur HW uniquement
        SGP40_Async_ClearFlags(ctx);
        SGP40_Async_Reset(ctx);
        return SGP40_TICK_ERROR;
    }

    if (SGP40_Async_HasData(ctx)) {                    // Donnée prête
        uint16_t raw = 0U;
        SGP40_Async_GetData(ctx, &raw);
        SGP40_Async_ClearFlags(ctx);
        if (voc_raw_out) *voc_raw_out = raw;
        return SGP40_TICK_DATA_READY;
    }

    return SGP40_Async_IsIdle(ctx) ? SGP40_TICK_IDLE : SGP40_TICK_BUSY;
}

/**
 * @brief Déclenche ReadAll_IT() si IDLE && intervalle (stocké dans hsgp40) écoulé.
 *        Met à jour *last_ms uniquement si le trigger réussit.
 * @note  Compatible multi-capteurs : HAL_BUSY n'incrémente pas consecutive_errors.
 */
SGP40_Status SGP40_Async_TriggerEvery(SGP40_Async_t *ctx, uint32_t now_ms,
                                       uint32_t *last_ms) {
    if (!ctx || !last_ms) return SGP40_ERR_NULL_PTR;
    if ((now_ms - *last_ms) < ctx->hsgp40->sample_interval_ms) return SGP40_OK;  // Trop tôt
    if (!SGP40_Async_IsIdle(ctx)) return SGP40_ERR_BUSY;                          // FSM occupée

    SGP40_Status st = SGP40_ReadAll_IT(ctx);
    if (st == SGP40_OK) *last_ms = now_ms;
    return st;
}

/**
 * @brief Tick + calcul VOC Index en un seul appel.
 */
SGP40_TickResult SGP40_Async_TickIndex(SGP40_Async_t *ctx, uint32_t now_ms,
                                       uint16_t *voc_raw_out, uint16_t *voc_index_out) {
    if (!ctx || !voc_index_out) return SGP40_TICK_ERROR;

    uint16_t raw = 0U;
    SGP40_TickResult tick = SGP40_Async_Tick(ctx, now_ms, &raw);

    if (tick != SGP40_TICK_DATA_READY) return tick;

    if (voc_raw_out) *voc_raw_out = raw;

    SGP40_Status idx_status = SGP40_CalculateVOCIndex(ctx->hsgp40, raw, voc_index_out);
    if (idx_status != SGP40_OK) {
        ctx->last_status = idx_status;
        return SGP40_TICK_ERROR;
    }

    return SGP40_TICK_DATA_READY;
}

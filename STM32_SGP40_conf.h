/**
  ******************************************************************************
  * @file    STM32_SGP40_conf.h
  * @brief   Configuration overridable de la librairie STM32_SGP40
  * @author  Generated via STM32_LIB_STYLE_GUIDE.md
  * @date    2026-02-21
  * @version 0.9.0
 * @copyright Libre sous licence MIT.
  ******************************************************************************
  * @attention
  *
  * Ce fichier centralise toutes les constantes reconfigurables de la lib SGP40.
  * Pour personnaliser, définir les macros AVANT l'inclusion de STM32_SGP40.h,
  * ou via les options de compilation (-DSGP40_MAX_CONSECUTIVE_ERRORS=5).
  *
  * Exemple CMakeLists.txt :
  *   target_compile_definitions(my_target PRIVATE SGP40_MAX_CONSECUTIVE_ERRORS=5)
  *
  * Exemple STM32CubeIDE (Project Properties → C/C++ Build → Settings → Preprocessor) :
  *   SGP40_MAX_CONSECUTIVE_ERRORS=5
  *
  ******************************************************************************
  */

#ifndef STM32_SGP40_CONF_H
#define STM32_SGP40_CONF_H

/* =============================================================================
 * Debug / traces série (désactivé par défaut en production)
 * ============================================================================= */

/**
 * @brief Active SGP40_StatusToString() et d'éventuelles traces.
 *
 * Décommenter pour activer — ou définir via option de compilation.
 * En production (firmware final), laisser commenté pour économiser la Flash.
 *
 * @note  SGP40_StatusToString() retourne "" (chaîne vide) si non défini.
 */
/* #define SGP40_DEBUG_ENABLE */   /* Décommenter pour activer */

/* =============================================================================
 * Timeouts I2C (ms)
 * ============================================================================= */

/** Timeout par défaut pour les transactions I2C bloquantes (ms). */
#ifndef SGP40_DEFAULT_TIMEOUT_MS
#define SGP40_DEFAULT_TIMEOUT_MS     100U
#endif

/* =============================================================================
 * Délais de mesure (ms) — extraits datasheet Sensirion SGP40
 * ============================================================================= */

/** Délai mesure VOC raw (CMD 0x260F) — datasheet §5.1 : 30 ms max. */
#ifndef SGP40_MEASURE_WAIT_MS
#define SGP40_MEASURE_WAIT_MS        30U
#endif

/** Délai auto-test (CMD 0x280E) — datasheet §5.2 : 320 ms max. */
#ifndef SGP40_SELFTEST_WAIT_MS
#define SGP40_SELFTEST_WAIT_MS       320U
#endif

/* =============================================================================
 * Gestion des erreurs consécutives
 * ============================================================================= */

/**
 * @brief Seuil d'erreurs I2C consécutives avant abandon de la mesure.
 *
 * Quand h->consecutive_errors atteint cette valeur, l'application doit
 * considérer le capteur hors-ligne et appeler SGP40_DeInit() + SGP40_Init().
 * Une erreur HAL_BUSY (bus occupé par une autre lib) ne compte PAS.
 * @note  Valeur recommandée : 3 à 5.
 */
#ifndef SGP40_MAX_CONSECUTIVE_ERRORS
#define SGP40_MAX_CONSECUTIVE_ERRORS 3U
#endif

/* =============================================================================
 * Délais internes (ms)
 * ============================================================================= */

/** @brief Délai post-commande GetSerialNumber avant lecture — datasheet §5.3 : < 1 ms. */
#ifndef SGP40_SERIAL_READ_DELAY_MS
#define SGP40_SERIAL_READ_DELAY_MS   1U
#endif

/** @brief Délai de réessai RX HAL_BUSY (bus partagé multi-capteurs, mode async IT). */
#ifndef SGP40_ASYNC_BUSY_RETRY_MS
#define SGP40_ASYNC_BUSY_RETRY_MS    2U
#endif

#endif /* STM32_SGP40_CONF_H */

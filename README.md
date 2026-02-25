# STM32_SGP40 – Driver STM32 HAL pour capteur VOC Sensirion SGP40

Driver C pur pour **Sensirion SGP40** sur STM32 HAL (I2C, adresse fixe `0x59`).

**Fonctionnalités :**

- Mesure VOC raw 16 bits
- Compensation température / humidité paramétrable (−45 °C → +130 °C)
- Calcul VOC Index 0..500 via algorithme officiel Sensirion GasIndexAlgorithm v3.2
- Catégorisation qualité d'air (Excellent → Dangereux)
- API synchrone bloquante + API asynchrone IT uniquement (DMA non implémenté : aucun gain sur trames 3-8 octets)
- I2C handle et base de temps injectables (portabilité multi-famille STM32)
- Vérification CRC-8 sur toutes les trames capteur
- Rate-limiting intégré (`SGP40_ERR_BUSY` si cadence non respectée)
- Gestion d'erreurs structurée (`SGP40_Status` : code + HAL + message)
- Aucun `printf` dans la librairie

---

## 1) Périmètre

| Supporté                                | Non supporté                        |
| --------------------------------------- | ----------------------------------- |
| VOC raw (`MEASURE_RAW` `0x260F`)        | CO2, NOx, température, humidité     |
| Auto-test (`MEASURE_TEST` `0x280E`)     | Soft reset (retirée volontairement) |
| Arrêt heater (`HEATER_OFF` `0x3615`)    |                                     |
| Numéro de série (`GET_SERIAL` `0x3682`) |                                     |

> **Soft reset** : volontairement absente. Contrairement à DFRobot qui réinitialise
> silencieusement la compensation à 25 °C / 50 %RH via `softReset()`, notre librairie
> conserve toujours les valeurs T/RH configurées par l'utilisateur. La seule façon de
> les modifier est l'appel explicite à `SGP40_SetCompensation()`.

---

## 2) Fichiers

| Fichier                           | Rôle                                        |
| --------------------------------- | ------------------------------------------- |
| `STM32_SGP40.h`                   | API publique, types, constantes, defines    |
| `STM32_SGP40.c`                   | Implémentation sync + async (IT uniquement) |
| `sensirion_gas_index_algorithm.h` | Header algorithme Sensirion GasIndex v3.2   |
| `sensirion_gas_index_algorithm.c` | Moteur de calcul VOC Index officiel         |
| `exemples/`                       | 8 exemples couvrant 100 % de l'API          |

---

## 3) Pré-requis CubeMX / HAL

### Volet 1 — Mode polling (pré-requis minimaux)

- **I2C** : mode 7-bit, 100 kHz ou 400 kHz
- **Pull-ups** : résistances SDA/SCL recommandées (4.7 kΩ typ., obligatoires si pas intégrées)
- **Horloge** : `HAL_GetTick()` fonctionnel (SysTick activé par défaut dans CubeMX)
- **NVIC I2C** : non requis en polling

### Volet 2 — Mode async IT (en plus du polling)

- **NVIC** : activer **I2C Event** ET **I2C Error** du bus choisi
  (ex. `I2C1_EV_IRQn` + `I2C1_ER_IRQn` dans CubeMX → NVIC → I2C1 event/error interrupt)
- **Pas de DMA requis** — DMA non implémenté (aucun gain sur trames 3-8 octets, cf. Q3)
- Placer les trois callbacks dans `USER CODE BEGIN 4` (voir §9)

### Optionnel

- UART pour `printf` dans les exemples

### Décision async (Q1→Q4)

- **Q1 (temps de mesure)** : ~30 ms par mesure SGP40 → async pertinent pour libérer le CPU.
- **Q2 (callbacks HAL exploitables)** : oui, callbacks I2C IT natifs STM32 (`HAL_I2C_MasterTxCpltCallback`, `HAL_I2C_MasterRxCpltCallback`, `HAL_I2C_ErrorCallback`).
- **Q3 (taille trame)** : 8 octets TX + 3 octets RX → DMA non pertinent (overhead supérieur au gain).
- **Q4 (bus partagé)** : bus I2C potentiellement partagé multi-capteurs → IT recommandé, DMA évité.

**Conclusion** : mode async **IT uniquement** ; DMA non implémenté par choix technique (Q3+Q4).

---

## 4) Initialisation

### Méthode rapide (tout-en-un)

```c
#include "STM32_SGP40.h"

SGP40_HandleTypeDef hsgp40;

SGP40_Status st = SGP40_Init(&hsgp40, &hi2c3);
if (st != SGP40_OK) { Error_Handler(); }
```

`SGP40_Init()` configure les paramètres par défaut utiles : adresse `0x59`, timeout 100 ms,
compensation 25 °C / 50 %RH, cadence 1000 ms, puis initialise le capteur (serial + algo VOC).

### Méthode paramétrable (recommandée)

`SGP40_Init()` configure les défauts (`addr=0x59`, `timeout=100 ms`, `25 °C / 50 %RH`, `1000 ms`).
Les personnalisations se font **après** `Init()` via les setters dédiés.

```c
#include "STM32_SGP40.h"

SGP40_HandleTypeDef hsgp40;

// 1) Initialisation capteur (applique tous les défauts)
SGP40_Status st = SGP40_Init(&hsgp40, &hi2c3);
if (st != SGP40_OK) { Error_Handler(); }

// 2) Personnalisation post-Init
SGP40_SetCompensation(&hsgp40, 22.5f, 60.0f);   // T/RH depuis capteur externe
SGP40_SetSampleInterval(&hsgp40, 2000U);          // Cadence 2 s (défaut : 1000 ms)
hsgp40.i2c_timeout = 200U;                        // Timeout I2C personnalisé (ms)
```

---

## 5) Flux recommandé (VOC Index)

```
SGP40_Init
        │
SGP40_SetSampleInterval(1000U)     ← cadence algo (500..10000 ms)
        │
SGP40_SetCompensation(T, RH)       ← si capteur T/RH externe
        │
SGP40_PrimeForVocIndex()           ← 1re mesure jetée (amorçage algo)
        │
    ┌───┴──── Boucle 1 Hz ────┐
    │  SGP40_MeasureVOCIndex()    │
    │  SGP40_IsVOCWarmupComplete  │
    │  SGP40_GetVOCCategory      │
    └──────────────────────────┘
```

Exemple minimal :

```c
SGP40_SetSampleInterval(&hsgp40, 1000U);
SGP40_PrimeForVocIndex(&hsgp40);

while (1) {
    uint16_t voc_raw, voc_index;
    SGP40_Status s = SGP40_MeasureVOCIndex(&hsgp40, &voc_raw, &voc_index);
    if (s == SGP40_OK) {
        if (SGP40_IsVOCWarmupComplete(&hsgp40)) {
            const char *cat = SGP40_VOCCategoryToString(SGP40_GetVOCCategory(voc_index));
            printf("VOC Index: %u  %s\r\n", voc_index, cat);
        } else {
            printf("Warmup...\r\n");
        }
    }
    HAL_Delay(1000U);
}
```

---

## 6) API publique complète

### 6.1 Configuration et initialisation

| Fonction                                        | Description                      |
| ----------------------------------------------- | -------------------------------- |
| `SGP40_Init(hsgp40, hi2c)`                      | Init capteur (serial + algo VOC) |
| `SGP40_SetSampleInterval(hsgp40, interval_ms)`  | Cadence algo VOC (500..10000 ms) |
| `SGP40_SetCompensation(hsgp40, temp_c, rh_pct)` | Compensation T/RH (−45..+130 °C) |

### 6.2 Mesure et diagnostic

| Fonction                                 | Description                          |
| ---------------------------------------- | ------------------------------------ |
| `SGP40_MeasureRaw(hsgp40, &voc_raw)`     | Mesure VOC raw 16 bits               |
| `SGP40_GetSerialNumber(hsgp40, &serial)` | Lit le serial 48 bits                |
| `SGP40_SelfTest(hsgp40, &test_result)`   | Auto-test capteur (pattern `0xD4XX`) |
| `SGP40_HeaterOff(hsgp40)`                | Arrête le heater (nécessite re-init) |
| `SGP40_PrimeForVocIndex(hsgp40)`         | Amorçage algo (1re mesure jetée)     |

### 6.3 Calcul VOC Index

| Fonction                                       | Description                     |
| ---------------------------------------------- | ------------------------------- |
| `SGP40_CalculateVOCIndex(hsgp40, raw, &index)` | Raw → VOC Index (0..500)        |
| `SGP40_GetVOCCategory(voc_index)`              | Index → catégorie qualité d'air |
| `SGP40_VOCCategoryToString(category)`          | Catégorie → chaîne lisible      |
| `SGP40_StatusToString(code)`                   | Code erreur → chaîne lisible    |

### 6.4 Fonctions convenience (warmup + mesure tout-en-un)

| Fonction                                      | Description                                                  |
| --------------------------------------------- | ------------------------------------------------------------ |
| `SGP40_MeasureVOCIndex(hsgp40, &raw, &index)` | MeasureRaw + CalculateVOCIndex en un appel                   |
| `SGP40_ReadAll(hsgp40, &data)`                | Wrapper métier : remplit `SGP40_Data` { voc_raw, voc_index } |
| `SGP40_IsVOCWarmupComplete(hsgp40)`           | Warmup terminé ? (dépend de sample_interval_ms)              |
| `SGP40_GetVOCWarmupSamples(hsgp40)`           | Nombre d'échantillons warmup = ceil(45 s / interval_s)       |

### 6.5 API avancée algorithme VOC (Sensirion)

| Fonction                                       | Description                      |
| ---------------------------------------------- | -------------------------------- |
| `SGP40_VOCAlgoReset(hsgp40)`                   | Réinitialise l'algo VOC          |
| `SGP40_VOCAlgoGetStates(hsgp40, &s0, &s1)`     | Exporte les états internes       |
| `SGP40_VOCAlgoSetStates(hsgp40, s0, s1)`       | Restaure les états internes      |
| `SGP40_VOCAlgoSetTuning(hsgp40, ...)`          | Configure le tuning (6 params)   |
| `SGP40_VOCAlgoGetTuning(hsgp40, ...)`          | Lit le tuning courant            |
| `SGP40_VOCAlgoGetSamplingInterval(hsgp40, &s)` | Lit l'intervalle algo (secondes) |

### 6.6 API asynchrone IT (uniquement — DMA non implémenté, aucun gain sur 3-8 octets)

> **Note sur la fréquence de mesure :** La fonction `SGP40_Async_TriggerEvery` ne prend plus de paramètre `interval_ms`. L'intervalle est géré en interne par la librairie (via `SGP40_SetSampleInterval`) pour garantir une synchronisation parfaite avec l'algorithme VOC de Sensirion (qui requiert typiquement 1Hz).

| Fonction                                                  | Description                                                                                     |
| --------------------------------------------------------- | ----------------------------------------------------------------------------------------------- |
| `SGP40_Async_Init(ctx, hsgp40)`                           | Initialise le contexte async                                                                    |
| `SGP40_Async_Reset(ctx)`                                  | Reset machine d'état (garde callbacks)                                                          |
| `SGP40_Async_SetCallbacks(ctx, on_data, on_err, user)`    | Callbacks main-loop                                                                             |
| `SGP40_Async_SetIrqCallbacks(ctx, on_data, on_err, user)` | Callbacks ISR                                                                                   |
| `SGP40_ReadAll_IT(ctx)`                                   | Lance mesure non-bloquante (IT, DMA non implémenté : aucun gain pour les trames 8B/3B)          |
| `SGP40_Async_TriggerEvery(ctx, now_ms, &last_ms)`         | Déclenche une mesure périodique                                                                 |
| `SGP40_Async_Process(ctx, now_ms)`                        | Machine d'état (appeler en boucle)                                                              |
| `SGP40_Async_DataReadyFlag(ctx)`                          | Flag donnée disponible                                                                          |
| `SGP40_Async_ErrorFlag(ctx)`                              | Flag erreur                                                                                     |
| `SGP40_Async_ClearFlags(ctx)`                             | Reset des flags                                                                                 |
| `SGP40_Async_IsIdle(ctx)`                                 | Machine au repos ?                                                                              |
| `SGP40_Async_HasData(ctx)`                                | Donnée prête à lire ?                                                                           |
| `SGP40_Async_GetData(ctx, &voc_raw)`                      | Récupère la donnée                                                                              |
| `SGP40_Async_Tick(ctx, now_ms, &voc_raw)`                 | Helper combinant Process + flags — retourne `SGP40_TickResult` (IDLE/BUSY/DATA_READY/ERROR)     |
| `SGP40_Async_TickIndex(ctx, now_ms, &voc_raw, &voc_idx)`  | Helper Tick + calcul VOC Index Sensirion — format d'entrée recommandé pour la boucle principale |
| `SGP40_Async_OnI2CMasterTxCplt(ctx, hi2c)`                | Relais callback HAL TX                                                                          |
| `SGP40_Async_OnI2CMasterRxCplt(ctx, hi2c)`                | Relais callback HAL RX                                                                          |
| `SGP40_Async_OnI2CError(ctx, hi2c)`                       | Relais callback HAL Error                                                                       |

---

## 7) Gestion d'erreurs

Toutes les fonctions synchrones retournent :

```c
typedef enum {
    SGP40_OK = 0,
    SGP40_ERR_NULL_PTR,
    SGP40_ERR_INVALID_PARAM,
    SGP40_ERR_NOT_CONFIGURED,
    SGP40_ERR_BUSY,
    SGP40_ERR_I2C,
    SGP40_ERR_CRC,
    SGP40_ERR_SELF_TEST,
    SGP40_ERR_NOT_READY,
    SGP40_ERR_TIMEOUT,
    SGP40_ERR_NOT_INITIALIZED,
    SGP40_ERR_OVERFLOW,
    SGP40_ERR_UNKNOWN
} SGP40_Status;
```

| Code                        | Signification                                 |
| --------------------------- | --------------------------------------------- |
| `SGP40_OK`                  | Succès                                        |
| `SGP40_ERR_NULL_PTR`        | Pointeur NULL passé en argument               |
| `SGP40_ERR_INVALID_PARAM`   | Paramètre hors plage                          |
| `SGP40_ERR_NOT_CONFIGURED`  | Config HAL/NVIC/DMA absente                   |
| `SGP40_ERR_BUSY`            | Cadence non respectée                         |
| `SGP40_ERR_I2C`             | Erreur bus I2C                                |
| `SGP40_ERR_CRC`             | CRC-8 invalide sur trame capteur              |
| `SGP40_ERR_SELF_TEST`       | Auto-test échoué                              |
| `SGP40_ERR_NOT_READY`       | Capteur non initialisé ou heater off          |
| `SGP40_ERR_TIMEOUT`         | Timeout mesure                                |
| `SGP40_ERR_NOT_INITIALIZED` | Handle non initialisé (SGP40_Init non appelé) |
| `SGP40_ERR_OVERFLOW`        | File de mesures async pleine (future use)     |
| `SGP40_ERR_UNKNOWN`         | Erreur inconnue                               |

---

## 8) Contraintes et comportements

- **Cadence** : `SGP40_MeasureRaw` et `SGP40_ReadAll_IT` retournent `SGP40_ERR_BUSY` si appelées avant l'intervalle configuré.
- **Intervalle** : `SGP40_SetSampleInterval` accepte 500..10000 ms.
- **Self-test** : bloque 320 ms, valide le pattern `0xD4XX`.
- **HeaterOff** : remet le handle en état non-initialisé → refaire `SGP40_Init`.
- **VOC Index** : nécessite un algo initialisé (fait par `Init`) et une cadence stable.
- **Compensation** : les valeurs T/RH ne sont jamais réinitialisées silencieusement (contrairement à d'autres librairies).
- **Portabilité** : `#include "main.h"` (pas de dépendance directe à `stm32l4xx_hal.h`).

### Résumé décision async (Q1→Q4)

- Q1 : mesure lente (~30 ms) → async utile.
- Q2 : callbacks I2C IT disponibles et fiables sur STM32 HAL.
- Q3 : trames courtes (3–8 octets) → DMA sans gain réel.
- Q4 : coexistence multi-capteurs sur bus I2C → IT plus robuste pour arbitrage `HAL_BUSY`.

---

## 9) Intégration asynchrone

### Callbacks HAL (dans `USER CODE BEGIN 4`)

```c
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    SGP40_Async_OnI2CMasterTxCplt(&sgp40_async, hi2c);
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    SGP40_Async_OnI2CMasterRxCplt(&sgp40_async, hi2c);
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    SGP40_Async_OnI2CError(&sgp40_async, hi2c);
}
```

### Boucle principale (pattern recommandé : TriggerEvery + TickIndex)

```c
uint32_t last_ms = 0U;

while (1) {
    uint32_t now = HAL_GetTick();

    /* Déclenchement périodique (utilise hsgp40.sample_interval_ms) */
    SGP40_Async_TriggerEvery(&sgp40_async, now, &last_ms);

    /* Avance FSM + récupère donnée si prête */
    uint16_t voc_raw = 0U, voc_index = 0U;
    SGP40_TickResult tick = SGP40_Async_TickIndex(&sgp40_async, now,
                                                   &voc_raw, &voc_index);
    switch (tick) {
        case SGP40_TICK_DATA_READY:
            printf("VOC raw=%u  index=%u  %s\r\n",
                   voc_raw, voc_index,
                   SGP40_VOCCategoryToString(SGP40_GetVOCCategory(voc_index)));
            break;
        case SGP40_TICK_ERROR:
            printf("Erreur async SGP40 : %s\r\n",
                   SGP40_StatusToString(sgp40_async.last_status));
            break;
        default:
            break;
    }
}
```

### Diagramme états FSM

```
                     SGP40_ReadAll_IT()
                           │
                   ┌───────▼───────┐
                   │  WRITE_CMD    │  ← HAL_I2C_Master_Transmit_IT (8 B)
                   └───────┬───────┘
                           │ OnI2CMasterTxCplt
                   ┌───────▼───────┐
                   │  WAIT_MEAS    │  ← attente SGP40_MEASURE_WAIT_MS (30 ms, Process/SysTick)
                   └───────┬───────┘
                           │ délai écoulé → HAL_I2C_Master_Receive_IT (3 B)
                   ┌───────▼───────┐
                   │  READ_DATA    │  ← en attente RxCplt
                   └───────┬───────┘
                           │ OnI2CMasterRxCplt (CRC ok)
                   ┌───────▼───────┐
                   │     DONE      │  ← data_ready_flag = true
                   └───────┬───────┘
                           │ GetData() / Tick() consomme
                   ┌───────▼───────┐
                   │     IDLE      │
                   └───────────────┘

  Erreur (NACK / CRC invalide / timeout) ──► ERROR ──► Reset() ──► IDLE
  HAL_BUSY sur TX ──────────────────────────────────────────────► IDLE
  (TriggerEvery retentera automatiquement au prochain cycle)
```

### Intégration FreeRTOS

- Remplacer `HAL_GetTick()` par `osKernelGetTickCount()` si la timebase FreeRTOS est activée dans CubeMX.
- Ne jamais appeler `HAL_Delay()` dans la tâche de mesure — utiliser `osDelay()` ou `vTaskDelay()`.
- Si le bus I2C est partagé entre plusieurs tâches FreeRTOS, protéger les appels `TriggerEvery` / `TickIndex` avec un mutex (les callbacks IRQ restent sans mutex — ils n'accèdent qu'à des champs `volatile`).
- Appeler `SGP40_Init()` et `SGP40_Async_Init()` depuis une **tâche d'initialisation** uniquement — jamais depuis une IRQ ni depuis `main()` avant `vTaskStartScheduler()`.

---

## 10) Exemples

Dossier : `exemples/` — index détaillé dans `exemples/README.md`.

| Fichier                                       | Description                                                      |
| --------------------------------------------- | ---------------------------------------------------------------- |
| `exemple_sgp40_polling.c`                     | Polling de base : mesure simple VOC raw + index (point d'entrée) |
| `exemple_sgp40_polling_statistiques.c`        | Polling avancé : suivi continu + statistiques (min/max/moy)      |
| `exemple_sgp40_polling_diagnostic.c`          | Polling diagnostic : self-test, compensation, monitoring         |
| `exemple_sgp40_polling_selftest.c`            | Polling validation complète : serial, self-test, API avancée     |
| `exemple_sgp40_async_it.c`                    | Chaîne asynchrone complète en interruptions I2C                  |
| `exemple_sgp40_async_multi_capteurs.c`        | SGP40 async IT + BME280 sur le même bus I2C partagé (no DMA)     |
| `exemple_sgp40_polling_aht20_compensation.c`  | Polling SGP40 + AHT20 : compensation T/RH en temps réel          |
| `exemple_sgp40_async_it_aht20_compensation.c` | Async IT SGP40 + AHT20 : compensation T/RH non-bloquante         |

### Pattern harmonisé (tous les exemples)

```c
/* PV — Variables globales */
static SGP40_HandleTypeDef hsgp40;  ///< Handle principal du capteur SGP40

/* main() — USER CODE 2 */
SGP40_Status init_status = SGP40_Init(&hsgp40, &hi2c3);
if (init_status != SGP40_OK) {
    printf("ERREUR Init SGP40: %s\r\n", SGP40_StatusToString(init_status));
    Error_Handler();
}
SGP40_SetCompensation(&hsgp40, 25.0f, 50.0f);
```

---

## 11) Conformité

- Timing mesure SGP40 respecté (30 ms)
- Timing self-test datasheet (320 ms max)
- CRC-8 vérifié sur toutes les trames capteur
- Algorithme Sensirion GasIndexAlgorithm v3.2 (dernière version)
- Plage compensation T/RH conforme datasheet (−45..+130 °C / 0..100 %RH)
- Convention CubeMX `USER CODE BEGIN/END` respectée dans tous les exemples
- Aucun `HAL_Delay` après `__disable_irq()` (Error_Handler sécurisé)

---

## 12) Dépannage

| Erreur                     | Diagnostic                                    |
| -------------------------- | --------------------------------------------- |
| `SGP40_ERR_I2C`            | Câblage SDA/SCL, pull-up, adresse, timing I2C |
| `SGP40_ERR_CRC`            | Bruit bus/alimentation, longueur câbles       |
| `SGP40_ERR_BUSY`           | Cadence trop rapide → augmenter intervalle    |
| `SGP40_ERR_NOT_READY`      | Driver non initialisé ou `HeaterOff` appelé   |
| `SGP40_ERR_NOT_CONFIGURED` | NVIC I2C EV/ER non activé (mode IT)           |

---

## 13) Paramètres configurables (`STM32_SGP40_conf.h`)

Tous les paramètres sont surchargeable via `#define` avant l'inclusion de `STM32_SGP40.h`
ou via les options de compilation (`-DSGP40_MAX_CONSECUTIVE_ERRORS=5`).

| Macro                          | Défaut | Unité | Description                                                               |
| ------------------------------ | :----: | :---: | ------------------------------------------------------------------------- |
| `SGP40_DEFAULT_TIMEOUT_MS`     | `100`  |  ms   | Timeout I2C pour les transactions bloquantes                              |
| `SGP40_MEASURE_WAIT_MS`        |  `30`  |  ms   | Délai mesure VOC raw (datasheet §5.1)                                     |
| `SGP40_SELFTEST_WAIT_MS`       | `320`  |  ms   | Délai auto-test (datasheet §5.2)                                          |
| `SGP40_MAX_CONSECUTIVE_ERRORS` |  `3`   |   —   | Seuil erreurs I2C consécutives avant abandon                              |
| `SGP40_SERIAL_READ_DELAY_MS`   |  `1`   |  ms   | Délai post-commande GetSerialNumber avant lecture (datasheet §5.3 < 1 ms) |
| `SGP40_ASYNC_BUSY_RETRY_MS`    |  `2`   |  ms   | Délai retry RX HAL_BUSY (bus partagé multi-capteurs, async IT)            |
| `SGP40_DEBUG_ENABLE`           |   —    |   —   | Active `SGP40_StatusToString()` et les traces série                       |

> **Production** : `SGP40_DEBUG_ENABLE` doit rester commenté pour économiser la Flash.  
> Une erreur `HAL_BUSY` ne compte pas comme erreur consécutive.

---

## 14) Version

- **Driver** : 0.9.0
- **Algorithme** : Sensirion GasIndexAlgorithm v3.2
- **API** : alignée sur `STM32_SGP40.h` et `STM32_SGP40.c` actuels

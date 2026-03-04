# Exemples STM32_SGP40

Exemples de référence couvrant **100 % de l'API publique** de `STM32_SGP40.h`.

Emplacement : `Drivers/STM32_SGP40/exemples/`

---

## Fichiers

| Fichier                                       | Objectif                                                                                       | Difficulté    |
| --------------------------------------------- | ---------------------------------------------------------------------------------------------- | ------------- |
| `exemple_sgp40_polling.c`                     | Polling de base : init → mesure raw → VOC index → catégorie                                    | Débutant      |
| `exemple_sgp40_polling_statistiques.c`        | Polling avancé : suivi continu + stats min/max/moy + serial number                             | Intermédiaire |
| `exemple_sgp40_polling_diagnostic.c`          | Polling diagnostic complet : self-test, compensation, monitoring                               | Intermédiaire |
| `exemple_sgp40_polling_selftest.c`            | Polling validation capteur : serial, self-test, API avancée VOC, heater off/re-init + synthèse | Avancé        |
| `exemple_sgp40_async_it.c`                    | Chaîne asynchrone complète en interruptions I2C                                                | Avancé        |
| `exemple_sgp40_async_multi_capteurs.c`        | SGP40 async IT + BME280 polling sur le même bus I2C (partage coopératif)                       | Avancé        |
| `exemple_sgp40_polling_aht20_compensation.c`  | Polling SGP40 + AHT20 : compensation T/RH en temps réel                                        | Intermédiaire |
| `exemple_sgp40_async_it_aht20_compensation.c` | Async IT SGP40 + AHT20 : compensation T/RH non-bloquante sur bus partagé                       | Avancé        |

---

## Pré-requis CubeMX communs à tous les exemples

- **I2C** (I2C1 ou I2C3) : Mode I2C, 100 kHz ou 400 kHz, Addressing 7-bit, Pull-ups externes 4,7 kΩ sur SCL/SDA
- **NVIC** (exemples async IT uniquement) : I2C event interrupt + I2C error interrupt **Enabled**
- **USART2** : 115 200 bauds, 8N1, Asynchronous (pour `printf` → UART2 via `__io_putchar`)

> **Note IT vs DMA** : DMA non implémenté — trames SGP40 = 3 à 8 octets, overhead DMA > gain CPU. Mode IT suffit et simplifie le partage de bus multi-capteurs.

---

## Pattern harmonisé

Tous les exemples suivent le même schéma d'initialisation :

```c
/* PV — Variables globales */
SGP40_Handle_t hsgp40;  ///< Handle principal du capteur SGP40

/* main() — USER CODE 2 */
SGP40_Status init_status = SGP40_Init(&hsgp40, &hi2c3);
if (init_status != SGP40_OK) {
    printf("ERREUR Init SGP40: %s\r\n", SGP40_StatusToString(init_status));
    Error_Handler();
}
SGP40_SetCompensation(&hsgp40, 25.0f, 50.0f);
```

La bibliothèque utilise `HAL_GetTick()` et `HAL_Delay()` en interne — aucun wrapper de timebase n'est nécessaire.

---

## Matrice API → exemple

### Configuration et initialisation

| API                       | lecture_simple | voc_stats | debug | selftest | async_it | multi_capt | aht20_poll | aht20_async |
| ------------------------- | :------------: | :-------: | :---: | :------: | :------: | :--------: | :--------: | :---------: |
| `SGP40_Init`              |       x        |     x     |   x   |    x     |    x     |     x      |     x      |      x      |
| `SGP40_DeInit`            |       x        |     x     |   x   |    x     |    x     |     x      |     x      |      x      |
| `SGP40_SetSampleInterval` |       x        |     x     |   x   |    x     |    x     |     x      |     x      |      x      |
| `SGP40_SetCompensation`   |       x        |     x     |   x   |    x     |    x     |     x      |     x      |      x      |

### Mesure et diagnostic

| API                      | lecture_simple | voc_stats | debug | selftest | async_it | multi_capt | aht20_poll | aht20_async |
| ------------------------ | :------------: | :-------: | :---: | :------: | :------: | :--------: | :--------: | :---------: |
| `SGP40_MeasureRaw`       |                |           |   x   |    x     |          |            |            |             |
| `SGP40_GetSerialNumber`  |       x        |     x     |   x   |    x     |    x     |     x      |     x      |      x      |
| `SGP40_SelfTest`         |                |           |   x   |    x     |          |            |            |             |
| `SGP40_HeaterOff`        |                |           |       |    x     |          |            |            |             |
| `SGP40_PrimeForVocIndex` |       x        |     x     |       |    x     |    x     |     x      |     x      |      x      |

### Calcul VOC Index

| API                         | lecture_simple | voc_stats | debug | selftest | async_it | multi_capt | aht20_poll | aht20_async |
| --------------------------- | :------------: | :-------: | :---: | :------: | :------: | :--------: | :--------: | :---------: |
| `SGP40_CalculateVOCIndex`   |                |           |   x   |    x     |    x     |     x      |            |             |
| `SGP40_GetVOCCategory`      |       x        |     x     |   x   |    x     |    x     |     x      |     x      |      x      |
| `SGP40_VOCCategoryToString` |       x        |     x     |   x   |    x     |    x     |     x      |     x      |      x      |
| `SGP40_StatusToString`      |       x        |     x     |   x   |    x     |    x     |     x      |     x      |      x      |

### Fonctions convenience (warmup + mesure tout-en-un)

| API                         | lecture_simple | voc_stats | async_it | multi_capt | aht20_poll | aht20_async |
| --------------------------- | :------------: | :-------: | :------: | :--------: | :--------: | :---------: |
| `SGP40_MeasureVOCIndex`     |       x        |     x     |          |            |     x      |             |
| `SGP40_IsVOCWarmupComplete` |       x        |     x     |    x     |     x      |     x      |      x      |
| `SGP40_GetVOCWarmupSamples` |       x        |     x     |    x     |     x      |     x      |      x      |

### API avancée algorithme VOC

| API                                | selftest |
| ---------------------------------- | :------: |
| `SGP40_VOCAlgoReset`               |    x     |
| `SGP40_VOCAlgoGetStates`           |    x     |
| `SGP40_VOCAlgoSetStates`           |    x     |
| `SGP40_VOCAlgoSetTuning`           |    x     |
| `SGP40_VOCAlgoGetTuning`           |    x     |
| `SGP40_VOCAlgoGetSamplingInterval` |    x     |

### API asynchrone IT (mode IT uniquement — DMA non implémenté, aucun gain sur 3-8 octets)

| API                             | async_it | multi_capt | aht20_async |
| ------------------------------- | :------: | :--------: | :---------: | ----------------------------------------- |
| `SGP40_Async_Init`              |    x     |     x      |      x      |
| `SGP40_Async_Reset`             |    x     |            |             |
| `SGP40_Async_SetCallbacks`      |    x     |            |             |
| `SGP40_Async_SetIrqCallbacks`   |    x     |            |             |
| `SGP40_ReadAll_IT`              |    x     |     x      |      x      |
| `SGP40_Async_Process`           |    x     |     x      |             |
| `SGP40_Async_DataReadyFlag`     |    x     |            |             |
| `SGP40_Async_ErrorFlag`         |    x     |            |             |
| `SGP40_Async_ClearFlags`        |    x     |            |             |
| `SGP40_Async_IsIdle`            |    x     |     x      |             |
| `SGP40_Async_HasData`           |    x     |            |             |
| `SGP40_Async_GetData`           |    x     |            |             |
| `SGP40_Async_TriggerEvery`      |    x     |     x      |      x      |
| `SGP40_Async_Tick`              |          |            |      x      | _(couvert via `TickIndex` dans async_it)_ |
| `SGP40_Async_TickIndex`         |    x     |     x      |      x      |
| `SGP40_Async_OnI2CMasterTxCplt` |    x     |     x      |      x      |
| `SGP40_Async_OnI2CMasterRxCplt` |    x     |     x      |      x      |
| `SGP40_Async_OnI2CError`        |    x     |     x      |      x      |

---

## Conventions

- `PrintPhase()` est une fonction locale dans `exemple_sgp40_polling.c` (affiche une bannière de phase console) — conservée pour la lisibilité pédagogique de l'exemple de base, non dupliquée dans les autres fichiers.
- Tous les exemples sont des templates CubeMX (`main.c`) avec zones `USER CODE BEGIN/END`
- Aucun `HAL_Delay` après `__disable_irq()` dans `Error_Handler` (busy-wait volatile)
- Toutes les temporisations utilisent `HAL_Delay()` directement (la lib gère son timing en interne)
- Nom du fichier affiché dans la bannière d'init via `LOG_NAME`
- Gestion d'erreur via `SGP40_StatusToString()` (pas de magic numbers)
- `printf` redirigé vers UART2 via `__io_putchar()` dans tous les exemples

## Adaptation à votre projet

Pour un projet final, conservez la logique métier et adaptez :

1. Le bus I2C dans `SGP40_Init()` (ex: `&hi2c1` au lieu de `&hi2c3`)
2. L'intervalle de mesure via `SGP40_SetSampleInterval()` (par défaut 1000 ms — abaisser à 250 ms possible si la datasheet le permet)
3. Les valeurs de compensation T/RH via `SGP40_SetCompensation()` (connecter un AHT20 ou BME280 pour la compensation réelle)
4. Le seuil d'erreurs consécutives via `SGP40_MAX_CONSECUTIVE_ERRORS` dans `STM32_SGP40_conf.h`
5. Les paramètres UART si console différente

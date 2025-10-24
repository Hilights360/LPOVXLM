#pragma once

#include <Arduino.h>
#include "ConfigTypes.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// SD hardware pin assignments
extern const int PIN_SD_CLK;
extern const int PIN_SD_CMD;
extern const int PIN_SD_D0;
extern const int PIN_SD_D1;
extern const int PIN_SD_D2;
extern const int PIN_SD_D3;
extern const int PIN_SD_CD;

// Shared SD state
extern SemaphoreHandle_t g_sdMutex;
extern SdBusPreference g_sdPreferredBusWidth;
extern uint32_t g_sdBaseFreqKHz;
extern uint32_t g_sdFreqKHz;
extern int g_sdFailStreak;
extern bool g_sdReady;
extern uint8_t g_sdBusWidth;

bool SD_LOCK(TickType_t timeout = portMAX_DELAY);
void SD_UNLOCK();
void ensureBgEffectsDirLocked();

SdBusPreference sanitizeSdMode(uint8_t mode);
bool isValidSdFreq(uint32_t freq);
uint32_t sanitizeSdFreq(uint32_t freq);
uint32_t nextLowerSdFreq(uint32_t freq);

bool cardPresent();
bool mountSdmmc();

void persistSettingsToSd();
void ensureSettingsFromBackup(const PrefPresence &present);
void checkSdFirmwareUpdate();

void listFseqInDir(const char *path, String &optionsHtml, uint8_t depth = 0);
void listBgEffects(String &optionsHtml, const String &current);

void handleFiles();
void handleDownload();
void handlePlayLink();
void handleDelete();
void handleMkdir();
void handleRename();
void handleUploadData();
void handleUploadDone();

void handleSdReinit();
void handleSdConfig();

void handleUpdatesPage();
void handleOtaPage();
void handleOtaData();
void handleOtaFinish();
void handleFwUploadData();
void handleFwUploadDone();

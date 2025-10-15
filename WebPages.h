#pragma once

#include <Arduino.h>
#include <vector>

#include "WebUtils.h"

struct ControlPageContext {
  String optionsHtml;
  String currentPath;
  String statusClass;
  String statusText;
  uint32_t startChannel;
  uint16_t totalSpokes;
  uint8_t armCount;
  uint16_t pixelsPerArm;
  uint8_t strideMode; // 0 = spoke, 1 = led
  uint16_t maxArms;
  uint16_t maxPixelsPerArm;
  uint16_t fps;
  uint8_t brightnessPercent;
};

struct FilesPageEntry {
  String name;
  bool isDirectory;
  uint64_t size;
};

struct FilesPageContext {
  String path;
  String parentPath;
  const std::vector<FilesPageEntry> *entries;
};

String buildControlPage(const ControlPageContext &ctx);
String buildFilesPage(const FilesPageContext &ctx);


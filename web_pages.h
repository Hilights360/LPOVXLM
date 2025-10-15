#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "spinner_types.h"

namespace WebPages {

struct ControlPageData {
  String optionsHtml;
  String currentPath;
  const char* statusClass = nullptr;
  const char* statusText = nullptr;
  uint32_t startChannel = 0;
  uint16_t totalSpokes = 0;
  uint8_t armCount = 0;
  uint8_t maxArms = 0;
  uint16_t pixelsPerArm = 0;
  uint16_t maxPixelsPerArm = 0;
  StrideMode strideMode = STRIDE_SPOKE;
  uint16_t fps = 0;
  uint8_t brightnessPercent = 0;
};

struct FilesPageData {
  String currentPath;
  String parentPath;
  String currentPathEncoded;
  String parentPathEncoded;
  String uploadBackEncoded;
  String rowsHtml;
};

String buildControlPage(const ControlPageData& data);
String buildFilesPage(const FilesPageData& data);
String buildDirectoryRow(const String& fullName, const String& currentPathEncoded);
String buildFileRow(const String& fullName, uint64_t size, const String& currentPathEncoded);
String buildUploadRejectedPage(const String& backUrl);
String buildUploadFailedPage(const String& backUrl);
String buildUploadSuccessPage(const String& filename, size_t bytes, const String& backUrl);

}  // namespace WebPages

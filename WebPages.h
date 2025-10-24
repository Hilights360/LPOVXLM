#pragma once

#include <Arduino.h>

namespace WebPages {

String filesPageHeader(const String &pathEscaped,
                       const String &parentEncoded,
                       const String &currentPathEncoded,
                       const String &backEncoded,
                       const String &currentPathAttrEscaped,
                       const String &backAttrEscaped);

String filesDirectoryRow(const String &displayNameEscaped,
                         const String &linkEncoded,
                         const String &confirmNameEscaped,
                         const String &renameDefaultEscaped,
                         const String &backParam);

String filesFileRow(const String &displayNameEscaped,
                    const String &linkEncoded,
                    uint64_t size,
                    const String &confirmNameEscaped,
                    const String &renameDefaultEscaped,
                    const String &backParam);

String filesPageFooter();

String rootPage(const String &statusClass,
                const String &statusText,
                const String &currentFileEscaped,
                const String &optionsHtml,
                const String &apSsid,
                const String &apIp,
                const String &mdnsName,
                const String &staSsid,
                const String &staStatus,
                const String &staIp,
                const String &stationId,
                uint32_t startChannel,
                uint16_t spokes,
                uint8_t arms,
                uint16_t pixelsPerArm,
                uint8_t maxArms,
                uint16_t maxPixelsPerArm,
                bool strideIsSpoke,
                uint16_t fps,
                uint8_t brightnessPercent,
                uint8_t sdPreferredMode,
                uint32_t sdBaseFreqKHz,
                uint8_t sdActiveWidth,
                uint32_t sdActiveFreqKHz,
                bool sdReady,
                bool playing,
                bool paused,
                bool autoplayEnabled,
                bool hallDiagEnabled,
                bool watchdogEnabled,
                bool bgEffectEnabled,
                bool bgEffectActive,
                const String &bgEffectCurrentEscaped,
                const String &bgEffectOptionsHtml);

// NEW: Standalone Updates page (Upload + Reboot UI)
// Pass canReboot=true to enable the Reboot button (e.g., after successful upload)
String updatesPage(bool canReboot);

String directOtaPage();
String uploadRejectedPage(const String &backUrl);
String uploadFailurePage(const String &backUrl);
String uploadSuccessPage(const String &backUrl,
                         const String &filename,
                         size_t bytesWritten);

}  // namespace WebPages

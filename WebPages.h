#pragma once

#include <Arduino.h>

namespace WebPages {

String filesPageHeader(const String &pathEscaped,
                       const String &parentEncoded,
                       const String &currentPathEncoded,
                       const String &backEncoded);

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
                uint32_t startChannel,
                uint16_t spokes,
                uint8_t arms,
                uint16_t pixelsPerArm,
                uint8_t maxArms,
                uint16_t maxPixelsPerArm,
                bool strideIsSpoke,
                uint16_t fps,
                uint8_t brightnessPercent,
                const String &wifiStatusEscaped,
                const String &wifiSsidEscaped,
                bool wifiConfigured);

}  // namespace WebPages


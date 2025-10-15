#include "WebUtils.h"

namespace {
constexpr bool isSafeChar(uint8_t c) {
  return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
         c == '-' || c == '_' || c == '.' || c == '/';
}
}

String htmlEscape(const String &in) {
  String s;
  s.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); ++i) {
    char c = in[i];
    switch (c) {
      case '&': s += "&amp;"; break;
      case '<': s += "&lt;"; break;
      case '>': s += "&gt;"; break;
      case '\"': s += "&quot;"; break;
      case '\'': s += "&#39;"; break;
      default: s += c; break;
    }
  }
  return s;
}

String urlEncode(const String &in) {
  static const char hex[] = "0123456789ABCDEF";
  String s;
  s.reserve(in.length() * 3);
  for (size_t i = 0; i < in.length(); ++i) {
    uint8_t c = static_cast<uint8_t>(in[i]);
    if (isSafeChar(c)) {
      s += static_cast<char>(c);
    } else {
      s += '%';
      s += hex[c >> 4];
      s += hex[c & 0x0F];
    }
  }
  return s;
}

String dirnameOf(const String &path) {
  if (path == "/") return "/";
  int slash = path.lastIndexOf('/');
  if (slash <= 0) return "/";
  return path.substring(0, slash);
}

String joinPath(const String &dir, const String &name) {
  String d = dir;
  if (!d.length() || d[0] != '/') d = "/" + d;
  if (d != "/" && d.endsWith("/")) d.remove(d.length() - 1);
  String base = name;
  int slash = base.lastIndexOf('/');
  if (slash >= 0) base = base.substring(slash + 1);
  return (d == "/") ? ("/" + base) : (d + "/" + base);
}

String baseName(const String &path) {
  int slash = path.lastIndexOf('/');
  if (slash >= 0) {
    return path.substring(slash + 1);
  }
  return path;
}


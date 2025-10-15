#pragma once

#include <Arduino.h>

inline String htmlEscape(const String& in) {
  String s;
  s.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); ++i) {
    char c = in[i];
    if (c == '&') s += "&amp;";
    else if (c == '<') s += "&lt;";
    else if (c == '>') s += "&gt;";
    else if (c == '\"') s += "&quot;";
    else if (c == '\'') s += "&#39;";
    else s += c;
  }
  return s;
}

inline String urlEncode(const String& in) {
  const char* hex = "0123456789ABCDEF";
  String s;
  s.reserve(in.length() * 3);
  for (size_t i = 0; i < in.length(); ++i) {
    uint8_t c = static_cast<uint8_t>(in[i]);
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
        c == '-' || c == '_' || c == '.' || c == '/') {
      s += static_cast<char>(c);
    } else {
      s += '%';
      s += hex[c >> 4];
      s += hex[c & 0x0F];
    }
  }
  return s;
}

inline String dirnameOf(const String& path) {
  if (path == "/") return "/";
  int slash = path.lastIndexOf('/');
  if (slash <= 0) return "/";
  return path.substring(0, slash);
}

inline String joinPath(const String& dir, const String& name) {
  String d = dir;
  if (!d.length() || d[0] != '/') d = "/" + d;
  if (d != "/" && d.endsWith("/")) d.remove(d.length() - 1);
  String base = name;
  int slash = base.lastIndexOf('/');
  if (slash >= 0) base = base.substring(slash + 1);
  return (d == "/") ? ("/" + base) : (d + "/" + base);
}

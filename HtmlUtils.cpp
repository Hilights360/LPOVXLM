#include "HtmlUtils.h"

#include <string.h>

const char* const BG_EFFECTS_DIR = "/BGEffects";

bool isFseqName(const String& name) {
  int dot = name.lastIndexOf('.');
  if (dot < 0) return false;
  String ext = name.substring(dot + 1);
  ext.toLowerCase();
  return ext == "fseq";
}

bool isBgEffectPath(const String& path) {
  const size_t prefixLen = strlen(BG_EFFECTS_DIR);
  if (path.length() <= prefixLen) return false;
  if (!path.startsWith(BG_EFFECTS_DIR)) return false;
  return path.charAt(prefixLen) == '/';
}

String sanitizeBgEffectPath(const String& in) {
  String path = in;
  path.trim();
  if (!path.length()) return String();
  if (path.indexOf("..") >= 0) return String();
  if (path[0] != '/') path = "/" + path;
  if (!isBgEffectPath(path)) return String();
  if (!isFseqName(path)) return String();
  return path;
}

String bgEffectDisplayName(const String& path) {
  if (isBgEffectPath(path)) {
    size_t prefixLen = strlen(BG_EFFECTS_DIR);
    if (path.length() > prefixLen + 1) {
      return path.substring(prefixLen + 1);
    }
  }
  String name = path;
  int slash = name.lastIndexOf('/');
  if (slash >= 0) name = name.substring(slash + 1);
  return name;
}

String htmlEscape(const String& in) {
  String s;
  s.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); ++i) {
    char c = in[i];
    if (c == '&')       s += "&amp;";
    else if (c == '<')  s += "&lt;";
    else if (c == '>')  s += "&gt;";
    else if (c == '\"') s += "&quot;";
    else if (c == '\'') s += "&#39;";
    else s += c;
  }
  return s;
}

String urlEncode(const String& in) {
  const char* hex = "0123456789ABCDEF";
  String s;
  s.reserve(in.length() * 3);
  for (size_t i = 0; i < in.length(); ++i) {
    uint8_t c = (uint8_t)in[i];
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
        (c >= '0' && c <= '9') || c == '-' || c == '_' ||
        c == '.' || c == '/') {
      s += (char)c;
    } else {
      s += '%';
      s += hex[c >> 4];
      s += hex[c & 0xF];
    }
  }
  return s;
}

String dirnameOf(const String& path) {
  if (path == "/") return "/";
  int slash = path.lastIndexOf('/');
  if (slash <= 0) return "/";
  return path.substring(0, slash);
}

String joinPath(const String& dir, const String& name) {
  String d = dir;
  if (!d.length() || d[0] != '/') d = "/" + d;
  if (d != "/" && d.endsWith("/")) d.remove(d.length() - 1);
  String base = name;
  int slash = base.lastIndexOf('/');
  if (slash >= 0) base = base.substring(slash + 1);
  return (d == "/") ? ("/" + base) : (d + "/" + base);
}

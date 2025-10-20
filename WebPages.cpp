#include "WebPages.h"

namespace {
String htmlEscape(const String &in) {
  String s; s.reserve(in.length()+8);
  for (size_t i=0;i<in.length();++i) {
    char c = in[i];
    if (c=='&') s += "&amp;";
    else if (c=='<') s += "&lt;";
    else if (c=='>') s += "&gt;";
    else if (c=='\"') s += "&quot;";
    else if (c=='\'') s += "&#39;";
    else s += c;
  }
  return s;
}
}

namespace WebPages {

String filesPageHeader(const String &pathEscaped,
                       const String &parentEncoded,
                       const String &currentPathEncoded,
                       const String &backEncoded) {
  String html =
      "<!doctype html><html><head><meta charset='utf-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>Files - " + pathEscaped + "</title>"
      "<style>"
      "body{font:16px system-ui,Segoe UI,Roboto,Arial;background:#0b1320;color:#e8ecf1;margin:0;padding:1rem}"
      ".card{max-width:860px;margin:0 auto;background:#121b2d;padding:1rem;border-radius:12px}"
      "a{color:#a7c3ff;text-decoration:none}a:hover{text-decoration:underline}"
      "table{width:100%;border-collapse:collapse;margin-top:.5rem}"
      "th,td{padding:.5rem;border-bottom:1px solid #1b2741}"
      ".row{display:flex;gap:.5rem;flex-wrap:wrap;margin-top:.5rem}"
      "button{padding:.4rem .7rem;border:0;border-radius:8px;background:#1c2b4a;color:#e8ecf1;cursor:pointer}"
      "button:hover{filter:brightness(1.1)}"
      "input{padding:.45rem .5rem;border-radius:8px;border:1px solid #253756;background:#0e1627;color:#e8ecf1}"
      "</style></head><body><div class='card'>"
      "<div style='display:flex;justify-content:space-between;align-items:center'>"
      "<h2 style='margin:0'>Files</h2>"
      "<a href='/'>Back to Control</a>"
      "</div>"
      "<p>Path: <b>" + pathEscaped + "</b> &middot; "
      "<a href='/files?path=" + parentEncoded + "'>Up</a></p>"
      "<form method='POST' action='/upload?dir=" + currentPathEncoded + "&back=" + backEncoded + "' enctype='multipart/form-data'>"
      "<input type='file' name='f' accept='.fseq' required> "
      "<button type='submit'>Upload here</button>"
      "<div class='muted' style='margin-top:.25rem'>Only <b>.fseq</b> files are accepted.</div>"
      "</form>"
      "<div class='row' style='margin-top:.75rem'>"
      "<button onclick=\"const n=prompt('New folder name'); if(n) location='/mkdir?path=" + currentPathEncoded + "&name='+encodeURIComponent(n);\">New Folder</button>"
      "<button onclick=\"location.reload()\">Refresh</button>"
      "</div>"
      "<table><thead><tr><th>Name</th><th>Size</th><th>Actions</th></tr></thead><tbody>";
  return html;
}

String filesDirectoryRow(const String &displayNameEscaped,
                         const String &linkEncoded,
                         const String &confirmNameEscaped,
                         const String &renameDefaultEscaped,
                         const String &backParam) {
  String row =
      "<tr><td>📁 <a href='/files?path=" + linkEncoded + "'>" + displayNameEscaped + "</a></td>"
      "<td>—</td>"
      "<td>"
      "<a href='#' onclick=\"if(confirm('Delete folder " + confirmNameEscaped + "? (must be empty)')) location='/rm?path=" + linkEncoded + "&back=" + backParam + "'; return false;\">🗑️ Delete</a> &nbsp; "
      "<a href='#' onclick=\"const n=prompt('Rename folder to:', '" + renameDefaultEscaped + "'); if(n) location='/ren?path=" + linkEncoded + "&to='+encodeURIComponent(n)+'&back=" + backParam + "'; return false;\">✏️ Rename</a>"
      "</td></tr>";
  return row;
}

String filesFileRow(const String &displayNameEscaped,
                    const String &linkEncoded,
                    uint64_t size,
                    const String &confirmNameEscaped,
                    const String &renameDefaultEscaped,
                    const String &backParam) {
  String row =
      "<tr><td>📄 " + displayNameEscaped + "</td>"
      "<td>" + String((unsigned long)size) + "</td>"
      "<td>"
      "<a href='/dl?path=" + linkEncoded + "'>⬇️ Download</a> &nbsp; "
      "<a href='/play?path=" + linkEncoded + "&back=" + backParam + "'>▶️ Play</a> &nbsp; "
      "<a href='#' onclick=\"if(confirm('Delete file " + confirmNameEscaped + "?')) location='/rm?path=" + linkEncoded + "&back=" + backParam + "'; return false;\">🗑️ Delete</a> &nbsp; "
      "<a href='#' onclick=\"const n=prompt('Rename file to:', '" + renameDefaultEscaped + "'); if(n) location='/ren?path=" + linkEncoded + "&to='+encodeURIComponent(n)+'&back=" + backParam + "'; return false;\">✏️ Rename</a>"
      "</td></tr>";
  return row;
}

String filesPageFooter() {
  return "</tbody></table></div></body></html>";
}

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
                bool watchdogEnabled) {
  const char *spokeSel = strideIsSpoke ? "selected" : "";
  const char *ledSel = strideIsSpoke ? "" : "selected";

  const char *sdAutoSel = (sdPreferredMode == 0) ? "selected" : "";
  const char *sd4Sel   = (sdPreferredMode == 4) ? "selected" : "";
  const char *sd1Sel   = (sdPreferredMode == 1) ? "selected" : "";

  const char *freq8Sel = (sdBaseFreqKHz == 8000) ? "selected" : "";
  const char *freq4Sel = (sdBaseFreqKHz == 4000) ? "selected" : "";
  const char *freq2Sel = (sdBaseFreqKHz == 2000) ? "selected" : "";
  const char *freq1Sel = (sdBaseFreqKHz == 1000) ? "selected" : "";
  const char *freq0Sel = (sdBaseFreqKHz == 400)  ? "selected" : "";

  String sdCurrent = "Current: ";
  if (!sdReady) {
    sdCurrent += "Card not mounted";
  } else {
    if (sdActiveWidth == 0) {
      sdCurrent += "Mounted (width unknown)";
    } else {
      sdCurrent += String((unsigned int)sdActiveWidth) + "-bit";
    }
    sdCurrent += " @ " + String((unsigned long)sdActiveFreqKHz) + " kHz";
  }
  sdCurrent += " • Target: ";
  if (sdPreferredMode == 0) sdCurrent += "Auto";
  else sdCurrent += String((unsigned int)sdPreferredMode) + "-bit";
  sdCurrent += " @ " + String((unsigned long)sdBaseFreqKHz) + " kHz";

  String hallDiagAttrs;
  if (hallDiagEnabled) hallDiagAttrs += " checked";
  if (playing) hallDiagAttrs += " disabled";
  String hallDiagHelp = playing
      ? String("Stop playback to enable the hall sensor blink test.")
      : String("Blinks all arms red whenever the hall sensor toggles.");

  String watchdogAttrs;
  if (watchdogEnabled) watchdogAttrs += " checked";
  String watchdogHelp = String("Automatically reboots if the controller stops responding.");

  const char *pauseLabel = paused ? "Resume" : "Pause";
  String pauseAttrs;
  if (!playing) pauseAttrs += " disabled";
  String autoplayAttrs;
  if (autoplayEnabled) autoplayAttrs += " checked";

  String html =
      "<!doctype html><html><head><meta charset='utf-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>POV Spinner</title>"
      "<style>"
      "body{font:16px system-ui,Segoe UI,Roboto,Arial,sans-serif;background:#0b1320;color:#e8ecf1;margin:0;padding:1rem}"
      ".card{max-width:960px;margin:0 auto;background:#121b2d;padding:1rem;border-radius:12px;box-shadow:0 1px 8px rgba(0,0,0,.2)}"
      "a{color:#a7c3ff;text-decoration:none}a:hover{text-decoration:underline}"
      "label{display:block;margin:.5rem 0 .2rem}select,input[type=range],input[type=number]{width:100%}"
      ".row{display:flex;gap:.5rem;flex-wrap:wrap;margin-top:.5rem}"
      "button{padding:.6rem 1rem;border:0;border-radius:10px;background:#1c2b4a;color:#e8ecf1;cursor:pointer}"
      "button:hover{filter:brightness(1.1)}"
      "input[type=file],input[type=number]{padding:.5rem;border-radius:10px;border:1px solid #253756;background:#0e1627;color:#e8ecf1}"
      ".muted{opacity:.75}"
      ".pill{display:inline-block;padding:.2rem .6rem;border-radius:999px;background:#0e1627;margin-left:.5rem}"
      ".sep{height:1px;background:#1b2741;margin:1rem 0}"
      ".badge{display:inline-block;margin-left:.5rem;padding:.15rem .55rem;border-radius:999px;font-size:.85rem}"
      ".badge.play{background:#0e2a19;color:#9af0b7}"
      ".badge.pause{background:#2a1f0e;color:#f0d49a}"
      ".badge.stop{background:#2a0e12;color:#f09aa6}"
      "</style></head><body>"
      "<div class='card'>"
      "<div style='display:flex;justify-content:space-between;align-items:center'>"
      "<h1 style='display:flex;align-items:center;gap:.4rem;margin:0'>"
      "POV Spinner"
      "<span id='status' class='" + statusClass + "'>" + statusText + "</span>"
      "<span id='which' class='pill'>" + currentFileEscaped + "</span>"
      "</h1>"
      "<a href='/files?path=/'>Files</a>"
      "</div>"
      "<p class='muted'>AP SSID: <b>" + apSsid + "</b> &middot; AP IP: <b>" + apIp + "</b> &middot; Wi-Fi IP: <b>" + htmlEscape(staIp) + "</b> &middot; mDNS: <b>" + mdnsName + "</b></p>"
      "<label>Choose .fseq file</label>"
      "<select id='sel'>" + optionsHtml + "</select>"
      "<div class='row'>"
      "<button id='start'>Start</button>"
      "<button id='pause'" + pauseAttrs + ">" + pauseLabel + "</button>"
      "<button id='stop'>Stop</button>"
      "<button id='refresh'>Refresh</button>"
      "</div>"
      "<div class='sep'></div>"
      "<h3>Wi-Fi Station</h3>"
      "<p class='muted'>Status: <b>" + htmlEscape(staStatus) + "</b> &middot; IP: <b>" + htmlEscape(staIp) + "</b></p>"
      "<label>Station SSID</label><input id='wssid' type='text' value='" + htmlEscape(staSsid) + "'>"
      "<label>Station Password</label><input id='wpass' type='password' placeholder='Leave blank to keep current'>"
      "<label>Station ID / Hostname</label><input id='wstation' type='text' value='" + htmlEscape(stationId) + "'>"
      "<div class='row'><button id='applywifi'>Save Wi-Fi</button><button id='wforget'>Forget Wi-Fi</button></div>"
      "<p class='muted'>Password is optional; leave blank to keep the stored value.</p>"
      "<div class='sep'></div>"
      "<h3>Spinner Layout</h3>"
      "<div class='row' style='gap:1rem;flex-wrap:wrap'>"
      "<div><label>Start Channel (Arm 1)</label><input id='startch' type='number' min='1' value='" + String(startChannel) + "'></div>"
      "<div><label>Total Spokes</label><input id='spokes' type='number' min='1' value='" + String(spokes) + "'></div>"
      "<div><label>Arm Count</label><input id='arms' type='number' min='1' max='" + String((int)maxArms) + "' value='" + String((int)arms) + "'></div>"
      "<div><label>Pixels per Arm</label><input id='pixels' type='number' min='1' max='" + String((int)maxPixelsPerArm) + "' value='" + String(pixelsPerArm) + "'></div>"
      "<div><label>Stride</label><select id='stride'><option value='spoke' " + String(spokeSel) + ">SPOKE</option><option value='led' " + String(ledSel) + ">LED</option></select></div>"
      "<div style='align-self:end'><button id='applymap'>Apply Layout</button></div>"
      "</div>"
      "<div class='sep'></div>"
      "<h3>Playback Speed</h3>"
      "<label>FPS: <span id='fpsv'>" + String(fps) + "</span></label>"
      "<input id='fps' type='range' min='1' max='120' value='" + String(fps) + "'>"
      "<div class='row'><button id='applyfps'>Apply</button><button id='fps10'>10 FPS</button><button id='fps40'>40 FPS</button><button id='fps60'>60 FPS</button></div>"
      "<div class='sep'></div>"
      "<h3>Brightness</h3>"
      "<label>Value: <span id='v'>" + String(brightnessPercent) + "%</span></label>"
      "<input id='rng' type='range' min='0' max='100' value='" + String(brightnessPercent) + "'>"
      "<div class='row'><button id='set'>Apply</button><button id='low'>10%</button><button id='med'>40%</button><button id='hi'>100%</button></div>"
      "<div class='sep'></div>"
      "<h3>SD Card</h3>"
      "<div class='row' style='gap:1rem;flex-wrap:wrap'>"
      "<div style='min-width:140px'><label>Bus Mode</label><select id='sdmode'>"
      "<option value='0' " + String(sdAutoSel) + ">Auto (try 4-bit then 1-bit)</option>"
      "<option value='4' " + String(sd4Sel) + ">Force 4-bit</option>"
      "<option value='1' " + String(sd1Sel) + ">Force 1-bit</option>"
      "</select></div>"
      "<div style='min-width:140px'><label>Clock Frequency</label><select id='sdfreq'>"
      "<option value='8000' " + String(freq8Sel) + ">8 MHz</option>"
      "<option value='4000' " + String(freq4Sel) + ">4 MHz</option>"
      "<option value='2000' " + String(freq2Sel) + ">2 MHz</option>"
      "<option value='1000' " + String(freq1Sel) + ">1 MHz</option>"
      "<option value='400' " + String(freq0Sel) + ">400 kHz</option>"
      "</select></div>"
      "<div style='align-self:end'><button id='applysd'>Apply SD Settings</button></div>"
      "<div style='align-self:end'><button id='sdrefresh'>Refresh SD Status</button></div>"
      "</div>"
      "<div id='sdinfo' class='muted' style='margin-top:.4rem'>" + sdCurrent + "</div>"
      "<div class='sep'></div>"
      "<h3>Diagnostics</h3>"
      "<div class='row'>"
      "<button id='hdr'>FSEQ Header</button>"
      "<button id='cblocks'>Compression Blocks</button>"
      "<button id='sdre'>SD Reinit</button>"
      "<button id='stat'>Status JSON</button>"
      "</div>"
      "<div style='margin-top:.75rem'>"
      "<label style='display:flex;align-items:center;gap:.5rem'>"
      "<input type='checkbox' id='halldiag'" + hallDiagAttrs + "> Hall Sensor Blink Test"
      "</label>"
      "<div class='muted'>" + hallDiagHelp + "</div>"
      "</div>"
      "<div style='margin-top:.75rem'>"
      "<label style='display:flex;align-items:center;gap:.5rem'>"
      "<input type='checkbox' id='watchdog'" + watchdogAttrs + "> Enable watchdog auto-reboot"
      "</label>"
      "<div class='muted'>" + watchdogHelp + "</div>"
      "</div>"
      "<div class='sep'></div>"
      "<h3>Auto-Play</h3>"
      "<label style='display:flex;align-items:center;gap:.5rem'>"
      "<input type='checkbox' id='autoplay'" + autoplayAttrs + "> Enable fallback auto-play"
      "</label>"
      "<p class='muted'>When enabled, <b>/test2.fseq</b> will start automatically after 5 minutes of inactivity.</p>"
      "</div>"
      "<script>"
      "const fps=document.getElementById('fps'), fpsv=document.getElementById('fpsv');"
      "fps.oninput=()=>fpsv.textContent=fps.value;"
      "const r=document.getElementById('rng'),v=document.getElementById('v');"
      "r.oninput=()=>v.textContent=r.value+'%';"
      "function post(u){fetch(u,{method:'POST'}).then(()=>location.reload());}"
      "document.getElementById('applyfps').onclick=()=>post('/speed?fps='+fps.value);"
      "document.getElementById('fps10').onclick=()=>post('/speed?fps=10');"
      "document.getElementById('fps40').onclick=()=>post('/speed?fps=40');"
      "document.getElementById('fps60').onclick=()=>post('/speed?fps=60');"
      "document.getElementById('set').onclick=()=>post('/b?value='+r.value);"
      "document.getElementById('low').onclick=()=>post('/b?value=10');"
      "document.getElementById('med').onclick=()=>post('/b?value=40');"
      "document.getElementById('hi').onclick=()=>post('/b?value=100');"
      "document.getElementById('start').onclick=()=>{const p=document.getElementById('sel').value;fetch('/start?path='+encodeURIComponent(p)).then(()=>location.reload());};"
      "const pause=document.getElementById('pause');"
      "if(pause){pause.onclick=()=>{fetch('/pause?toggle=1',{method:'POST'}).then(()=>location.reload()).catch(()=>location.reload());};}"
      "document.getElementById('stop').onclick =()=>post('/stop');"
      "document.getElementById('refresh').onclick=()=>location.reload();"
      "document.getElementById('applymap').onclick=()=>{"
      "const sc=+document.getElementById('startch').value||1;"
      "const sp=+document.getElementById('spokes').value||40;"
      "const ar=+document.getElementById('arms').value||1;"
      "const px=+document.getElementById('pixels').value||1;"
      "const st=(document.getElementById('stride').value)||'spoke';"
      "fetch('/mapcfg?start='+sc+'&spokes='+sp+'&arms='+ar+'&pixels='+px+'&stride='+st,{method:'POST'})"
      ".then(()=>location.reload());"
      "};"
      "document.getElementById('applywifi').onclick=()=>{"
      "const ss=document.getElementById('wssid').value;"
      "const pw=document.getElementById('wpass').value;"
      "const hn=document.getElementById('wstation').value;"
      "let url='/wifi?ssid='+encodeURIComponent(ss)+'&station='+encodeURIComponent(hn);"
      "if(pw.length) url+='&pass='+encodeURIComponent(pw);"
      "fetch(url,{method:'POST'}).then(()=>location.reload());"
      "};"
      "document.getElementById('wforget').onclick=()=>{"
      "fetch('/wifi?forget=1',{method:'POST'}).then(()=>location.reload());"
      "};"
      "document.getElementById('hdr').onclick=()=>fetch('/fseq/header').then(r=>r.json()).then(j=>alert(JSON.stringify(j,null,2)));"
      "document.getElementById('cblocks').onclick=()=>fetch('/fseq/cblocks').then(r=>r.json()).then(j=>alert(JSON.stringify(j,null,2)));"
      "document.getElementById('sdre').onclick=()=>fetch('/sd/reinit',{method:'POST'}).then(r=>r.text()).then(t=>alert(t));"
      "document.getElementById('stat').onclick=()=>fetch('/status').then(r=>r.json()).then(j=>alert(JSON.stringify(j,null,2)));"
      "const halldiag=document.getElementById('halldiag');"
      "if(halldiag){halldiag.onchange=()=>{const en=halldiag.checked?'1':'0';fetch('/halldiag?enable='+en,{method:'POST'}).then(()=>location.reload());};}"
      "const watchdog=document.getElementById('watchdog');"
      "if(watchdog){watchdog.onchange=()=>{const en=watchdog.checked?'1':'0';fetch('/watchdog?enable='+en,{method:'POST'}).catch(()=>{watchdog.checked=!watchdog.checked;});};}"
      "const autoplay=document.getElementById('autoplay');"
      "if(autoplay){autoplay.onchange=()=>{const en=autoplay.checked?'1':'0';fetch('/autoplay?enable='+en,{method:'POST'}).catch(()=>{autoplay.checked=!autoplay.checked;});};}"
      "const sdinfo=document.getElementById('sdinfo');"
      "function formatSd(j){if(!j||!j.sd) return 'Unavailable';const d=j.sd;let cur=d.ready?(d.currentWidth?d.currentWidth+'-bit':'Unknown width')+' @ '+d.freq+' kHz':'Card not mounted';const tgt=(d.desiredMode?d.desiredMode+'-bit':'Auto')+' @ '+d.baseFreq+' kHz';return 'Current: '+cur+' • Target: '+tgt;}"
      "function updateSd(){fetch('/status').then(r=>r.json()).then(j=>{if(sdinfo) sdinfo.textContent=formatSd(j);}).catch(()=>{if(sdinfo) sdinfo.textContent='Status unavailable';});}"
      "updateSd();"
      "document.getElementById('sdrefresh').onclick=()=>updateSd();"
      "document.getElementById('applysd').onclick=()=>{"
      "const mode=document.getElementById('sdmode').value;"
      "const freq=document.getElementById('sdfreq').value;"
      "fetch('/sd/config?mode='+mode+'&freq='+freq,{method:'POST'}).then(r=>r.json()).then(j=>{"
      "if(sdinfo){if(j.ok){sdinfo.textContent=formatSd({sd:j});}else if(j.error){sdinfo.textContent='Error: '+j.error;}else{sdinfo.textContent='Error applying SD settings';}}"
      "}).catch(()=>{if(sdinfo) sdinfo.textContent='Error applying SD settings';});"
      "};"
      "</script>"
      "</body></html>";
  return html;
}

}  // namespace WebPages


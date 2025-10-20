#include "WebPages.h"

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
                const String &staPass,
                uint32_t startChannel,
                uint16_t spokes,
                uint8_t arms,
                uint16_t pixelsPerArm,
                uint8_t maxArms,
                uint16_t maxPixelsPerArm,
                bool strideIsSpoke,
                uint16_t fps,
                uint8_t brightnessPercent) {
  const char *spokeSel = strideIsSpoke ? "selected" : "";
  const char *ledSel = strideIsSpoke ? "" : "selected";
  String wifiStatus = "Status: <b>" + staStatus + "</b>";
  if (staIp.length()) wifiStatus += " &middot; IP: <b>" + staIp + "</b>";

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
      "<p class='muted'>AP SSID: <b>" + apSsid + "</b> &middot; IP: <b>" + apIp + "</b> &middot; mDNS: <b>" + mdnsName + "</b></p>"
      "<label>Choose .fseq file</label>"
      "<select id='sel'>" + optionsHtml + "</select>"
      "<div class='row'>"
      "<button id='start'>Start</button>"
      "<button id='stop'>Stop</button>"
      "<button id='refresh'>Refresh</button>"
      "</div>"
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
      "<h3>Upload a new .fseq to SD (root)</h3>"
      "<form id='u' method='POST' action='/upload' enctype='multipart/form-data'>"
      "<input type='file' name='f' accept='.fseq' required>"
      "<div class='row'><button type='submit'>Upload to /</button></div>"
      "<p class='muted'>Uploads here go to the SD card root. Use the Files page to upload into a specific folder.</p>"
      "</form>"
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
      "<h3>Wi-Fi Station</h3>"
      "<p class='muted'>" + wifiStatus + "</p>"
      "<label>Station SSID</label><input id='sta-ssid' type='text' value='" + staSsid + "'>"
      "<label>Station Password</label><input id='sta-pass' type='password' value='" + staPass + "'>"
      "<label>Station ID</label><input id='sta-id' type='text' value='" + stationId + "'>"
      "<div class='row'><button id='savewifi'>Save Wi-Fi</button></div>"
      "<div class='sep'></div>"
      "<h3>Firmware Update</h3>"
      "<p class='muted'>Provide the SD card path to a firmware .bin file and the spinner will update and reboot.</p>"
      "<div class='row'><input id='ota-path' type='text' placeholder='/firmware.bin' style='flex:1' value=''>"
      "<button id='applyota'>Apply Update</button></div>"
      "<div class='sep'></div>"
      "<h3>Diagnostics</h3>"
      "<div class='row'>"
      "<button id='hdr'>FSEQ Header</button>"
      "<button id='cblocks'>Compression Blocks</button>"
      "<button id='sdre'>SD Reinit</button>"
      "<button id='stat'>Status JSON</button>"
      "</div>"
      "<div class='sep'></div>"
      "<p class='muted'>If no file is started within 5 minutes after boot, <b>/test2.fseq</b> will auto-play.</p>"
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
      "document.getElementById('savewifi').onclick=()=>{"
      "const body='ssid='+encodeURIComponent(document.getElementById('sta-ssid').value)"
      "+'&pass='+encodeURIComponent(document.getElementById('sta-pass').value)"
      "+'&id='+encodeURIComponent(document.getElementById('sta-id').value);"
      "fetch('/wifi',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body}).then(r=>{if(r.ok){alert('Wi-Fi settings saved. Reconnect may take a few seconds.');}else{r.text().then(t=>alert('Wi-Fi save failed: '+t));}});"
      "};"
      "document.getElementById('applyota').onclick=()=>{"
      "const path=document.getElementById('ota-path').value||'/firmware.bin';"
      "fetch('/ota?path='+encodeURIComponent(path),{method:'POST'}).then(r=>{if(r.ok){alert('Update started. Device will reboot when complete.');}else{r.text().then(t=>alert('OTA failed: '+t));}});"
      "};"
      "document.getElementById('hdr').onclick=()=>fetch('/fseq/header').then(r=>r.json()).then(j=>alert(JSON.stringify(j,null,2)));"
      "document.getElementById('cblocks').onclick=()=>fetch('/fseq/cblocks').then(r=>r.json()).then(j=>alert(JSON.stringify(j,null,2)));"
      "document.getElementById('sdre').onclick=()=>fetch('/sd/reinit',{method:'POST'}).then(r=>r.text()).then(t=>alert(t));"
      "document.getElementById('stat').onclick=()=>fetch('/status').then(r=>r.json()).then(j=>alert(JSON.stringify(j,null,2)));"
      "</script>"
      "</body></html>";
  return html;
}

}  // namespace WebPages


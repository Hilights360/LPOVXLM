#include "WebPages.h"

namespace {
String strideOptionSelected(uint8_t strideMode, uint8_t option) {
  return (strideMode == option) ? String("selected") : String();
}
}

String buildControlPage(const ControlPageContext &ctx) {
  String html;
  html.reserve(4096);
  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>POV Spinner</title>";
  html += "<style>";
  html += "body{font:16px system-ui,Segoe UI,Roboto,Arial,sans-serif;background:#0b1320;color:#e8ecf1;margin:0;padding:1rem}";
  html += ".card{max-width:960px;margin:0 auto;background:#121b2d;padding:1rem;border-radius:12px;box-shadow:0 1px 8px rgba(0,0,0,.2)}";
  html += "a{color:#a7c3ff;text-decoration:none}a:hover{text-decoration:underline}";
  html += "label{display:block;margin:.5rem 0 .2rem}select,input[type=range],input[type=number]{width:100%}";
  html += ".row{display:flex;gap:.5rem;flex-wrap:wrap;margin-top:.5rem}";
  html += "button{padding:.6rem 1rem;border:0;border-radius:10px;background:#1c2b4a;color:#e8ecf1;cursor:pointer}";
  html += "button:hover{filter:brightness(1.1)}";
  html += "input[type=file],input[type=number]{padding:.5rem;border-radius:10px;border:1px solid #253756;background:#0e1627;color:#e8ecf1}";
  html += ".muted{opacity:.75}";
  html += ".pill{display:inline-block;padding:.2rem .6rem;border-radius:999px;background:#0e1627;margin-left:.5rem}";
  html += ".sep{height:1px;background:#1b2741;margin:1rem 0}";
  html += ".badge{display:inline-block;margin-left:.5rem;padding:.15rem .55rem;border-radius:999px;font-size:.85rem}";
  html += ".badge.play{background:#0e2a19;color:#9af0b7}";
  html += ".badge.pause{background:#2a1f0e;color:#f0d49a}";
  html += ".badge.stop{background:#2a0e12;color:#f09aa6}";
  html += "</style></head><body>";
  html += "<div class='card'>";
  html += "<div style='display:flex;justify-content:space-between;align-items:center'>";
  html += "<h1 style='display:flex;align-items:center;gap:.4rem;margin:0'>";
  html += "POV Spinner";
  html += "<span id='status' class='" + htmlEscape(ctx.statusClass) + "'>" + htmlEscape(ctx.statusText) + "</span>";
  html += "<span id='which' class='pill'>" + htmlEscape(ctx.currentPath) + "</span>";
  html += "</h1>";
  html += "<a href='/files?path=/'>Files</a>";
  html += "</div>";
  html += "<p class='muted'>AP SSID: <b>POV-Spinner</b> &middot; IP: <b>192.168.4.1</b> &middot; mDNS: <b>pov.local</b></p>";
  html += "<label>Choose .fseq file</label>";
  html += "<select id='sel'>" + ctx.optionsHtml + "</select>";
  html += "<div class='row'>";
  html += "<button id='start'>Start</button>";
  html += "<button id='stop'>Stop</button>";
  html += "<button id='refresh'>Refresh</button>";
  html += "</div>";
  html += "<div class='sep'></div>";
  html += "<h3>Spinner Layout</h3>";
  html += "<div class='row' style='gap:1rem;flex-wrap:wrap'>";
  html += "<div><label>Start Channel (Arm 1)</label><input id='startch' type='number' min='1' value='" + String(ctx.startChannel) + "'></div>";
  html += "<div><label>Total Spokes</label><input id='spokes' type='number' min='1' value='" + String(ctx.totalSpokes) + "'></div>";
  html += "<div><label>Arm Count</label><input id='arms' type='number' min='1' max='" + String(ctx.maxArms) + "' value='" + String(ctx.armCount) + "'></div>";
  html += "<div><label>Pixels per Arm</label><input id='pixels' type='number' min='1' max='" + String(ctx.maxPixelsPerArm) + "' value='" + String(ctx.pixelsPerArm) + "'></div>";
  html += "<div><label>Stride</label><select id='stride'><option value='spoke' " + strideOptionSelected(ctx.strideMode, 0) + ">SPOKE</option><option value='led' " + strideOptionSelected(ctx.strideMode, 1) + ">LED</option></select></div>";
  html += "<div style='align-self:end'><button id='applymap'>Apply Layout</button></div>";
  html += "</div>";
  html += "<div class='sep'></div>";
  html += "<h3>Upload a new .fseq to SD (root)</h3>";
  html += "<form id='u' method='POST' action='/upload' enctype='multipart/form-data'>";
  html += "<input type='file' name='f' accept='.fseq' required>";
  html += "<div class='row'><button type='submit'>Upload to /</button></div>";
  html += "<p class='muted'>Uploads here go to the SD card root. Use the Files page to upload into a specific folder.</p>";
  html += "</form>";
  html += "<div class='sep'></div>";
  html += "<h3>Playback Speed</h3>";
  html += "<label>FPS: <span id='fpsv'>" + String(ctx.fps) + "</span></label>";
  html += "<input id='fps' type='range' min='1' max='120' value='" + String(ctx.fps) + "'>";
  html += "<div class='row'><button id='applyfps'>Apply</button><button id='fps10'>10 FPS</button><button id='fps40'>40 FPS</button><button id='fps60'>60 FPS</button></div>";
  html += "<div class='sep'></div>";
  html += "<h3>Brightness</h3>";
  html += "<label>Value: <span id='v'>" + String(ctx.brightnessPercent) + "%</span></label>";
  html += "<input id='rng' type='range' min='0' max='100' value='" + String(ctx.brightnessPercent) + "'>";
  html += "<div class='row'><button id='set'>Apply</button><button id='low'>10%</button><button id='med'>40%</button><button id='hi'>100%</button></div>";
  html += "<div class='sep'></div>";
  html += "<h3>Diagnostics</h3>";
  html += "<div class='row'>";
  html += "<button id='hdr'>FSEQ Header</button>";
  html += "<button id='cblocks'>Compression Blocks</button>";
  html += "<button id='sdre'>SD Reinit</button>";
  html += "<button id='stat'>Status JSON</button>";
  html += "</div>";
  html += "<div class='sep'></div>";
  html += "<p class='muted'>If no file is started within 5 minutes after boot, <b>/test2.fseq</b> will auto-play.</p>";
  html += "</div>";
  html += "<script>";
  html += "const fps=document.getElementById('fps'), fpsv=document.getElementById('fpsv');";
  html += "fps.oninput=()=>fpsv.textContent=fps.value;";
  html += "const r=document.getElementById('rng'),v=document.getElementById('v');";
  html += "r.oninput=()=>v.textContent=r.value+'%';";
  html += "function post(u){fetch(u,{method:'POST'}).then(()=>location.reload());}";
  html += "document.getElementById('applyfps').onclick=()=>post('/speed?fps='+fps.value);";
  html += "document.getElementById('fps10').onclick=()=>post('/speed?fps=10');";
  html += "document.getElementById('fps40').onclick=()=>post('/speed?fps=40');";
  html += "document.getElementById('fps60').onclick=()=>post('/speed?fps=60');";
  html += "document.getElementById('set').onclick=()=>post('/b?value='+r.value);";
  html += "document.getElementById('low').onclick=()=>post('/b?value=10');";
  html += "document.getElementById('med').onclick=()=>post('/b?value=40');";
  html += "document.getElementById('hi').onclick=()=>post('/b?value=100');";
  html += "document.getElementById('start').onclick=()=>{const p=document.getElementById('sel').value;fetch('/start?path='+encodeURIComponent(p)).then(()=>location.reload());};";
  html += "document.getElementById('stop').onclick =()=>post('/stop');";
  html += "document.getElementById('refresh').onclick=()=>location.reload();";
  html += "document.getElementById('applymap').onclick=()=>{";
  html += "const sc=+document.getElementById('startch').value||1;";
  html += "const sp=+document.getElementById('spokes').value||40;";
  html += "const ar=+document.getElementById('arms').value||1;";
  html += "const px=+document.getElementById('pixels').value||1;";
  html += "const st=(document.getElementById('stride').value)||'spoke';";
  html += "fetch('/mapcfg?start='+sc+'&spokes='+sp+'&arms='+ar+'&pixels='+px+'&stride='+st,{method:'POST'}).then(()=>location.reload());";
  html += "};";
  html += "document.getElementById('hdr').onclick=()=>fetch('/fseq/header').then(r=>r.json()).then(j=>alert(JSON.stringify(j,null,2)));";
  html += "document.getElementById('cblocks').onclick=()=>fetch('/fseq/cblocks').then(r=>r.json()).then(j=>alert(JSON.stringify(j,null,2)));";
  html += "document.getElementById('sdre').onclick=()=>fetch('/sd/reinit',{method:'POST'}).then(r=>r.text()).then(t=>alert(t));";
  html += "document.getElementById('stat').onclick=()=>fetch('/status').then(r=>r.json()).then(j=>alert(JSON.stringify(j,null,2)));";
  html += "</script>";
  html += "</body></html>";
  return html;
}

String buildFilesPage(const FilesPageContext &ctx) {
  const std::vector<FilesPageEntry> empty;
  const std::vector<FilesPageEntry> &entries = ctx.entries ? *ctx.entries : empty;
  const String escapedPath = htmlEscape(ctx.path);
  const String parentLink = urlEncode(ctx.parentPath);
  const String encodedPath = urlEncode(ctx.path);
  const String backTarget = urlEncode(String("/files?path=") + ctx.path);

  String html;
  html.reserve(4096);
  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Files - " + escapedPath + "</title>";
  html += "<style>";
  html += "body{font:16px system-ui,Segoe UI,Roboto,Arial;background:#0b1320;color:#e8ecf1;margin:0;padding:1rem}";
  html += ".card{max-width:860px;margin:0 auto;background:#121b2d;padding:1rem;border-radius:12px}";
  html += "a{color:#a7c3ff;text-decoration:none}a:hover{text-decoration:underline}";
  html += "table{width:100%;border-collapse:collapse;margin-top:.5rem}";
  html += "th,td{padding:.5rem;border-bottom:1px solid #1b2741}";
  html += ".row{display:flex;gap:.5rem;flex-wrap:wrap;margin-top:.5rem}";
  html += "button{padding:.4rem .7rem;border:0;border-radius:8px;background:#1c2b4a;color:#e8ecf1;cursor:pointer}";
  html += "button:hover{filter:brightness(1.1)}";
  html += "input{padding:.45rem .5rem;border-radius:8px;border:1px solid #253756;background:#0e1627;color:#e8ecf1}";
  html += ".muted{opacity:.75}";
  html += "</style></head><body><div class='card'>";
  html += "<div style='display:flex;justify-content:space-between;align-items:center'>";
  html += "<h2 style='margin:0'>Files</h2>";
  html += "<a href='/'>Back to Control</a>";
  html += "</div>";
  html += "<p>Path: <b>" + escapedPath + "</b> &middot; <a href='/files?path=" + parentLink + "'>Up</a></p>";
  html += "<form method='POST' action='/upload?dir=" + encodedPath + "&back=" + backTarget + "' enctype='multipart/form-data'>";
  html += "<input type='file' name='f' accept='.fseq' required> ";
  html += "<button type='submit'>Upload here</button>";
  html += "<div class='muted' style='margin-top:.25rem'>Only <b>.fseq</b> files are accepted.</div>";
  html += "</form>";
  html += "<div class='row' style='margin-top:.75rem'>";
  html += "<button onclick=\"const n=prompt('New folder name'); if(n) location='/mkdir?path=" + encodedPath + "&name='+encodeURIComponent(n);\">New Folder</button>";
  html += "<button onclick=\"location.reload()\">Refresh</button>";
  html += "</div>";
  html += "<table><thead><tr><th>Name</th><th>Size</th><th>Actions</th></tr></thead><tbody>";

  const String backParam = urlEncode(ctx.path);
  for (const auto &entry : entries) {
    const String &name = entry.name;
    const String esc = htmlEscape(name);
    const String enc = urlEncode(name);
    const String escBase = htmlEscape(baseName(name));
    if (entry.isDirectory) {
      html += "<tr><td>📁 <a href='/files?path=" + enc + "'>" + esc + "</a></td>";
      html += "<td>—</td>";
      html += "<td>";
      html += "<a href='#' onclick=\"if(confirm('Delete folder " + esc + "? (must be empty)')) location='/rm?path=" + enc + "&back=/files?path=" + backParam + "'; return false;\">🗑️ Delete</a> &nbsp; ";
      html += "<a href='#' onclick=\"const n=prompt('Rename folder to:', '" + escBase + "'); if(n) location='/ren?path=" + enc + "&to='+encodeURIComponent(n)+'&back=/files?path=" + backParam + "'; return false;\">✏️ Rename</a>";
      html += "</td></tr>";
    } else {
      html += "<tr><td>📄 " + esc + "</td>";
      html += "<td>" + String(static_cast<unsigned long>(entry.size)) + "</td>";
      html += "<td>";
      html += "<a href='/dl?path=" + enc + "'>⬇️ Download</a> &nbsp; ";
      html += "<a href='/play?path=" + enc + "&back=/files?path=" + backParam + "'>▶️ Play</a> &nbsp; ";
      html += "<a href='#' onclick=\"if(confirm('Delete " + esc + "?')) location='/rm?path=" + enc + "&back=/files?path=" + backParam + "'; return false;\">🗑️ Delete</a> &nbsp; ";
      html += "<a href='#' onclick=\"const n=prompt('Rename file to:', '" + escBase + "'); if(n) location='/ren?path=" + enc + "&to='+encodeURIComponent(n)+'&back=/files?path=" + backParam + "'; return false;\">✏️ Rename</a>";
      html += "</td></tr>";
    }
  }
  html += "</tbody></table></div></body></html>";
  return html;
}


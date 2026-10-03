// npm install --prefix build/ui-test --no-audit --no-fund playwright@1.51.1
// node tests/web_smoke.cjs (uses the installed Microsoft Edge browser)
const assert = require('node:assert/strict');
const fs = require('node:fs');
const http = require('node:http');
const {chromium} = require('../build/ui-test/node_modules/playwright');
const pageHtml = fs.readFileSync('main/web/index.html');
const sdHtml = fs.readFileSync('main/web/sd.html');
const recentScript = fs.readFileSync('main/web/recent.js');
let filesFail = false, startFails = false;
let mountFails = false;
let mountAsync = false;
let loopSaveFails = false;
let autoDutySaveFails = false;
let fseqSaveFails = false, fileDefaults = null;
let radialSaveFails = false;
const radialFiles = new Map();
const requests = [];
function fileSettings(){
  const enabled=!!state.settings.autoFseq,f=fileDefaults;
  return {loaded:!!f,path:f?.path||'',stepTimeMs:f?.step||0,channelCount:f?.channels||0,
    fpsFromFile:enabled&&!!f?.step,spokesFromFile:enabled&&!!f?.spokes,
    effectiveFps:enabled&&f?.step?1000/f.step:state.settings.fps,
    effectiveSpokes:enabled&&f?.spokes?f.spokes:state.settings.spokes,
    timingNote:'Missing frame interval; using manual FPS.',layoutNote:'Ambiguous layout; using manual spokes.'};
}
const dma = {running:false,state:'idle',phase:'gpio',framesCompleted:0,framesTarget:10,error:'',
  gpio:{meanTotal_us:1600},dma:{meanTotal_us:360,meanPack_us:40,meanSubmitToDone_us:310},speedup:1600/360};
const state = {
  sdTools:{running:false,state:'idle',operation:'speed',phase:'writing',error:'',temporaryFile:'',bytesTarget:0,bytesWritten:0,bytesRead:0,writeMiBps:0,readMiBps:0,maxWrite_ms:0,maxRead_ms:0,busWidth:1,clockKHz:20000,elapsed_ms:0,verified:false},
  dmaTest:dma,
  firmware:'native-esp-idf',mode:0,paused:false,path:'',error:'',frame:0,rpm:0,pulseCount:0,
  psramBytes:8388608,freePsramBytes:7000000,
  sd:{ready:true,currentWidth:4,freq:20000,desiredMode:4,baseFreq:20000,fallback:true,recoverErrors:true,recovery:{running:false,state:'idle'}},
  firmwareBuild:'Sep 17 2026 12:00:00',
  firmwareVersion:'1.2',firmwareBuildNumber:42,
  wifi:{ssid:'Workshop',station:'pov-test',ip:'192.168.1.20',mode:'AP+STA',channel:6,
    apClients:1,apDisconnects:2,lastApDisconnectReason:4,uptime_ms:65000,
    retriesPaused:true,stationConnecting:false,brownoutReset:false},
  settings:{brightness:25,duty:60,fps:40,start:1,spokes:40,arms:4,pixels:144,usepa:false,
    autoplay:true,loop:true,watchdog:false,background:false,backgroundPath:'',strobe:false,strobeWidth:3,
    phase:0,starts:[1,433,865,1297],armPhase:[0,0,0,0]}
};
const fileEntries = [
    {name:'file10.fseq',path:'/file10.fseq',directory:false,size:12,modified:1720000000},
    {name:'Z-folder',path:'/Z-folder',directory:true,size:0,modified:1710000000},
    {name:'demo.fseq',path:'/demo.fseq',directory:false,size:2048,modified:1700000000},
    {name:'File2.fseq',path:'/File2.fseq',directory:false,size:12,modified:1710000000},
    {name:'a-folder',path:'/a-folder',directory:true,size:0,modified:null},
    {name:'<img src=x onerror=alert(1)>.fseq',path:'/literal-name.fseq',directory:false,size:9,modified:null}
  ];
const server = http.createServer(async(req,res)=>{
  const url = new URL(req.url,'http://localhost');
  const chunks=[];for await(const chunk of req)chunks.push(chunk);
  const body=Buffer.concat(chunks);
  requests.push({path:url.pathname,query:url.searchParams,method:req.method,body,type:req.headers['content-type']});
  let result={ok:true};
  if(req.method==='POST'){
    const p=new URLSearchParams(body.toString());
    if(url.pathname==='/upload')fileEntries.push({name:url.searchParams.get('path').split('/').pop(),path:url.searchParams.get('path'),directory:false,size:body.length,modified:+url.searchParams.get('modified')});
    if(url.pathname==='/b')state.settings.brightness=+p.get('value');
    if(url.pathname==='/autoduty'){
      if(autoDutySaveFails){res.statusCode=500;result={error:'Settings write failed'};}
      else state.settings.autoDuty=p.get('enable')==='1';
    }
    if(url.pathname==='/fseq/settings'){
      if(fseqSaveFails){res.statusCode=500;result={error:'Settings write failed'};}
      else state.settings.autoFseq=p.get('enable')==='1';
    }
    if(url.pathname==='/fseq/radial'){
      if(radialSaveFails){res.statusCode=500;result={error:'Pixel order write failed'};}
      else radialFiles.set(p.get('path'),p.get('tipFirst')==='1');
    }
    if(url.pathname==='/loop'){
      if(loopSaveFails){res.statusCode=500;result={error:'Settings write failed'};}
      else state.settings.loop=p.get('enable')==='1';
    }
    if(url.pathname==='/mapcfg')for(const key of ['start','spokes','arms','pixels'])if(p.has(key))state.settings[key]=+p.get(key);
    if(url.pathname==='/diag/dma'){dma.running=true;dma.state='running';state.mode=5;result=dma;}
    if(url.pathname==='/colorfade'){dma.running=false;state.mode=6;}
    if(url.pathname==='/stop'){dma.running=false;dma.state='cancelled';state.mode=0;}
    if(url.pathname==='/ota'){res.statusCode=400;result={error:'Firmware image rejected'};}
    if(url.pathname==='/sd/config'){
      state.sd.desiredMode=+p.get('mode');state.sd.baseFreq=+p.get('freq');
      state.sd.fallback=p.get('fallback')==='1';state.sd.recoverErrors=p.get('recover')==='1';
      state.sd.currentWidth=state.sd.desiredMode||4;state.sd.freq=state.sd.baseFreq;
    }
    if(['/sd/config','/sd/reinit'].includes(url.pathname)){
      state.sd.ready=!mountFails;
      if(mountFails){res.statusCode=400;result={error:'SD mount failed'};}
      else if(mountAsync){state.sd.ready=false;Object.assign(state.sdTools,{running:true,state:'running',operation:'mount',phase:'mounting'});res.statusCode=202;result=state.sdTools;}
    }
    if(['/sd/format','/sd/speed'].includes(url.pathname)){
      if(url.pathname==='/sd/format'&&p.get('confirm')!=='ERASE SD CARD'){res.statusCode=400;result={error:'Explicit SD erase confirmation is required'};}
      else {
        Object.assign(state.sdTools,{running:true,state:'running',operation:url.pathname==='/sd/format'?'format':'speed',phase:'writing',bytesTarget:+p.get('mib')*1048576,bytesWritten:0,bytesRead:0,verified:false,error:''});
        if(state.sdTools.operation==='format')state.sd.ready=false;
        result=state.sdTools;
      }
    }
    if(url.pathname==='/sd/cancel'){state.sdTools.running=false;state.sdTools.state='cancelled';result=state.sdTools;}
    if(url.pathname==='/sd/recover'){
      state.sd.ready=false;Object.assign(state.sd.recovery,{running:true,state:'recovering',reason:'Manual lower-speed recovery',attempts:1,width:1,frequency:400,error:''});
      Object.assign(state.sdTools,{running:true,operation:'recovery',state:'running',phase:'recovering'});result=state.sd.recovery;
    }
  }else if(url.pathname==='/status')result={...state,fileSettings:fileSettings(),radialMapping:{path:state.path,tipFirst:!!radialFiles.get(state.path)}};
  else if(url.pathname==='/start'){if(startFails){res.statusCode=404;result={error:'Sequence not found'};}else{state.path=url.searchParams.get('path');state.mode=1;}}
  else if(url.pathname==='/api/files'){if(filesFail){res.statusCode=503;result={error:'SD is busy; retry shortly'};}else result={path:url.searchParams.get('path'),files:fileEntries};}
  else if(url.pathname==='/recent.js'){res.setHeader('Content-Type','application/javascript');res.end(recentScript);return;}
  else if(url.pathname==='/duty.js'){res.setHeader('Content-Type','application/javascript');res.end(fs.readFileSync('main/web/duty.js'));return;}
  else if(url.pathname==='/diag/spi')result={driver:'native-idf-shared-clock-gpio',lastTransmit_us:1200,bitsPerStrip:4744};
  else if(url.pathname==='/diag/wifi')result=state.wifi;
  else if(url.pathname==='/diag/crash')result={available:false,status:'ESP_ERR_NOT_FOUND'};
  else if(url.pathname==='/logs.txt'){res.end('Native controller ready\n');return;}
  else if(['/files','/diagnostics','/updates','/ota','/logs'].includes(url.pathname)){
    res.writeHead(302,{Location:'/sd#'+(url.pathname==='/ota'?'updates':url.pathname.slice(1))});res.end();return;
  }
  else {res.setHeader('Content-Type','text/html; charset=utf-8');res.end(url.pathname.startsWith('/sd')?sdHtml:pageHtml);return;}
  res.setHeader('Content-Type','application/json');res.end(JSON.stringify(result));
});

(async()=>{
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
  const browser=await chromium.launch({channel:'msedge',headless:true});
  const page=await browser.newPage({viewport:{width:1280,height:900}});
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  page.on('dialog',dialog=>dialog.accept());
  const latest=path=>requests.filter(r=>r.path===path&&r.method==='POST').at(-1);
  const origin=`http://127.0.0.1:${server.address().port}`;
  try{
    await page.goto(`http://127.0.0.1:${server.address().port}/`);
    await page.waitForFunction(()=>document.getElementById('headerbuild').textContent==='Ver1.2 \u00b7 Build 42');
    assert((await page.getByRole('heading',{level:1}).textContent()).startsWith('LPOVXLM Spinner'));
    await page.waitForFunction(()=>document.getElementById('brightness').value==='25');
    assert.equal(await page.locator('#psram').textContent(),'8 MB');
    assert(await page.locator('#sequenceTipFirst').isDisabled());
    state.settings.strobe=true;
    state.dutyControl={available:true,feasible:true,continuous:false,calculatedDuty:35,active:false};
    await page.locator('#autoDuty').check();
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Auto-calc enabled.'&&!document.getElementById('autoDuty').disabled);
    assert.equal(await page.locator('#duty').inputValue(),'35');
    assert(await page.locator('#duty').isDisabled());assert(await page.locator('#strobe').isDisabled());
    assert(await page.locator('#strobeWidth').isDisabled());
    assert(await page.locator('#phase').isEnabled()); // Alignment remains adjustable.
    assert.equal(state.settings.duty,60);assert.equal(state.settings.strobe,true);
    await page.reload();await page.waitForFunction(()=>document.getElementById('brightness').value==='25');
    assert(await page.locator('#autoDuty').isChecked());assert.equal(await page.locator('#duty').inputValue(),'35');
    autoDutySaveFails=true;await page.locator('#autoDuty').click();
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Settings write failed'&&!document.getElementById('autoDuty').disabled);
    assert(await page.locator('#autoDuty').isChecked());autoDutySaveFails=false;
    await page.locator('#autoDuty').uncheck();
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Auto-calc off. Manual timing restored.'&&!document.getElementById('autoDuty').disabled);
    assert.equal(await page.locator('#duty').inputValue(),'60');assert(await page.locator('#strobe').isEnabled());
    assert.equal(state.settings.strobe,true);state.settings.strobe=false;
    await page.locator('#autoFseq').check();
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='FSEQ settings enabled.'&&!document.getElementById('autoFseq').disabled);
    assert((await page.locator('#fseqsettingsinfo').textContent()).includes('Load a sequence'));
    assert(await page.locator('#fps').isEnabled());assert(await page.locator('#spokes').isEnabled());
    fileDefaults={path:'/Spooky_256_spoke_20FPS.fseq',step:50,spokes:256,channels:110592};
    await page.evaluate(()=>refresh());
    assert.equal(await page.locator('#fps').inputValue(),'20');assert.equal(await page.locator('#spokes').inputValue(),'256');
    assert(await page.locator('#fps').isDisabled());assert(await page.locator('#spokes').isDisabled());
    assert.equal(state.settings.fps,40);assert.equal(state.settings.spokes,40);
    await page.reload();await page.waitForFunction(()=>document.getElementById('fps').value==='20');
    assert(await page.locator('#autoFseq').isChecked());
    fileDefaults={path:'/fractional.fseq',step:33,spokes:80,channels:34560};
    await page.evaluate(()=>refresh());assert.equal(await page.locator('#fps').inputValue(),'30.303');
    assert.equal(await page.locator('#spokes').inputValue(),'80');
    await page.getByRole('button',{name:'Save mapping',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Settings saved.');
    assert.equal(new URLSearchParams(latest('/mapcfg').body.toString()).get('spokes'),null);
    assert.equal(await page.locator('#spokes').inputValue(),'80');
    fseqSaveFails=true;await page.locator('#autoFseq').click();
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Settings write failed'&&!document.getElementById('autoFseq').disabled);
    assert(await page.locator('#autoFseq').isChecked());fseqSaveFails=false;
    fileDefaults.spokes=0;await page.evaluate(()=>refresh());
    assert((await page.locator('#fseqsettingsinfo').textContent()).includes('Ambiguous layout'));
    assert(await page.locator('#spokes').isEnabled());assert(await page.locator('#fps').isDisabled());
    await page.locator('#spokes').fill('133');await page.evaluate(()=>refresh());
    assert.equal(await page.locator('#spokes').inputValue(),'133');
    await page.locator('#autoFseq').uncheck();
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Manual FPS and spokes restored.'&&!document.getElementById('autoFseq').disabled);
    assert.equal(await page.locator('#fps').inputValue(),'40');
    assert(await page.locator('#fps').isEnabled());fileDefaults=null;
    await page.waitForFunction(()=>document.getElementById('recentSequences').options.length===5);
    assert.equal(await page.locator('#sequence').inputValue(),'/file10.fseq');
    assert.deepEqual(await page.locator('#recentSequences optgroup').last().locator('option').evaluateAll(items=>items.map(o=>o.value)),['/file10.fseq','/File2.fseq','/demo.fseq','/literal-name.fseq']);
    const startsBefore=requests.filter(r=>r.path==='/start').length;
    await page.getByRole('combobox',{name:'Recent sequence files',exact:true}).selectOption('/demo.fseq');
    assert.equal(await page.locator('#sequence').inputValue(),'/demo.fseq');
    assert.equal(requests.filter(r=>r.path==='/start').length,startsBefore); // Picking is not Play.
    await page.reload();
    await page.waitForFunction(()=>document.getElementById('sequence').value==='/demo.fseq');
    assert.equal(await page.locator('#recentSequences optgroup').first().locator('option').first().getAttribute('value'),'/demo.fseq');
    const special='/Folder/space # &.fseq';
    await page.locator('#sequence').fill(special);
    await page.getByRole('button',{name:'Play',exact:true}).click();
    await page.waitForFunction(path=>document.getElementById('current').textContent.startsWith(path),special);
    assert.equal(requests.filter(r=>r.path==='/start').at(-1).query.get('path'),special);
    assert.equal(await page.locator('#recentSequences optgroup').first().locator('option').first().getAttribute('value'),special);
    await page.locator('#sequenceTipFirst').check();
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Pixel order saved for this sequence.'&&!document.getElementById('sequenceTipFirst').disabled);
    assert.equal(radialFiles.get(special),true);
    await page.reload();await page.waitForFunction(()=>document.getElementById('sequenceTipFirst').checked);
    state.path='/new-export.fseq';await page.evaluate(()=>refresh());
    assert(!(await page.locator('#sequenceTipFirst').isChecked()));
    state.path=special;await page.evaluate(()=>refresh());
    assert(await page.locator('#sequenceTipFirst').isChecked());
    radialSaveFails=true;await page.locator('#sequenceTipFirst').click();
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Pixel order write failed'&&!document.getElementById('sequenceTipFirst').disabled);
    assert(await page.locator('#sequenceTipFirst').isChecked());radialSaveFails=false;
    startFails=true;
    await page.locator('#sequence').fill('/missing.fseq');
    await page.getByRole('button',{name:'Play',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Sequence not found');
    assert(!await page.locator('#recentSequences option').evaluateAll(items=>items.some(o=>o.value==='/missing.fseq')));
    startFails=false;state.path='';state.mode=0;
    filesFail=true;
    await page.getByRole('button',{name:'Refresh file list',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('sequencehint').textContent==='SD is busy; retry shortly');
    assert(await page.getByRole('combobox',{name:'Recent sequence files',exact:true}).isEnabled());
    assert.equal(await page.locator('#sequence').inputValue(),'/missing.fseq');
    filesFail=false;
    await page.getByRole('button',{name:'Refresh file list',exact:true}).click();
    await page.waitForFunction(()=>!document.getElementById('refreshSequences').disabled&&document.getElementById('sequencehint').textContent==='');
    await page.setViewportSize({width:390,height:844});
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
    await page.screenshot({path:'build/recent-sequences-mobile.png',fullPage:true});
    await page.setViewportSize({width:1280,height:900});
    await page.screenshot({path:'build/recent-sequences-desktop.png',fullPage:true});

    assert(await page.getByRole('checkbox',{name:'Loop playback',exact:true}).isChecked());
    state.mode=1;
    await page.locator('#loop').uncheck();
    await page.waitForFunction(()=>!document.getElementById('loop').disabled&&document.getElementById('notice').textContent.startsWith('Loop disabled.'));
    assert.equal(new URLSearchParams(latest('/loop').body.toString()).get('enable'),'0');
    assert.equal(state.mode,1); // Changing looping does not restart or stop playback.
    await page.reload();
    await page.waitForFunction(()=>document.getElementById('brightness').value==='25');
    assert(!await page.locator('#loop').isChecked());
    loopSaveFails=true;
    await page.locator('#loop').click(); // The rejected save can roll back before check() returns.
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Settings write failed'&&!document.getElementById('loop').disabled);
    assert(!await page.locator('#loop').isChecked());
    assert.equal(state.settings.loop,false);
    loopSaveFails=false;
    await page.locator('#loop').check();
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Loop enabled.'&&!document.getElementById('loop').disabled);
    assert.equal(new URLSearchParams(latest('/loop').body.toString()).get('enable'),'1');
    state.mode=0;state.playbackComplete=true;
    await page.waitForFunction(()=>document.getElementById('state').textContent==='Finished');
    state.playbackComplete=false;
    assert((await page.locator('#wifidiag').textContent()).includes('2 disconnects'));
    assert((await page.locator('#wifidiag').textContent()).includes('retries paused'));
    assert((await page.locator('#wifidiag').textContent()).includes('Last disconnect reason: 4'));
    assert.equal(await page.getByRole('link',{name:'Wi-Fi setup',exact:true}).getAttribute('href'),'/wifi');
    assert.equal(await page.locator('#wifiform').count(),0);
    assert.equal(await page.locator('#files,#diagnostics,#updates,#logs,#sdmode').count(),0);
    assert(requests.some(r=>r.path==='/api/files')); // Load the picker once, not every status poll.
    assert.equal(await page.getByRole('link',{name:'LEDs',exact:true}).getAttribute('href'),'/leds');
    assert.equal(await page.getByRole('link',{name:'LED setup',exact:true}).getAttribute('href'),'/leds#setup');
    assert.equal(await page.locator('#colorfade,#dmaform,#armtest,#halldiag,#connectors').count(),0);
    await page.locator('#brightness').fill('42');
    await page.getByRole('button',{name:'Save brightness',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Settings saved.');
    assert.equal(new URLSearchParams(latest('/b').body.toString()).get('value'),'42');
    await page.locator('#spokes').fill('80');
    await page.getByRole('button',{name:'Save mapping',exact:true}).click();
    await page.waitForResponse(r=>r.url().endsWith('/status'));
    assert.equal(new URLSearchParams(latest('/mapcfg').body.toString()).get('usepa'),'0');
    assert.equal(new URLSearchParams(latest('/mapcfg').body.toString()).get('spokes'),'80');
    await page.getByRole('navigation',{name:'Main',exact:true}).getByRole('link',{name:'SD Card',exact:true}).click();
    await page.waitForURL(origin+'/sd');
    await page.waitForSelector('#filelist tr');
    assert.equal(await page.locator('#filelist img').count(),0);
    assert((await page.locator('#filelist').textContent()).includes('<img src=x'));
    const names=()=>page.locator('#filelist tr td:first-child').allTextContents();
    assert.deepEqual((await names()).slice(0,2),['a-folder/','Z-folder/']);
    assert.deepEqual((await names()).filter(n=>/^file\d/i.test(n)),['File2.fseq','file10.fseq']);
    await page.locator('#filesort').selectOption('name-desc');
    assert.deepEqual((await names()).filter(n=>/^file\d/i.test(n)),['file10.fseq','File2.fseq']);
    await page.locator('#filesort').selectOption('date-desc');
    assert.deepEqual((await names()).slice(0,5),['Z-folder/','a-folder/','file10.fseq','File2.fseq','demo.fseq']);
    await page.reload();await page.waitForSelector('#filelist tr');
    assert.equal(await page.locator('#filesort').inputValue(),'date-desc');
    await page.locator('#filesort').selectOption('date-asc');
    assert.deepEqual((await names()).slice(2,5),['demo.fseq','File2.fseq','file10.fseq']);
    assert.equal(await page.locator('#filelist tr').last().locator('td').nth(2).textContent(),'Unknown');
    await page.locator('#filesort').selectOption('name-asc');
    assert.equal(await page.locator('#headerbuild').textContent(),'Ver1.2 \u00b7 Build 42');
    await page.getByRole('button',{name:'Wi-Fi details',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('diagnostic').textContent.includes('lastApDisconnectReason'));
    await page.getByRole('button',{name:'Saved crash report',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('diagnostic').textContent.includes('ESP_ERR_NOT_FOUND'));
    assert.equal(await page.locator('#sdladder').textContent(),'Attempt order: 4-bit / 20 MHz → 1-bit / 20 MHz → 1-bit / 400 kHz.');
    assert.deepEqual(await page.locator('#sdfreq option').evaluateAll(options=>options.map(o=>o.value)),['400','20000','40000']);
    await page.locator('#sdmode').selectOption('1');await page.locator('#sdfreq').selectOption('20000');
    await page.evaluate(()=>refresh());
    assert.equal(await page.locator('#sdfreq').inputValue(),'20000'); // Polling preserves edits.
    await page.getByRole('button',{name:'Save and mount',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('sdinfo').textContent==='1-bit at 20000 kHz');
    assert.equal(new URLSearchParams(latest('/sd/config').body.toString()).get('fallback'),'1');
    assert.equal(new URLSearchParams(latest('/sd/config').body.toString()).get('recover'),'1');
    assert((await page.locator('#sdladder').textContent()).includes('1-bit / 20 MHz'));
    await page.locator('#sdfallback').uncheck();
    assert(await page.locator('#sdrecover').isDisabled());
    assert(!(await page.locator('#sdladder').textContent()).includes('400 kHz'));
    await page.locator('#sdfallback').check();
    await page.locator('#sdrecover').uncheck();
    await page.getByRole('button',{name:'Save and mount',exact:true}).click();
    await page.waitForResponse(r=>r.url().endsWith('/status'));
    assert.equal(new URLSearchParams(latest('/sd/config').body.toString()).get('recover'),'0');
    await page.locator('#sdstepdown').click();
    await page.waitForFunction(()=>document.getElementById('sdrecovery').textContent.includes('recovering'));
    assert(await page.locator('#remount').isDisabled());
    assert(await page.locator('#cancelsdspeed').isDisabled());
    assert(await page.locator('button[value="/ota"]').isDisabled());
    Object.assign(state.sd,{ready:true,currentWidth:1,freq:400});
    Object.assign(state.sd.recovery,{running:false,state:'complete'});
    Object.assign(state.sdTools,{running:false,operation:'speed',state:'idle'});
    await page.evaluate(()=>refresh());
    assert((await page.locator('#sdinfo').textContent()).includes('1-bit at 400'));
    assert((await page.locator('#sdrecovery').textContent()).includes('complete'));
    assert(!await page.locator('#runsdspeed').isDisabled());
    mountAsync=true;
    await page.locator('#remount').click();
    await page.waitForFunction(()=>document.getElementById('sdtoolstatus').textContent==='SD mount: mounting.');
    assert(await page.locator('#remount').isDisabled());
    Object.assign(state.sd,{ready:true,freq:20000});
    Object.assign(state.sdTools,{running:false,state:'complete'});
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='SD card mounted.');
    await page.waitForSelector('#filelist tr');
    mountAsync=false;
    assert.equal(new URLSearchParams(latest('/sd/config').body.toString()).get('freq'),'20000');
    await page.locator('#sdtestsize').selectOption('1');
    await page.getByRole('button',{name:'Run SD speed test',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('sdtoolstatus').textContent.includes('writing'));
    assert.equal(new URLSearchParams(latest('/sd/speed').body.toString()).get('mib'),'1');
    assert(await page.getByRole('button',{name:'Install over Wi-Fi',exact:true}).isDisabled());
    assert(await page.getByRole('button',{name:'Retry mount',exact:true}).isDisabled());
    Object.assign(state.sdTools,{phase:'reading',bytesWritten:1048576,bytesRead:524288,writeMiBps:0.8});
    await page.waitForFunction(()=>document.getElementById('sdprogress').value===75);
    Object.assign(state.sdTools,{running:false,state:'complete',phase:'done',bytesRead:1048576,verified:true,readMiBps:1.1,maxRead_ms:21.5,maxWrite_ms:35.2,elapsed_ms:2300});
    await page.waitForFunction(()=>document.getElementById('sdtoolstatus').textContent.includes('All data verified'));
    assert.equal(await page.locator('#sdread').textContent(),'1.10 MiB/s');
    assert((await page.locator('#sdtiming').textContent()).includes('21.50 ms'));
    Object.assign(state.sdTools,{state:'failed',phase:'writing',bytesWritten:32768,bytesRead:0,verified:false,error:'SD write failed at byte 32768: I/O error (errno 5); Closing SD test file failed: I/O error (errno 5)',temporaryFile:'/.lpov-speed-test.tmp'});
    state.sd.ioErrors={count:1,lastError:'ESP_ERR_INVALID_CRC',command:25,busWidth:4,clockKHz:8000};
    await page.waitForFunction(()=>document.getElementById('sdtoolstatus').textContent.includes('Test incomplete'));
    assert((await page.locator('#sdtoolstatus').textContent()).includes('0 bytes read and verified'));
    assert((await page.locator('#sdtoolstatus').textContent()).includes('partial results'));
    assert((await page.locator('#sdtoolstatus').textContent()).includes('SD write failed at byte 32768'));
    assert((await page.locator('#sdinfo').textContent()).includes('ESP_ERR_INVALID_CRC on CMD25 at 4-bit/8000 kHz'));
    state.sdTools.temporaryFile='';
    await page.waitForSelector('#filelist tr');
    await page.locator('#sdtestsize').selectOption('16');
    await page.getByRole('button',{name:'Run SD speed test',exact:true}).click();
    await page.getByRole('button',{name:'Cancel speed test',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('sdtoolstatus').textContent.includes('cancelled'));
    await page.getByText('Format SD card (erase files)',{exact:true}).click();
    await page.locator('#formatconfirm').fill('erase');
    await page.getByRole('button',{name:'Erase and format SD card',exact:true}).click();
    assert.equal(latest('/sd/format'),undefined); // Invalid confirmation never sends a request.
    await page.locator('#formatconfirm').fill('ERASE');
    await page.getByRole('button',{name:'Erase and format SD card',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('formatsd').disabled);
    assert.equal(new URLSearchParams(latest('/sd/format').body.toString()).get('confirm'),'ERASE SD CARD');
    assert.equal(await page.locator('#formatconfirm').inputValue(),'');
    assert(await page.getByRole('button',{name:'Cancel speed test',exact:true}).isDisabled());
    Object.assign(state.sdTools,{running:false,state:'failed',error:'Card did not respond'});
    await page.waitForFunction(()=>document.getElementById('sdtoolstatus').textContent.includes('Card did not respond'));
    assert(await page.getByRole('button',{name:'Install over Wi-Fi',exact:true}).isEnabled());
    await page.locator('#formatconfirm').fill('ERASE');
    await page.getByRole('button',{name:'Erase and format SD card',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('formatsd').disabled);
    state.sd.ready=true;Object.assign(state.sdTools,{running:false,state:'complete',error:''});
    await page.waitForFunction(()=>document.getElementById('sdtoolstatus').textContent==='SD format: complete.');
    await page.waitForSelector('#filelist tr');
    const payload=Buffer.from([0,1,2,255,3,13,10]);
    await page.locator('#file').setInputFiles({name:'space #.fseq',mimeType:'application/octet-stream',buffer:payload});
    await page.getByRole('button',{name:'Upload to this folder',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='File uploaded.');
    assert.equal(latest('/upload').query.get('path'),'/space #.fseq');
    assert.deepEqual(latest('/upload').body,payload);
    assert.equal(latest('/upload').type,'application/octet-stream');
    assert(+latest('/upload').query.get('modified')>315532800);
    assert.equal(await page.evaluate(()=>sequenceHistory.list()[0]),'/space #.fseq');
    // A failed mount must leave diagnostics, logs and direct OTA usable.
    mountFails=true;
    await page.getByRole('button',{name:'Retry mount',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='SD mount failed');
    assert.equal(await page.locator('#sdstate').textContent(),'Not mounted');
    assert.equal(await page.locator('#filelist tr').count(),0);
    assert(await page.getByRole('button',{name:'Upload to this folder',exact:true}).isDisabled());
    assert(await page.getByRole('button',{name:'Save firmware.bin on SD',exact:true}).isDisabled());
    assert(await page.getByRole('button',{name:'Run SD speed test',exact:true}).isDisabled());
    assert(await page.getByRole('button',{name:'Install over Wi-Fi',exact:true}).isEnabled());
    await page.getByRole('button',{name:'SD card status',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('diagnostic').textContent.includes('"ready": false'));
    await page.locator('#firmware').setInputFiles({name:'lpovxlm.bin',mimeType:'application/octet-stream',buffer:payload});
    await page.getByRole('button',{name:'Install over Wi-Fi',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Firmware image rejected');
    assert((await page.locator('#notice').getAttribute('class')).includes('error'));
    state.error='SD read failed';
    await page.waitForFunction(()=>document.getElementById('state').textContent.includes('SD read failed'));
    await page.getByRole('button',{name:'Refresh logs',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('logtext').textContent.includes('Native controller ready'));
    await page.evaluate(()=>scrollTo(0,0));
    await page.screenshot({path:'build/native-web-desktop.png',fullPage:true});
    await page.setViewportSize({width:390,height:844});
    await page.screenshot({path:'build/native-web-mobile.png',fullPage:true});
    assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),true);
    await page.locator('#logs').scrollIntoViewIfNeeded();
    assert.equal(await page.evaluate(()=>Math.round(document.querySelector('header').getBoundingClientRect().top)),0);
    mountFails=false;
    await page.getByRole('button',{name:'Retry mount',exact:true}).click();
    await page.waitForSelector('#filelist tr');
    await page.locator('#filelist tr').filter({has:page.getByText('demo.fseq',{exact:true})}).getByRole('button',{name:'Play',exact:true}).click();
    await page.waitForURL(origin+'/#playback');
    await page.waitForFunction(()=>document.getElementById('sequence').value==='/demo.fseq');
    // Old bookmarks reach the moved controls, including hash-only bookmarks.
    for(const path of ['/#files','/#diagnostics','/#updates','/#logs','/files','/diagnostics','/updates','/ota','/logs']){
      await page.goto(origin+path);
      const section=path==='/ota'?'updates':path.replace(/^\/#?/, '');
      await page.waitForURL(origin+'/sd#'+section);
      assert(await page.locator('#'+section).isVisible());
    }
    await page.setViewportSize({width:320,height:640});
    assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),true);
    assert.deepEqual(errors,[]);
    console.log('Native web smoke tests passed: controller settings, SD navigation/mount recovery, escaped files, raw upload, playback, diagnostics/OTA without SD, logs, legacy links, sticky header and mobile layout.');
  }finally{await browser.close();await new Promise(resolve=>server.close(resolve));}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});

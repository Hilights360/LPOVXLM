// Browser integration checks against a mock controller; no physical GPIO claim.
const assert=require('node:assert/strict');
const fs=require('node:fs');
const http=require('node:http');
const {chromium}=require('../build/ui-test/node_modules/playwright');
const requests=[];
const state={mode:0,ledProtocol:0,error:'',firmwareBuild:'LED pages test',
  firmwareVersion:'1.2',firmwareBuildNumber:17,
  settings:{arms:4,pixels:144,brightness:0,centerBrightness:10,centerDimming:false,spokes:80,duty:30,strobe:false,phase:0,armClockwise:false,rotationClockwise:true},sd:{ready:false},rpm:0,
  dmaTest:{running:false,state:'idle',phase:'gpio',framesCompleted:0,framesTarget:10,error:'',
    gpio:{meanTotal_us:2000},dma:{meanTotal_us:1500,meanPack_us:1100,meanSubmitToDone_us:390},speedup:4/3}};
let ready=true,failSave=false;
state.settings.autoDuty=false;
state.dutyControl={available:true,feasible:true,continuous:false,calculatedDuty:25,effectiveDuty:30,active:false};
const signal={running:false,pin:42,pattern:0,high:false};
function output(){return {ready,protocol:state.ledProtocol?'APA102':'SK9822',lastError:ready?'ESP_OK':'ESP_ERR_INVALID_STATE',
  clockPin:42,motorSpeedPwmPin:1,clockLevel:signal.running&&signal.pin===42&&signal.high?1:0,pixelsPerStrip:state.settings.pixels,
  brightnessPercent:state.settings.brightness,transmissions:120,signalCheck:signal,
  armOrderTest:{running:state.mode===12,activeArm:state.mode===12?1:0,remainingSeconds:state.mode===12?60:0},
  arms:[21,38,18,7].map((pin,i)=>({arm:i+1,connector:1+i*4,dataPin:pin,active:i<state.settings.arms,
    angleDegrees:i*(state.settings.armClockwise===state.settings.rotationClockwise?90:-90),
    level:signal.running&&signal.pin===pin&&signal.high?1:0}))};}
const server=http.createServer(async(req,res)=>{
  const url=new URL(req.url,'http://localhost'),chunks=[];
  for await(const chunk of req)chunks.push(chunk);
  const params=new URLSearchParams(Buffer.concat(chunks).toString());
  requests.push({path:url.pathname,method:req.method,params});
  let result={ok:true};
  if(req.method==='POST'){
    if(!ready&&['/led/solid','/led/blink','/led/signal','/colorfade','/led/spokes','/led/arm-order'].includes(url.pathname)){
      res.statusCode=409;result={error:'LED output unavailable'};
    }else if(url.pathname==='/led/config'){
      if(failSave){res.statusCode=500;result={error:'Settings write failed'};}
      else {state.ledProtocol=+params.get('protocol');state.settings.arms=+params.get('arms');state.settings.pixels=+params.get('pixels');state.mode=0;signal.running=false;}
    }else if(url.pathname==='/led/wiring'){
      if(failSave){res.statusCode=500;result={error:'Settings write failed'};}
      else {state.settings.armClockwise=params.get('order')==='cw';state.settings.rotationClockwise=params.get('rotation')==='cw';state.mode=0;signal.running=false;}
    }else if(url.pathname==='/led/blink'){
      state.mode=13;signal.running=false;
    }else if(url.pathname==='/led/arm-order'){
      state.mode=12;signal.running=false;
    }else if(url.pathname==='/b'){
      if(failSave){res.statusCode=500;result={error:'Settings write failed'};}
      else {
        state.settings.brightness=+params.get('value');
        if(params.has('center'))state.settings.centerBrightness=+params.get('center');
        if(params.has('fade'))state.settings.centerDimming=params.get('fade')==='1';
      }
    }
    else if(url.pathname==='/autoduty'){
      if(failSave){res.statusCode=500;result={error:'Settings write failed'};}
      else state.settings.autoDuty=params.get('enable')==='1';
    }
    else if(url.pathname==='/duty'){
      if(failSave){res.statusCode=500;result={error:'Settings write failed'};}
      else state.settings.duty=+params.get('percent');
    }
    else if(url.pathname==='/strobe'){
      if(failSave){res.statusCode=500;result={error:'Settings write failed'};}
      else state.settings.phase=Number(params.get('phase'))%360;
    }
    else if(url.pathname==='/stop'){state.mode=0;signal.running=false;state.dmaTest.running=false;state.dmaTest.state='cancelled';}
    else if(url.pathname==='/led/signal'){state.mode=8;signal.running=true;signal.pin=+params.get('pin');signal.pattern=+params.get('pattern');signal.high=signal.pattern===1;}
    else if(url.pathname==='/led/spokes'){state.mode=params.get('pattern')==='alignment'?14:params.get('pattern')==='quarters'?9:10;signal.running=false;}
    else if(url.pathname==='/diag/flash-proof'){
      if(failSave){res.statusCode=409;result={error:'Flash test cannot start'};}
      else {const running=params.get('enable')==='1',colorFrames=Number(params.get('colorFrames')||state.flashProof?.colorFrames||2);state.flashProof={running,state:running?'running':'cancelled',colorFrames,remainingSeconds:running?30:0,nominalPulse_us:235.6*colorFrames,nominalDutyPercent:10.1*colorFrames,bursts:12,latePreparedPulses:1};result=state.flashProof;}
    }
    else if(url.pathname==='/diag/dma'){signal.running=false;state.mode=5;state.dmaTest.running=true;state.dmaTest.state='running';result=state.dmaTest;}
    else if(['/colorfade','/led/solid','/armtest','/halldiag','/lanediag'].includes(url.pathname)){
      state.mode={'/colorfade':6,'/led/solid':7,'/armtest':2,'/halldiag':3,'/lanediag':4}[url.pathname];signal.running=false;
    }
  }else if(url.pathname==='/duty.js'){
    res.setHeader('Content-Type','application/javascript');res.end(fs.readFileSync('main/web/duty.js'));return;
  }else if(url.pathname==='/status')result=state;
  else if(url.pathname==='/diag/spi')result=output();
  else if(['/setup','/setup/','/setup.html'].includes(url.pathname)){
    res.writeHead(302,{Location:'/leds#setup'});res.end();return;
  }
  else {
    res.setHeader('Content-Type','text/html; charset=utf-8');
    res.end(fs.readFileSync('main/web/leds.html'));return;
  }
  res.setHeader('Content-Type','application/json');res.end(JSON.stringify(result));
});
(async()=>{
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
  const browser=await chromium.launch({channel:'msedge',headless:true});
  const page=await browser.newPage({viewport:{width:1280,height:900}}),errors=[];
  page.on('pageerror',e=>errors.push(e.message));
  const origin=`http://127.0.0.1:${server.address().port}`;
  const latest=path=>requests.filter(r=>r.method==='POST'&&r.path===path).at(-1);
  async function click(name,path){const done=page.waitForResponse(r=>r.url().endsWith(path)&&r.request().method()==='POST');await page.getByRole('button',{name,exact:true}).click();await done;}
  async function mode(text){await page.waitForFunction(text=>document.getElementById('state').textContent===text,text);}
  try{
    await page.goto(origin+'/leds');
    await mode('Stopped');
    assert.equal(await page.getByRole('heading',{level:1}).textContent(),'LPOVXLM Spinner Ver1.2 \u00b7 Build 17');
    assert.equal(await page.locator('#pixels').inputValue(),'144');
    assert.equal(await page.locator('#setupform,#colorfade,#dmaform').count(),3);
    assert((await page.locator('#darkwarning').textContent()).includes('Brightness is 0%'));
    assert(await page.locator('#centerBrightness').isDisabled());
    assert.equal(await page.locator('#centerDimming').isChecked(),false);
    assert.equal(await page.locator('#pinmap tr').count(),5);
    assert.equal(await page.locator('#signalpin option').count(),5);
    assert.equal(await page.locator('#signalpin option[value="1"]').count(),0);
    assert.equal(await page.locator('#signalpin').inputValue(),'42');
    assert((await page.locator('#motorinfo').textContent()).includes('GPIO1'));
    assert(!requests.some(r=>r.path==='/api/files'));
    await page.locator('#brightness').fill('10');await click('Save brightness','/b');
    await page.waitForFunction(()=>document.getElementById('darkwarning').textContent==='');
    for(const color of ['red','green','blue','white']){
      await click('Solid '+color,'/led/solid');await mode('Solid color');
      assert.equal(latest('/led/solid').params.get('color'),color);
    }
    await click('All-arm color fade','/colorfade');await mode('All-arm color fade');
    await click('White blinking','/led/blink');await mode('White blinking');
    assert.equal(state.settings.brightness,10);
    await click('Stop lights','/stop');await mode('Stopped');
    await click('Arm RGB test','/armtest');await mode('Arm RGB test');
    await click('Connector colors','/lanediag');await mode('Connector colors');
    await click('Hall sensor test','/halldiag');await mode('Hall test');
    assert.equal(await page.locator('#quarterinfo').textContent(),'80 spokes: 1-20 red; 21-40 green; 41-60 blue; 61-80 white.');
    await click('Quarter colors','/led/spokes');await mode('Quarter colors');
    assert.equal(latest('/led/spokes').params.get('pattern'),'quarters');
    assert((await page.locator('#darkwarning').textContent()).includes('Waiting for rotation'));
    assert.equal(state.sd.ready,false); // Spatial tests do not request a file or SD mount.
    assert(!requests.some(r=>r.path==='/play'||r.path==='/sd/reinit'));
    state.rpm=117;
    await page.waitForFunction(()=>document.getElementById('darkwarning').textContent==='');
    assert.equal(await page.locator('#armorder').inputValue(),'ccw');
    assert.equal(await page.locator('#rotationorder').inputValue(),'cw');
    assert.equal(await page.locator('#wireleft').textContent(),'Arm 2');
    await click('Light arms in order','/led/arm-order');await mode('Arm-order test');
    assert((await page.locator('#armorderstatus').textContent()).includes('Lighting arm 1 (red)'));
    assert(!requests.some(r=>r.path==='/led/wiring')); // Observing never changes saved geometry.
    await page.locator('#armorder').selectOption('cw');
    await page.evaluate(()=>refresh());
    assert.equal(await page.locator('#armorder').inputValue(),'cw'); // Keep an unsaved observation.
    assert.equal(await page.locator('#wireright').textContent(),'Arm 2');
    assert((await page.locator('#wiringsaved').textContent()).includes('arms counterclockwise'));
    await click('Save arm wiring','/led/wiring');await mode('Stopped');
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Arm wiring saved.');
    assert.equal(latest('/led/wiring').params.get('order'),'cw');
    assert.equal(latest('/led/wiring').params.get('rotation'),'cw');
    assert.deepEqual(output().arms.map(a=>a.angleDegrees),[0,90,180,270]);
    await page.locator('#rotationorder').selectOption('ccw');
    await click('Save arm wiring','/led/wiring');
    await page.waitForFunction(()=>document.getElementById('wiringsaved').textContent==='Saved: arms clockwise; rotor counterclockwise.');
    assert.deepEqual(output().arms.map(a=>a.angleDegrees).map(n=>n===0?0:n),[0,-90,-180,-270]);
    await page.reload();await mode('Stopped');
    assert.equal(await page.locator('#armorder').inputValue(),'cw');
    assert.equal(await page.locator('#rotationorder').inputValue(),'ccw');
    failSave=true;await page.locator('#armorder').selectOption('ccw');
    await click('Save arm wiring','/led/wiring');
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Settings write failed');
    assert.equal(state.settings.armClockwise,true);
    assert.equal(await page.locator('#armorder').inputValue(),'ccw');
    failSave=false;await page.locator('#rotationorder').selectOption('cw');
    await click('Save arm wiring','/led/wiring');
    await page.waitForFunction(()=>document.getElementById('wiringsaved').textContent==='Saved: arms counterclockwise; rotor clockwise.');
    assert((await page.locator('#spoketiming').textContent()).includes('30% duty. 117.0 RPM'));
    await click('Alternating spokes','/led/spokes');await mode('Alternating spokes');
    assert.equal(latest('/led/spokes').params.get('pattern'),'alternating');
    // Temporary hardware flash trial must preserve normal settings and expose
    // its nominal pulse, cancellation, automatic expiry, and start errors.
    assert(await page.locator('#startflashproof').isDisabled());
    const preFlash={mode:state.mode,ledProtocol:state.ledProtocol,settings:structuredClone(state.settings)};
    state.mode=1;state.ledProtocol=1;state.settings.arms=4;state.settings.pixels=144;
    await page.evaluate(()=>refresh());
    await page.locator('#flashwidth').selectOption('1');await page.evaluate(()=>refresh());
    assert.equal(await page.locator('#flashwidth').inputValue(),'1');
    await click('Test short flashes for 30 seconds','/diag/flash-proof');
    await page.waitForFunction(()=>document.getElementById('flashproofstatus').textContent.includes('235.6'));
    assert(await page.locator('#duty').isDisabled());assert(await page.locator('#startflashproof').isDisabled());
    assert((await page.locator('#autodutyinfo').textContent()).includes('Short flash verification'));
    await page.locator('#flashwidth').selectOption('2');
    await page.waitForFunction(()=>document.getElementById('flashproofstatus').textContent.includes('471.2'));
    assert.equal(latest('/diag/flash-proof').params.get('colorFrames'),'2');
    await click('Return to normal timing','/diag/flash-proof');
    await page.waitForFunction(()=>document.getElementById('flashproofstatus').textContent.includes('cancelled'));
    assert(await page.locator('#duty').isEnabled());
    assert.equal(await page.locator('#duty').inputValue(),String(state.settings.duty));
    failSave=true;await click('Test short flashes for 30 seconds','/diag/flash-proof');
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Flash test cannot start');
    assert(!state.flashProof.running);failSave=false;
    await click('Test short flashes for 30 seconds','/diag/flash-proof');
    await page.waitForFunction(()=>document.getElementById('flashproofstatus').textContent.includes('seconds remaining'));
    state.flashProof.running=false;state.flashProof.state='complete';await page.evaluate(()=>refresh());
    await page.waitForFunction(()=>document.getElementById('flashproofstatus').textContent.includes('complete'));
    assert.deepEqual(state.settings,{...preFlash.settings,arms:4,pixels:144});
    Object.assign(state,preFlash);delete state.flashProof;await page.evaluate(()=>refresh());
    await page.locator('#centerDimming').check();
    assert(await page.locator('#centerBrightness').isEnabled());
    await page.locator('#centerBrightness').fill('2');
    await page.evaluate(()=>refresh());
    assert.equal(await page.locator('#centerBrightness').inputValue(),'2');
    assert(await page.locator('#centerDimming').isChecked());
    await click('Save brightness','/b');
    await page.waitForFunction(()=>document.getElementById('centerinfo').textContent.includes('center 2% to tips 10%'));
    assert.equal(latest('/b').params.get('center'),'2');assert.equal(latest('/b').params.get('fade'),'1');
    assert.equal(state.mode,10);
    failSave=true;await page.locator('#centerBrightness').fill('4');await click('Save brightness','/b');
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Settings write failed');
    assert.equal(state.settings.centerBrightness,2);assert.equal(await page.locator('#centerBrightness').inputValue(),'4');
    failSave=false;await page.locator('#centerBrightness').fill('2');await click('Save brightness','/b');
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Brightness saved.');
    assert.equal(await page.locator('#duty').inputValue(),'30');
    await page.locator('#duty').fill('55');
    await page.evaluate(()=>refresh()); // Polling must preserve an unsaved edit.
    assert.equal(await page.locator('#duty').inputValue(),'55');
    await click('Save duty','/duty');
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Duty saved.');
    assert.equal(latest('/duty').params.get('percent'),'55');
    assert.equal(state.mode,10); // Adjust duty without stopping the test.
    assert((await page.locator('#spoketiming').textContent()).includes('55% duty'));
    failSave=true;await page.locator('#duty').fill('40');await click('Save duty','/duty');
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Settings write failed');
    assert.equal(state.settings.duty,55);assert.equal(await page.locator('#duty').inputValue(),'40');
    failSave=false;await click('Save duty','/duty');
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Duty saved.');
    state.settings.duty=30; // A change from Controller syncs when not editing.
    await page.waitForFunction(()=>document.getElementById('duty').value==='30');
    state.settings.strobe=true;
    await page.waitForFunction(()=>document.getElementById('dutyhint').textContent.startsWith('Angular strobe is enabled'));
    state.settings.strobe=false;
    state.rpm=0;
    await page.waitForFunction(()=>document.getElementById('darkwarning').textContent.includes('Waiting for rotation'));
    state.settings.duty=0;
    await page.waitForFunction(()=>document.getElementById('darkwarning').textContent.includes('Duty is 0%'));
    state.settings.duty=30;state.settings.spokes=81;
    await page.waitForFunction(()=>document.getElementById('quarterinfo').textContent==='81 spokes: 1-21 red; 22-41 green; 42-61 blue; 62-81 white.');
    state.settings.spokes=80;
    await page.locator('#autoDuty').check();
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Auto-calc enabled.'&&!document.getElementById('autoDuty').disabled);
    assert.equal(latest('/autoduty').params.get('enable'),'1');
    assert.equal(state.settings.duty,30);assert.equal(state.mode,10);
    assert(await page.locator('#duty').isDisabled());assert(await page.locator('#saveDuty').isDisabled());
    assert.equal(await page.locator('#duty').inputValue(),'25');
    state.rpm=180;state.dutyControl.calculatedDuty=40;state.dutyControl.active=true;
    await page.evaluate(()=>refresh());
    assert.equal(await page.locator('#duty').inputValue(),'40');
    assert((await page.locator('#autodutyinfo').textContent()).includes('40% duty at 180.0 RPM'));
    state.fileSettings={effectiveSpokes:256};await page.evaluate(()=>refresh());
    assert((await page.locator('#autodutyinfo').textContent()).includes('(256 spokes)'));
    assert((await page.locator('#quarterinfo').textContent()).startsWith('256 spokes: 1-64 red;'));
    delete state.fileSettings;
    await page.reload();await mode('Alternating spokes');
    assert(await page.locator('#autoDuty').isChecked());
    failSave=true;await page.locator('#autoDuty').click();
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Settings write failed'&&!document.getElementById('autoDuty').disabled);
    assert(await page.locator('#autoDuty').isChecked());assert(await page.locator('#duty').isDisabled());
    failSave=false;state.dutyControl.available=false;state.rpm=0;await page.evaluate(()=>refresh());
    assert((await page.locator('#autodutyinfo').textContent()).includes('waiting for rotation'));
    state.dutyControl.available=true;state.dutyControl.feasible=false;state.rpm=500;await page.evaluate(()=>refresh());
    assert((await page.locator('#autodutyinfo').textContent()).includes('RPM is too high'));
    state.dutyControl.feasible=true;state.dutyControl.continuous=true;state.dutyControl.calculatedDuty=100;
    await page.evaluate(()=>refresh());
    assert((await page.locator('#autodutyinfo').textContent()).includes('100% continuous output'));
    await page.locator('#autoDuty').uncheck();
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Auto-calc off. Manual timing restored.'&&!document.getElementById('autoDuty').disabled);
    assert.equal(await page.locator('#duty').inputValue(),'30');assert(await page.locator('#duty').isEnabled());
    assert.equal(state.settings.duty,30);state.rpm=0;
    await click('Stop lights','/stop');await mode('Stopped');
    assert.equal(await page.locator('#darkwarning').textContent(),'');
    assert(await page.locator('#stoplevel').isDisabled());
    await click('Show level pattern','/led/spokes');await mode('Arm alignment');
    assert.equal(latest('/led/spokes').params.get('pattern'),'alignment');
    assert((await page.locator('#darkwarning').textContent()).includes('Waiting for rotation'));
    state.rpm=120;state.settings.duty=0;state.settings.strobe=true;state.settings.spokes=1;
    await page.waitForFunction(()=>document.getElementById('darkwarning').textContent==='');
    assert((await page.locator('#levelstatus').textContent()).includes('Level pattern running'));
    const saveCount=()=>requests.filter(r=>r.path==='/strobe'&&r.method==='POST').length;
    await page.locator('#offset').fill('23.4');await page.evaluate(()=>refresh());
    assert.equal(await page.locator('#offset').inputValue(),'23.4');
    assert.equal(saveCount(),0); // Drafts stay local until saved.
    await click('Save offset','/strobe');
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Arm offset saved.');
    assert.equal(state.settings.phase,23.4);assert.equal(state.mode,14);
    assert.deepEqual([...latest('/strobe').params.keys()],['phase']);
    assert.equal(state.settings.duty,0);assert.equal(state.settings.strobe,true);assert.equal(state.settings.spokes,1);
    await click('Rotate clockwise','/strobe');
    await page.waitForFunction(()=>document.getElementById('offset').value==='22.4');
    await page.locator('#offsetstep').selectOption('0.1');
    await click('Rotate counterclockwise','/strobe');
    await page.waitForFunction(()=>document.getElementById('offset').value==='22.5');
    state.settings.rotationClockwise=false;await page.evaluate(()=>refresh());
    await click('Rotate clockwise','/strobe');
    await page.waitForFunction(()=>document.getElementById('offset').value==='22.6');
    await page.locator('#offset').fill('359.9');await page.locator('#offsetstep').selectOption('1');
    await click('Rotate clockwise','/strobe');
    await page.waitForFunction(()=>document.getElementById('offset').value==='0.9');
    await page.reload();await mode('Arm alignment');
    assert.equal(await page.locator('#offset').inputValue(),'0.9');
    failSave=true;await page.locator('#offset').fill('-45.5');await click('Save offset','/strobe');
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Settings write failed');
    assert.equal(state.settings.phase,0.9);assert.equal(state.mode,14);
    await page.evaluate(()=>refresh());assert.equal(await page.locator('#offset').inputValue(),'-45.5');
    assert(await page.locator('#saveoffset').isEnabled());failSave=false;
    await click('Reset offset to 0','/strobe');
    await page.waitForFunction(()=>document.getElementById('offset').value==='0');
    state.settings.phase=-12.3;await page.evaluate(()=>refresh());
    assert.equal(await page.locator('#offset').inputValue(),'-12.3'); // Controller-page changes sync.
    const beforeInvalid=saveCount();
    await page.locator('#offset').fill('361');await page.locator('#saveoffset').click();
    assert.equal(saveCount(),beforeInvalid);assert.equal(state.settings.phase,-12.3);
    await click('Stop level pattern','/stop');await mode('Stopped');
    assert.equal(state.settings.phase,-12.3);assert(await page.locator('#stoplevel').isDisabled());
    state.settings.duty=30;state.settings.strobe=false;state.settings.spokes=80;
    state.settings.rotationClockwise=true;
    for(const pin of ['42','38']){
    await page.locator('#signalpin').selectOption(pin);
    for(const pattern of ['0','1','2']){
      await page.locator('#signalpattern').selectOption(pattern);await click('Apply signal check','/led/signal');
      await mode('Signal check');
      assert.equal(latest('/led/signal').params.get('pin'),pin);
      assert.equal(latest('/led/signal').params.get('pattern'),pattern);
      if(pin==='42')await page.waitForFunction(high=>document.querySelector('#pinmap tr td:last-child').textContent===(high?'1':'0'),pattern==='1');
    }
    }
    await click('Stop lights','/stop');await mode('Stopped');
    await click('Run DMA test','/diag/dma');await mode('DMA speed test');
    assert.equal(latest('/diag/dma').params.get('mhz'),'4');
    assert(await page.locator('#rundma').isDisabled());assert(await page.locator('#dmaclock').isDisabled());
    state.dmaTest.phase='dma';state.dmaTest.framesCompleted=30;state.dmaTest.framesTarget=100;
    await page.waitForFunction(()=>document.getElementById('dmaresult').textContent.includes('30/100'));
    state.dmaTest.running=false;state.dmaTest.state='complete';state.mode=0;
    await page.waitForFunction(()=>document.getElementById('dmaresult').textContent.includes('1.33x speedup'));
    await page.locator('#dmaclock').selectOption('16');await click('Run DMA test','/diag/dma');
    assert.equal(latest('/diag/dma').params.get('mhz'),'16');
    await click('Stop DMA test','/stop');await mode('Stopped');
    ready=false;await click('Solid red','/led/solid');
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='LED output unavailable');
    await click('Quarter colors','/led/spokes');await mode('Stopped');
    assert.equal(await page.locator('#notice').textContent(),'LED output unavailable');
    await click('Show level pattern','/led/spokes');await mode('Stopped');
    assert.equal(await page.locator('#notice').textContent(),'LED output unavailable');
    ready=true;
    await page.getByRole('link',{name:'Setup',exact:true}).click();
    assert.equal(new URL(page.url()).pathname,'/leds');
    assert.equal(new URL(page.url()).hash,'#setup');
    await page.locator('#protocol').selectOption('1');await page.locator('#arms').fill('2');await page.locator('#pixels').fill('200');
    // Live test status and a brightness save must not overwrite setup edits.
    await page.locator('#brightness').fill('12');await click('Save brightness','/b');
    await page.waitForFunction(()=>document.getElementById('health').textContent.includes('12% brightness'));
    assert.equal(await page.locator('#protocol').inputValue(),'1');
    assert.equal(await page.locator('#arms').inputValue(),'2');
    assert.equal(await page.locator('#pixels').inputValue(),'200');
    await click('Save LED setup','/led/config');
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='LED setup saved.');
    assert.equal(latest('/led/config').params.get('protocol'),'1');
    assert.equal(await page.locator('#signalpin option').count(),3);
    await page.getByRole('link',{name:'Run an LED test',exact:true}).click();
    await click('Solid red','/led/solid');await mode('Solid color');
    assert((await page.locator('#health').textContent()).includes('APA102 | 2 arms | 200 pixels'));
    await click('Stop lights','/stop');await mode('Stopped');
    await page.reload();await page.waitForFunction(()=>document.getElementById('protocol').value==='1');
    assert(await page.locator('#centerDimming').isChecked());
    assert.equal(await page.locator('#centerBrightness').inputValue(),'2');
    assert.equal(await page.locator('#arms').inputValue(),'2');assert.equal(await page.locator('#pixels').inputValue(),'200');
    failSave=true;await page.locator('#protocol').selectOption('0');await click('Save LED setup','/led/config');
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Settings write failed');
    assert.equal(state.ledProtocol,1);failSave=false;
    for(const route of ['/leds','/setup','/setup/','/setup.html']){
      await page.goto(origin+route);await page.waitForSelector('#pinmap tr');
      assert.equal(new URL(page.url()).pathname,'/leds');
      if(route!=='/leds')assert.equal(new URL(page.url()).hash,'#setup');
      assert.equal(await page.locator('#signalpin option').count(),3);
      assert.equal(await page.locator('#setupform,#colorfade,#dmaform').count(),3);
      if(route==='/leds')await page.screenshot({path:'build/leds-desktop.png',fullPage:true});
      for(const width of [390,320]){
        await page.setViewportSize({width,height:844});
        assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),route+' width '+width);
      }
      if(route==='/leds')await page.screenshot({path:'build/leds-mobile.png',fullPage:true});
      await page.setViewportSize({width:1280,height:900});
    }
    assert.deepEqual(errors,[]);
    console.log('LED page passed: combined setup and tests, no-SD startup, preserved edits, brightness, patterns, signal checks, DMA, saved geometry, legacy links, errors, and mobile layout.');
  }finally{await browser.close();await new Promise(resolve=>server.close(resolve));}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});

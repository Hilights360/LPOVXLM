const assert = require('node:assert/strict');
const fs = require('node:fs');
const http = require('node:http');
const {chromium} = require('../build/ui-test/node_modules/playwright');
const html = fs.readFileSync('main/web/scope.html');
let tool = {operation:'speed',state:'idle',running:false}, capture = {}, id = 0;
let disconnected = false, failStart = false;
const requests = [];
const server = http.createServer(async(req,res)=>{
  const url = new URL(req.url,'http://localhost');let body='';for await(const chunk of req)body+=chunk;
  requests.push({path:url.pathname,method:req.method,body});
  if(disconnected){res.destroy();return;}
  let result = {};
  if(req.method==='POST'&&url.pathname==='/sd/scope'){
    if(failStart){res.statusCode=409;result={error:'SD read is still finishing; retry shortly'};}
    else {
      const p=new URLSearchParams(body),pin=+p.get('pin');
      capture={captureId:++id,running:true,state:'running',pin,signal:pin===10?'DAT0':'CMD',samples:[],requestedInterval_us:+p.get('interval'),samplesTarget:1024,calibrated:true,wasMounted:true,sdReady:false,error:'',restoreError:'',readError:'',calibrationError:''};
      capture.wifiOffRequested=p.get('wifiOff')==='1';
      if(pin===-1)Object.assign(capture,{signal:'All SD pins',captureMode:'sequential',channelsTarget:6,channels:[]});
      tool={operation:'scope',state:'running',running:true,captureId:capture.captureId,scopeWifiOff:capture.wifiOffRequested};result=capture;res.statusCode=202;
    }
  }else if(req.method==='POST'&&url.pathname==='/sd/cancel'){
    capture={...capture,state:'cancelled',running:false,sdReady:true};tool={...tool,state:'cancelled',running:false};result=tool;
  }else if(url.pathname==='/sd/tools')result=tool;
  else if(url.pathname==='/sd/scope/data')result=capture;
  else if(url.pathname==='/status')result={firmwareBuildNumber:123};
  else {res.setHeader('Content-Type','text/html');res.end(html);return;}
  res.setHeader('Content-Type','application/json');res.end(JSON.stringify(result));
});
function complete(changes={}){
  capture={...capture,running:false,state:'complete',sdReady:true,samples:[[0,3800,2900],[100,3900,3000],[200,null,null],[1600,4095,3150],[1700,3700,2800]],readError:'ESP_ERR_TIMEOUT',...changes};
  tool={...tool,running:false,state:capture.state};
}
(async()=>{
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
  const browser=await chromium.launch({channel:'msedge',headless:true});
  const page=await browser.newPage({viewport:{width:1280,height:900}}),errors=[];
  page.on('pageerror',e=>errors.push(e.message));
  const ready=()=>page.waitForFunction(()=>!document.getElementById('capture').disabled);
  const rendered=()=>page.waitForFunction(()=>document.getElementById('status').textContent.startsWith('Capture complete.'));
  try{
    await page.goto(`http://127.0.0.1:${server.address().port}/sd/scope`);await ready();
    assert.equal(await page.locator('#pin option').count(),7);
    assert.equal(await page.locator('#pin').inputValue(),'-1');
    await page.locator('#pin').selectOption('10');
    assert((await page.locator('#pinhelp').textContent()).includes('ADC1'));
    await page.locator('#capture').click();
    await page.waitForFunction(()=>!document.getElementById('cancel').disabled);
    const start=requests.filter(r=>r.method==='POST'&&r.path==='/sd/scope').at(-1);
    assert.equal(new URLSearchParams(start.body).get('pin'),'10');
    assert.equal(new URLSearchParams(start.body).get('interval'),'100');
    assert(await page.locator('#capture').isDisabled());
    complete();await rendered();
    assert((await page.locator('#quality').textContent()).includes('1 readings at or near'));
    assert((await page.locator('#quality').textContent()).includes('1 missed readings'));
    const stats=await page.locator('#stats').textContent();
    assert(stats.includes('2800.0 mV'));assert(stats.includes('200.0 mV'));assert(stats.includes('1.400 ms'));
    assert((await page.locator('#restore').textContent()).includes('remounted'));
    await page.locator('#scale').selectOption('auto');await page.locator('#zoom').selectOption('5');
    assert(!(await page.locator('#pan').isDisabled()));
    await page.locator('#pan').fill('650');await page.locator('#pan').dispatchEvent('input');
    await page.locator('#trace').scrollIntoViewIfNeeded();
    const box=await page.locator('#trace').boundingBox();await page.mouse.move(box.x+box.width/2,box.y+100);
    assert((await page.locator('#cursor').textContent()).includes('ms:'));
    const csvPending=page.waitForEvent('download');await page.locator('#csv').click();const csv=await csvPending;
    const csvText=fs.readFileSync(await csv.path(),'utf8');assert(csvText.includes('200,,,0\n'));assert(csvText.includes('1600,4095,,1'));
    const jsonPending=page.waitForEvent('download');await page.locator('#json').click();const json=await jsonPending;
    const saved=JSON.parse(fs.readFileSync(await json.path(),'utf8'));assert.equal(saved.samples[2][1],null);assert(saved.conditions.includes('paused'));
    // Raw fallback must never present counts as voltage.
    await ready();await page.locator('#pin').selectOption('12');assert((await page.locator('#pinhelp').textContent()).includes('ADC2'));
    await page.locator('#capture').click();await page.waitForFunction(()=>!document.getElementById('cancel').disabled);
    complete({calibrated:false,wasMounted:false,sdReady:false,calibrationError:'ESP_ERR_NOT_SUPPORTED',readError:'',samples:[[0,2345,null],[1000,2400,null]]});await rendered();
    assert((await page.locator('#stats').textContent()).includes('2345.0 counts'));
    assert((await page.locator('#quality').textContent()).includes('not volts'));
    assert((await page.locator('#restore').textContent()).includes('unmounted'));
    // Saturated ADC2 calibration once returned almost 5 V on the hardware.
    // Neither the chart, statistics, nor exports may present that as a voltage.
    await ready();await page.locator('#capture').click();await page.waitForFunction(()=>!document.getElementById('cancel').disabled);
    complete({samples:[[0,4095,4984],[120,4090,4973]],calibrated:true});await rendered();
    assert((await page.locator('#stats').textContent()).includes('4090.0 counts'));
    assert(!(await page.locator('#stats').textContent()).includes('4984'));
    assert((await page.locator('#quality').textContent()).includes('No valid in-range voltage'));
    // Restore failure must leave the acquired samples available for export.
    await ready();await page.locator('#capture').click();await page.waitForFunction(()=>!document.getElementById('cancel').disabled);
    complete({state:'failed',sdReady:false,restoreError:'SD mount failed at all permitted settings'});
    await page.waitForFunction(()=>document.getElementById('error').textContent.includes('SD mount failed'));
    assert(!(await page.locator('#csv').isDisabled()));
    assert((await page.locator('#restore').textContent()).includes('Retry mount'));
    // Cancel clears the busy state without requiring a successful capture.
    await ready();await page.locator('#capture').click();await page.waitForFunction(()=>!document.getElementById('cancel').disabled);
    await page.locator('#cancel').click();await page.waitForFunction(()=>document.getElementById('status').textContent.includes('Capture cancelled.'));await ready();
    failStart=true;await page.locator('#capture').click();await page.waitForFunction(()=>document.getElementById('error').textContent.includes('still finishing'));failStart=false;
    tool={operation:'mount',state:'running',running:true};await page.waitForFunction(()=>document.getElementById('capture').disabled);assert(await page.locator('#cancel').isDisabled());
    tool={operation:'speed',state:'idle',running:false};await ready();
    disconnected=true;await page.waitForFunction(()=>document.getElementById('status').textContent.includes('disconnected'));assert(await page.locator('#capture').isDisabled());disconnected=false;await ready();
    assert(!(await page.locator('#status').textContent()).includes('disconnected'));
    await page.locator('#capture').click();await page.waitForFunction(()=>!document.getElementById('cancel').disabled);
    complete({readError:'',samples:Array.from({length:1024},(_,i)=>[i*155,3700+Math.round(20*Math.sin(i/20)),2850+Math.round(16*Math.sin(i/20))])});await rendered();
    disconnected=true;await page.waitForFunction(()=>document.getElementById('status').textContent.includes('disconnected'));disconnected=false;await ready();await rendered();
    await page.locator('#scale').selectOption('auto');
    // A complete sample set must retain and display all six channels together.
    await ready();await page.locator('#pin').selectOption('-1');await page.locator('#capture').click();
    await page.waitForFunction(()=>!document.getElementById('cancel').disabled);
    assert.equal(new URLSearchParams(requests.filter(r=>r.method==='POST'&&r.path==='/sd/scope').at(-1).body).get('pin'),'-1');
    const channels=[[10,'DAT0'],[9,'DAT1'],[12,'CMD'],[14,'DAT2'],[13,'DAT3'],[11,'CLK']].map(([pin,signal],i)=>({pin,signal,calibrated:true,adcUnit:pin>10?2:1,startOffset_us:20000+i*170000,error:'',readError:'',calibrationError:'',samples:Array.from({length:1024},(_,n)=>[n*155,i===5?600+n%35:4095-n%8,i===5?500+n%25:null,i!==5])}));
    complete({channels,readError:''});await rendered();
    await page.waitForFunction(()=>document.querySelectorAll('#stats tbody tr').length===6);
    assert((await page.locator('#traceheading').textContent()).includes('All six SD pins - raw ADC'));
    assert((await page.locator('#settiming').textContent()).includes('not simultaneous'));
    assert((await page.locator('#status').textContent()).includes('6,144'));
    assert((await page.locator('#quality').textContent()).includes('5120 readings at or near'));
    assert((await page.locator('#stats').textContent()).includes('870.0 ms'));
    const groupCsvPending=page.waitForEvent('download');await page.locator('#csv').click();const groupCsv=await groupCsvPending;
    const lines=fs.readFileSync(await groupCsv.path(),'utf8').trim().split('\n');assert.equal(lines.length,6145);
    assert.equal(lines[0],'signal,gpio,start_offset_us,time_us,set_time_us,raw,millivolts,upper_limit');
    assert(lines[1].startsWith('DAT0,10,20000,0,20000,'));assert(lines[5121].startsWith('CLK,11,870000,0,870000,'));
    assert(lines.slice(1,5121).every(line=>line.split(',')[6]===''));
    const groupJsonPending=page.waitForEvent('download');await page.locator('#json').click();const groupJson=await groupJsonPending;
    const group=JSON.parse(fs.readFileSync(await groupJson.path(),'utf8'));assert.equal(group.channels.length,6);assert(group.conditions.includes('not simultaneous'));
    await page.locator('#units').selectOption('voltage');assert((await page.locator('#stats tr[data-pin="10"]').textContent()).includes('Unavailable'));
    assert((await page.locator('#stats tr[data-pin="11"]').textContent()).includes('500.0 mV'));
    await page.locator('#units').selectOption('raw');await page.locator('#scale').selectOption('auto');
    await page.reload();await rendered();assert.equal(await page.locator('#stats tbody tr').count(),6);
    await page.locator('#scale').selectOption('auto');
    await page.screenshot({path:'build/sd-scope-desktop.png',fullPage:true});
    await page.setViewportSize({width:390,height:844});
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
    await page.screenshot({path:'build/sd-scope-mobile.png',fullPage:true});
    // Radio-off captures survive the expected disconnection and retain their
    // measured condition across reload/export, independent of the form selection.
    await ready();await page.locator('#wifiOff').selectOption('1');
    assert((await page.locator('#wifihelp').textContent()).includes('Cancel is unavailable'));
    await page.locator('#capture').click();await page.waitForFunction(()=>!document.getElementById('cancel').disabled);
    assert.equal(new URLSearchParams(requests.filter(r=>r.method==='POST'&&r.path==='/sd/scope').at(-1).body).get('wifiOff'),'1');
    disconnected=true;await page.waitForFunction(()=>document.getElementById('status').textContent.includes('Wi-Fi paused'));
    assert(await page.locator('#cancel').isDisabled());assert(await page.locator('#wifiOff').isDisabled());
    complete({channels,readError:'',wifiOffDuringCapture:true,wifiRestored:true,wifiStoppedOffset_us:600000,wifiResumedOffset_us:2100000});
    disconnected=false;await rendered();
    assert((await page.locator('#radiocondition').textContent()).includes('OFF throughout'));
    assert((await page.locator('#radiocondition').textContent()).includes('restarted afterward'));
    const quietJsonPending=page.waitForEvent('download');await page.locator('#json').click();const quietJson=await quietJsonPending;
    assert(quietJson.suggestedFilename().includes('wifi-off'));
    const quietSaved=JSON.parse(fs.readFileSync(await quietJson.path(),'utf8'));assert(quietSaved.wifiOffRequested&&quietSaved.wifiOffDuringCapture&&quietSaved.wifiRestored);
    await page.reload();await rendered();assert.equal(await page.locator('#wifiOff').inputValue(),'0');
    assert((await page.locator('#radiocondition').textContent()).includes('OFF throughout'));
    await ready();await page.locator('#wifiOff').selectOption('1');await page.locator('#capture').click();await page.waitForFunction(()=>!document.getElementById('cancel').disabled);
    complete({channels:[],state:'failed',error:'Cannot stop Wi-Fi for capture',wifiOffDuringCapture:false,wifiRestored:true});
    await page.waitForFunction(()=>document.getElementById('radiocondition').textContent.includes('not confirmed'));
    assert((await page.locator('#error').textContent()).includes('Cannot stop Wi-Fi'));
    await page.locator('#wifiOff').selectOption('0');
    // Cancellation retains earlier pins and any partial final trace, with explicit set size.
    await ready();await page.locator('#capture').click();await page.waitForFunction(()=>!document.getElementById('cancel').disabled);
    capture.channels=[channels[0],{...channels[1],samples:channels[1].samples.slice(0,17),cancelled:true}];
    await page.locator('#cancel').click();await page.waitForFunction(()=>document.getElementById('status').textContent.includes('Capture cancelled.'));
    assert.equal(await page.locator('#stats tbody tr').count(),2);assert((await page.locator('#status').textContent()).includes('2 of 6'));
    assert.deepEqual(errors,[]);
    assert(!requests.some(r=>['/sd/format','/sd/config','/sd/speed','/ota'].includes(r.path)));
    console.log('SD scope page: single/all-six capture, common traces, range flags, voltage/raw units, sequential timing, 6144-row export, reload, partial cancellation, errors, and mobile layout passed.');
  }finally{await browser.close();server.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});

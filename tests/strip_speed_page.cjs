// UI checks with a mock controller; visual LED quality still requires inspection.
const assert=require('node:assert/strict'),fs=require('node:fs'),http=require('node:http');
const {chromium}=require('../build/ui-test/node_modules/playwright');
const state={firmwareVersion:'1.2',firmwareBuildNumber:14,firmwareBuild:'test',ledProtocol:1,
  settings:{brightness:60,centerBrightness:3,centerDimming:true,arms:4,pixels:144}};
let run=0,failStart=false,disconnected=false;
let test={runId:'0',running:false,state:'idle',error:'',clockHz:4000000,durationSeconds:30,
  elapsedSeconds:0,transmissions:0,updatesPerSecond:0,brightnessPercent:10,
  meanTransfer_us:0,maxTransfer_us:0,meanPack_us:0,meanSubmitToDone_us:0,wireTime_us:1178,
  pattern:'All red',protocol:'APA102',arms:4,pixelsPerStrip:144};
const requests=[];
const server=http.createServer(async(req,res)=>{
  const chunks=[];for await(const c of req)chunks.push(c);
  const params=new URLSearchParams(Buffer.concat(chunks).toString());
  requests.push({path:req.url,method:req.method,params});
  if(!['/status','/diag/strip-speed','/reboot'].includes(req.url)){
    res.setHeader('Content-Type','text/html');res.end(fs.readFileSync('main/web/speed.html'));return;
  }
  res.setHeader('Content-Type','application/json');
  if(disconnected){res.statusCode=503;res.end(JSON.stringify({error:'Disconnected'}));return;}
  if(req.method==='POST'&&req.url==='/diag/strip-speed'){
    if(params.get('action')==='stop'){
      if(test.running)test={...test,running:false,state:'cancelled'};
    }else if(failStart){res.statusCode=409;res.end(JSON.stringify({error:'LED output unavailable'}));return;}
    else test={...test,runId:String(++run),running:true,state:'running',error:'',
      clockHz:+params.get('hz'),durationSeconds:+params.get('seconds'),elapsedSeconds:0,transmissions:0};
  }
  res.end(JSON.stringify(req.url==='/status'?state:req.url==='/reboot'?{ok:true}:test));
});
(async()=>{
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
  const browser=await chromium.launch({channel:'msedge',headless:true}),page=await browser.newPage({viewport:{width:1280,height:900}}),errors=[];
  page.on('pageerror',e=>errors.push(e.message));
  const origin=`http://127.0.0.1:${server.address().port}`;
  async function click(id){const response=page.waitForResponse(r=>r.request().method()==='POST');await page.locator('#'+id).click();await response;await page.waitForFunction(()=>!busy);}
  async function idle(){await page.waitForFunction(()=>!busy&&!polling);}
  try{
    await page.goto(origin+'/speed-test');await page.waitForFunction(()=>!document.getElementById('start').disabled);
    assert.equal(await page.locator('#headerbuild').textContent(),'Ver1.2 · Build 14');
    assert.deepEqual(await page.locator('#clock option').evaluateAll(o=>o.map(x=>+x.value)),[2000000,4000000,8000000,10000000,16000000,20000000,26666666,40000000]);
    assert.equal(await page.locator('#clock').inputValue(),'4000000');
    assert(await page.locator('#pass').isDisabled());
    await page.locator('#duration').selectOption('15');await click('start');
    assert(test.running);assert.equal(test.clockHz,4000000);assert.equal(test.durationSeconds,15);
    assert(await page.locator('#start').isDisabled());assert(await page.locator('#clock').isDisabled());
    test={...test,elapsedSeconds:2,transmissions:800,updatesPerSecond:400,meanTransfer_us:1200,meanPack_us:600,meanSubmitToDone_us:600,maxTransfer_us:1300};
    await page.waitForFunction(()=>!document.getElementById('fail').disabled);
    assert(await page.locator('#pass').isDisabled()); // No premature visual success.
    await click('fail');assert(!test.running);
    assert.equal(await page.locator('#results tr').count(),1);
    assert((await page.locator('#results').textContent()).includes('Shows errors'));
    assert(await page.locator('#fail').isDisabled()); // Prevent duplicate record.
    await page.locator('#clock').selectOption('26666666');await click('start');
    assert.equal(test.clockHz,26666666); // Never round the request up to divisor 2 / 40 MHz.
    assert.equal(await page.locator('#rate').textContent(),'26.67 MHz');
    test={...test,elapsedSeconds:15,transmissions:7500,updatesPerSecond:500,running:false,state:'complete'};
    await page.waitForFunction(()=>!document.getElementById('pass').disabled);
    await click('pass');assert.equal(await page.locator('#results tr').count(),2);
    assert((await page.locator('#results').textContent()).includes('Looks correct'));
    assert((await page.locator('#results').textContent()).includes('26.67 MHz'));
    await page.locator('#recordsort').selectOption('date-asc');
    assert((await page.locator('#results tr').first().textContent()).includes('4 MHz'));
    await page.locator('#recordsort').selectOption('date-desc');
    assert((await page.locator('#results tr').first().textContent()).includes('26.67 MHz'));
    await page.locator('#recordsort').selectOption('clock-asc');
    assert((await page.locator('#results tr').first().textContent()).includes('4 MHz'));
    await page.locator('#recordsort').selectOption('clock-desc');
    assert((await page.locator('#results tr').first().textContent()).includes('26.67 MHz'));
    const download=page.waitForEvent('download');await page.locator('#download').click();assert.equal((await download).suggestedFilename(),'strip-speed-results.json');
    await page.reload();await page.waitForFunction(()=>document.querySelectorAll('#results tr').length===2);await idle();
    // Polling must preserve clock selection.
    await page.locator('#clock').selectOption('20000000');await page.waitForTimeout(850);assert.equal(await page.locator('#clock').inputValue(),'20000000');
    failStart=true;await click('start');assert((await page.locator('#notice').textContent()).includes('LED output unavailable'));failStart=false;
    state.settings.brightness=0;await page.waitForFunction(()=>document.getElementById('setup').textContent.includes('Set brightness above'));
    assert(await page.locator('#start').isDisabled());state.settings.brightness=60;
    await page.waitForFunction(()=>!document.getElementById('start').disabled);await click('start');
    test={...test,running:false,state:'failed',error:'ESP_ERR_TIMEOUT',elapsedSeconds:13,transmissions:3000};
    await page.waitForFunction(()=>document.getElementById('testerror').textContent==='ESP_ERR_TIMEOUT');assert(await page.locator('#pass').isDisabled());
    disconnected=true;await page.waitForFunction(()=>document.getElementById('state').textContent==='Controller disconnected');assert(await page.locator('#start').isDisabled());
    disconnected=false;await page.waitForFunction(()=>!document.getElementById('start').disabled);
    await click('start');await click('stop');assert(!test.running);
    await page.setViewportSize({width:390,height:844});await page.evaluate(()=>scrollTo(0,500));
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
    assert(Math.abs(await page.locator('header').evaluate(e=>e.getBoundingClientRect().top))<1);
    await page.screenshot({path:'build/strip-speed-mobile.png',fullPage:true});
    await page.locator('#clear').click();assert.equal(await page.locator('#results tr').count(),0);
    await page.reload();await idle();assert.equal(await page.locator('#results tr').count(),0);
    assert(!requests.some(r=>r.path==='/b'||r.path==='/duty'||r.path==='/play')); // Settings/playback untouched.
    assert.deepEqual(errors,[]);
    console.log('Strip speed page: rates, lifecycle, visual records, error states, persistence, export and mobile layout passed.');
  }finally{await browser.close();server.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});

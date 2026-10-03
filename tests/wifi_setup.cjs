// node tests/wifi_setup.cjs; uses the same Playwright/Edge setup as web_smoke.cjs.
const assert=require('node:assert/strict');
const fs=require('node:fs');
const http=require('node:http');
const {chromium}=require('../build/ui-test/node_modules/playwright');
const html=fs.readFileSync('main/web/wifi.html');
const root=fs.readFileSync('main/web/index.html');
const requests=[];
const wifi={ssid:'Workshop',station:'pov-test',ip:'',passwordSaved:true,apSsid:'POV-Spinner',
  mode:'AP only',apClients:1,apDisconnects:0,channel:1,uptime_ms:30000};
let savedPassword='Saved!pass#123',scanPolls=0,scanMode='normal',scanStarted=false;
let revealDelay=0;
const networks=[
  {ssid:'Workshop East',rssi:-41,channel:6,security:'WPA2',open:false},
  {ssid:'Guest',rssi:-60,channel:1,security:'Open',open:true},
  {ssid:'<img src=x onerror=alert(1)>',rssi:-72,channel:11,security:'WPA2',open:false}
];
function scanResult(){
  if(!scanStarted)return {state:'idle',running:false,networks:[]};
  if(scanPolls++<2)return {state:'scanning',running:true,networks:[]};
  return {state:scanMode==='failed'?'failed':'complete',running:false,
    error:scanMode==='failed'?'ESP_ERR_WIFI_TIMEOUT':'',truncated:false,
    networks:scanMode==='normal'?networks:[]};
}
const server=http.createServer(async(req,res)=>{
  const url=new URL(req.url,'http://localhost'),chunks=[];
  for await(const chunk of req)chunks.push(chunk);
  const body=Buffer.concat(chunks).toString(),params=new URLSearchParams(body);
  requests.push({path:url.pathname,method:req.method,params});
  let result={ok:true};
  if(req.method==='POST'){
    if(url.pathname==='/wifi/scan'){
      if(scanMode==='busy'){res.statusCode=409;result={error:'Router connection is in progress. Try scanning again shortly.'};}
      else{scanPolls=0;scanStarted=true;result={state:'scanning',running:true,networks:[]};}
    }
    if(url.pathname==='/wifi/password'){
      result={password:params.get('target')==='ap'?'POV123456':savedPassword};
      if(revealDelay)await new Promise(resolve=>setTimeout(resolve,revealDelay));
    }
    if(url.pathname==='/wifi'){
      if(params.get('forget')==='1'){wifi.ssid='';savedPassword='';}
      else{
        wifi.ssid=params.get('ssid');wifi.station=params.get('station');
        if(params.has('pass'))savedPassword=params.get('pass');
      }
      wifi.passwordSaved=!!savedPassword;
    }
  }else if(url.pathname==='/status')result={wifi,firmwareBuild:'Sep 17 2026 13:00:00',firmwareVersion:'1.2',firmwareBuildNumber:27};
  else if(url.pathname==='/wifi/scan')result=scanResult();
  else{res.setHeader('Content-Type','text/html; charset=utf-8');res.end(url.pathname.startsWith('/wifi')?html:root);return;}
  res.setHeader('Content-Type','application/json');res.setHeader('Cache-Control','no-store');res.end(JSON.stringify(result));
});
(async()=>{
  await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
  const browser=await chromium.launch({channel:'msedge',headless:true});
  const page=await browser.newPage({viewport:{width:1280,height:900}});
  const errors=[];page.on('pageerror',e=>errors.push(e.message));
  const latest=path=>requests.filter(r=>r.path===path&&r.method==='POST').at(-1);
  const reveals=()=>requests.filter(r=>r.path==='/wifi/password').length;
  async function save(){
    const done=page.waitForResponse(r=>r.url().endsWith('/wifi')&&r.request().method()==='POST');
    await page.getByRole('button',{name:'Save and connect',exact:true}).click();await done;
    await page.waitForFunction(()=>document.getElementById('password').value==='');
  }
  async function scan(){await page.locator('#scan').click();await page.waitForFunction(()=>document.getElementById('scan').disabled);await page.waitForFunction(()=>!document.getElementById('scan').disabled);}
  try{
    await page.goto(`http://127.0.0.1:${server.address().port}/wifi`);
    await page.waitForFunction(()=>document.getElementById('headerbuild').textContent==='Ver1.2 \u00b7 Build 27');
    assert((await page.getByRole('heading',{level:1}).textContent()).startsWith('LPOVXLM Spinner'));
    await page.waitForFunction(()=>document.getElementById('ssid').value==='Workshop');
    assert.equal(await page.title(),'Wi-Fi setup · LPOVXLM');
    assert.equal(await page.locator('input[type=password]').count(),2);
    assert.equal(await page.locator('#password').inputValue(),'');
    assert.equal(await page.locator('#appassword').inputValue(),'');
    assert.equal(reveals(),0);
    await page.getByRole('button',{name:'Show router password',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('password').type==='text');
    assert.equal(await page.locator('#password').inputValue(),savedPassword);
    assert.equal(latest('/wifi/password').params.get('target'),'router');
    await page.getByRole('button',{name:'Hide router password',exact:true}).click();
    assert.equal(await page.locator('#password').getAttribute('type'),'password');
    await page.getByRole('button',{name:'Show access point password',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('appassword').type==='text');
    assert.equal(await page.locator('#appassword').inputValue(),'POV123456');
    await page.getByRole('button',{name:'Hide access point password',exact:true}).click();
    assert.equal(await page.locator('#appassword').getAttribute('type'),'password');
    await page.locator('#password').fill('');await save();
    assert(!latest('/wifi').params.has('pass')); // Blank preserves the current saved password.
    await scan();
    assert.equal(await page.locator('#networks tr').count(),3);
    assert.equal(await page.locator('#networks img').count(),0);
    assert((await page.locator('#networks').textContent()).includes('<img src=x'));
    assert((await page.locator('#networks').textContent()).includes('-41 dBm'));
    const networkNames=()=>page.locator('#networks button').allTextContents();
    assert.equal((await networkNames())[0],'Workshop East');
    await page.locator('#networksort').selectOption('name-asc');
    assert.deepEqual(await networkNames(),networks.map(n=>n.ssid).sort((a,b)=>a.localeCompare(b,undefined,{numeric:true,sensitivity:'base'})));
    await page.locator('#networksort').selectOption('name-desc');
    assert.equal((await networkNames())[0],'Workshop East');
    await page.locator('#networksort').selectOption('signal');
    revealDelay=1000;
    await page.getByRole('button',{name:'Show router password',exact:true}).click();
    await page.getByRole('button',{name:'Workshop East',exact:true}).click();
    await page.waitForFunction(()=>!document.getElementById('showrouter').disabled);
    assert.equal(await page.locator('#password').inputValue(),''); // Ignore an old network's late reveal response.
    assert.equal(await page.locator('#password').getAttribute('type'),'password');
    revealDelay=0;
    await page.getByRole('button',{name:'Workshop East',exact:true}).click();
    assert.equal(await page.locator('#ssid').inputValue(),'Workshop East');
    const before=reveals();
    await page.locator('#password').fill('Typed-password!');
    await page.getByRole('button',{name:'Show router password',exact:true}).click();
    assert.equal(await page.locator('#password').getAttribute('type'),'text');
    assert.equal(reveals(),before); // Typed text is revealed locally, without fetching another network's secret.
    await save();
    assert.equal(latest('/wifi').params.get('ssid'),'Workshop East');
    assert.equal(latest('/wifi').params.get('pass'),'Typed-password!');
    assert.equal(await page.locator('#password').getAttribute('type'),'password');
    await page.getByRole('button',{name:'Guest',exact:true}).click();
    assert(await page.locator('#open').isChecked());
    assert(await page.locator('#password').isDisabled());
    await save();
    assert.equal(latest('/wifi').params.get('pass'),''); // Explicitly clear credentials for an open network.
    await page.locator('#ssid').fill('Hidden Workshop');
    await page.locator('#open').uncheck();
    await page.locator('#password').fill('Hidden-password!');await save();
    assert.equal(latest('/wifi').params.get('ssid'),'Hidden Workshop');
    await page.getByRole('button',{name:'Retry connection',exact:true}).click();
    await page.waitForFunction(()=>document.getElementById('notice').textContent==='Router connection retry requested.');
    assert(latest('/wifi/retry'));
    scanMode='empty';await scan();
    assert((await page.locator('#scanstatus').textContent()).includes('No networks found'));
    scanMode='failed';await scan();
    assert((await page.locator('#scanstatus').textContent()).includes('ESP_ERR_WIFI_TIMEOUT'));
    scanMode='busy';await page.locator('#scan').click();
    await page.waitForFunction(()=>document.getElementById('notice').textContent.includes('Router connection is in progress'));
    assert(await page.locator('#scan').isEnabled());
    scanMode='normal';await scan();
    await page.locator('#forget').click();
    await page.waitForFunction(()=>document.getElementById('ssid').value==='');
    assert.equal(latest('/wifi').params.get('forget'),'1');
    await page.evaluate(()=>scrollTo(0,0));
    await page.screenshot({path:'build/wifi-setup-desktop.png',fullPage:true});
    await page.setViewportSize({width:390,height:844});
    await page.screenshot({path:'build/wifi-setup-mobile.png',fullPage:true});
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
    await page.setViewportSize({width:320,height:640});
    assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth));
    assert.deepEqual(errors,[]);
    console.log('Wi-Fi setup tests passed: scan/selection, empty/error states, SSID escaping, typed/saved/AP password toggles, open/manual networks, retry/forget, and mobile layout.');
  }finally{await browser.close();await new Promise(resolve=>server.close(resolve));}
})().catch(e=>{console.error(e);server.close();process.exitCode=1;});

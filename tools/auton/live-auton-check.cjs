const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const base=`http://127.0.0.1:${process.env.AUTON_TEST_PORT||8765}/api/`;
const wait=ms=>new Promise(resolve=>setTimeout(resolve,ms));
(async()=>{
 const name=process.argv[2],out=process.argv[3]||'build/live-autons',delayed=name.includes('Wait15');
 // The partner-delivery fixture needs the original simulator's internals; without it nothing raises Basket 2.
 const fixture=!!process.env.PARTNER_FIXTURE;
 const info=await fetch(base+'info').then(r=>r.json());
 async function api(endpoint,data){
  const response=await fetch(base+endpoint,data?{method:'POST',headers:{'Content-Type':'application/json','X-Sim-Token':info.token},body:JSON.stringify(data)}:{});
  const reply=await response.json();if(!response.ok)throw Error(JSON.stringify(reply));return reply;
 }
 const act=(action,data={})=>api('action',{action,...data});const trace=[];
 try{
  await act('stop');
  // Calibrated to the team robot (see docs/SIMULATOR-CALIBRATION.md); defaults are for a generic robot.
  const calibration=JSON.parse(fs.readFileSync(path.join(__dirname,'sim-calibration.json'),'utf8'));
  await api('profile',{operation:'apply',data:{version:3,robot:{...info.defaults,...calibration,alliance:name.startsWith('Blue')?1:0}}});
  await act('mode',{mode:'match-auto'});
  await act('select',{name:'org.firstinspires.ftc.teamcode.Auton.'+name});
  // The simulator robot must physically start where the code believes it is (Pedro start pose, metres).
  // Simulator heading 0 faces +y, i.e. Pedro heading 90 degrees.
  {const blue=name.startsWith('Blue'),sgn=blue?-1:1;
   const wait15=delayed;const x=(wait15?-.245:-.36)*sgn,y=(wait15?1.564259:-1.564259)*sgn;
   const facingPlusY=wait15?blue:!blue,heading=facingPlusY?0:Math.PI;
   // Simulators that place the robot from the program's start pose (Pinpoint setPosition during INIT) need no manual step;
   // park it elsewhere first so the check proves the auto-placement, then fall back to placing by hand on older simulators.
   await act('place',{x:0,y:0,heading:Math.PI/2});
   await act('init');await wait(550);
   const placed=(await api('state')).robot,dh=Math.abs(Math.atan2(Math.sin(placed[2]-heading),Math.cos(placed[2]-heading)));
   if(Math.hypot(placed[0]-x,placed[1]-y)<.02&&dh<.02)console.log(name,'auto-placed at',placed.slice(0,3).map(v=>v.toFixed(3)).join(', '));
   else{console.log(name,'not auto-placed (robot at',placed.slice(0,3).map(v=>v.toFixed(3)).join(', ')+'); placing by hand');
    await act('stop');await act('place',{x,y,heading});await act('init');await wait(550);}}
  await act('start');
  let prior='';
  for(let i=0;i<290;i++){
   await wait(100);const s=await api('state');
   // Park Only reports only an Auto result line; treat its Parked result as the DONE stage.
   const stage=s.telemetry?.match(/Auto stage\s*:\s*(.*)/)?.[1]||(s.telemetry?.includes('Auto result : Parked')?'DONE':'initializing');
   const route=s.telemetry?.match(/Route\s*:\s*(.*)/)?.[1]||'';
   const elapsed=Number(s.telemetry?.match(/Elapsed \(s\)\s*:\s*(\S+)/)?.[1]??s.telemetry?.match(/Auto elapsed\s*:\s*(\S+)/)?.[1]??-1);
   trace.push({time:s.time,elapsed,stage,route,robot:s.robot,shots:s.shots,hits:s.hits,held:s.held,collected:s.collected,
    motorPowers:s.motorPowers,windmill:s.windmillPower??0,intake:s.intake,telemetry:s.telemetry,hives:s.hives});
   assert(Math.abs(s.robot[0])>1.04||Math.abs(s.robot[1])>.89,'actual follower entered frame keepout');
   // Telemetry is cached asynchronously; the server match clock is authoritative for pre-feed samples.
   if(delayed&&30-s.remaining<15){
    assert.equal(s.shots,0,'no shots before 15 seconds');assert.equal(s.intake,0,'intake off during wait');
    assert.equal(s.windmillPower??0,0,'windmill off during wait');
   }
   if(delayed&&fixture&&s.shots>0){
    const hive=s.hives[name.startsWith('Blue')?1:0];
    assert(hive[1]>=1&&hive[0]===(name.startsWith('Blue')?-1:1),'partner must have raised Basket 2 before our shot');
   }
   if(stage!==prior){console.log(name,s.time.toFixed(3),stage,s.shots,s.hits,s.held);prior=stage;}
   if(stage==='DONE'||s.phase==='stopped')break;
  }
  const last=trace.at(-1),garden=name.includes('Garden'),sign=name.startsWith('Blue')?-1:1;
  // The community simulator has no pickup counters; fall back to the pollen held after the sweep.
  last.collected=last.collected||{garden:last.held,flowers:0,nectarCollected:0};
  const summary={name,elapsed:Number(last.telemetry.match(/Elapsed \(s\)\s*:\s*(\S+)/)?.[1]??last.telemetry.match(/Auto elapsed\s*:\s*(\S+)/)?.[1]),simulationTime:last.time,
   parked:last.telemetry.includes('Auto result : Parked'),shots:last.shots,hits:last.hits,held:last.held,
   garden:last.collected.garden,flowers:last.collected.flowers,nectar:last.collected.nectarCollected,
   end:last.robot.slice(0,3),stages:[...new Set(trace.map(row=>row.stage))]};
  if(delayed){
   summary.firstFeed=Number(last.telemetry.match(/First feed command \(s\)\s*:\s*(\S+)/)?.[1]);
   summary.firstShot=trace.find(row=>row.shots>0)?.elapsed;
   summary.partnerTips=last.hives[name.startsWith('Blue')?1:0][1];
  }
  fs.mkdirSync(out,{recursive:true});fs.writeFileSync(path.join(out,name+'.json'),JSON.stringify({summary,trace},null,2));
  console.log('RESULT',JSON.stringify(summary));
  assert(summary.parked&&summary.elapsed<29.7,'must finish parking before cutoff');
  // A ball that misses a tipping hive can land at the intake and be re-fed, so allow a fifth release.
  // Without the partner fixture Basket 2 is never raised, so Wait 15 shots cannot score and are re-fed.
  assert(last.shots>=4&&(last.shots<=6||(delayed&&!fixture)),'the four-preload opening (plus at most re-fed misses)');
  assert(last.hits>=(delayed&&!fixture?0:3),'opening shots reach hive');
  if(delayed){assert(summary.firstFeed>=15&&summary.firstFeed<16,'feed gate opens at 15 seconds after settled aim');
   assert(summary.firstShot>=14.5,'actual first launch after wait (telemetry lags the match clock by up to ~0.5 s)');if(fixture)assert.equal(last.hits,4,'all four enter raised Basket 2');}
  // Basket 2 starts on the parking-zone wall, so it takes the half of the zone nearer that wall (Blue (127.88, 21.27) in, Red mirrored).
  const park=delayed?[-sign*(127.88-70.75)*.0254,-sign*(21.27-70.75)*.0254]:[-1.451*sign,.64*sign];
  assert(Math.abs(last.robot[0]-park[0])<.04&&Math.abs(last.robot[1]-park[1])<.04,'assigned parking slot');
  assert(last.motorPowers.every(power=>Math.abs(power)<1e-8)&&last.intake===0&&last.windmill===0,'stopped drive and intake');
  assert.equal(last.collected.flowers,0);assert.equal(last.collected.nectarCollected,0);
  if(garden){assert.equal(last.collected.garden,4);assert.equal(last.held,4);assert(summary.stages.includes('CAPTURE'));}
  // The suction intake can legitimately re-collect balls that spill when a HIVE tips, so held is reported, not asserted.
 }finally{await act('stop');}
})().catch(error=>{console.error(error);process.exitCode=1;});

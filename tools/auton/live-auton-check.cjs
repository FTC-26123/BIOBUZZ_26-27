const assert=require('node:assert/strict');
const fs=require('node:fs');
const path=require('node:path');
const base=`http://127.0.0.1:${process.env.AUTON_TEST_PORT||8777}/api/`;
const wait=ms=>new Promise(resolve=>setTimeout(resolve,ms));
(async()=>{
 const name=process.argv[2],out=process.argv[3]||'build/live-autons';
 const info=await fetch(base+'info').then(r=>r.json());
 async function api(endpoint,data){
  const response=await fetch(base+endpoint,data?{method:'POST',headers:{'Content-Type':'application/json','X-Sim-Token':info.token},body:JSON.stringify(data)}:{});
  const reply=await response.json();if(!response.ok)throw Error(JSON.stringify(reply));return reply;
 }
 const act=(action,data={})=>api('action',{action,...data});const trace=[];
 try{
  await act('stop');
  await api('profile',{operation:'apply',data:{version:3,robot:{...info.defaults,alliance:name.startsWith('Blue')?1:0,webcamEnabled:0}}});
  await act('mode',{mode:'match-auto'});
  await act('select',{name:'org.firstinspires.ftc.teamcode.Auton.'+name});
  await act('init');await wait(550);await act('start');
  let prior='';
  for(let i=0;i<290;i++){
   await wait(100);const s=await api('state');
   const stage=s.telemetry?.match(/Auto stage\s*:\s*(.*)/)?.[1]||'initializing';
   const route=s.telemetry?.match(/Route\s*:\s*(.*)/)?.[1]||'';
   trace.push({time:s.time,stage,route,robot:s.robot,shots:s.shots,hits:s.hits,held:s.held,collected:s.collected,
    motorPowers:s.motorPowers,windmill:s.windmillPower,intake:s.intake,telemetry:s.telemetry,hives:s.hives});
   assert(Math.abs(s.robot[0])>1.04||Math.abs(s.robot[1])>.89,'actual follower entered frame keepout');
   if(stage!==prior){console.log(name,s.time.toFixed(3),stage,s.shots,s.hits,s.held);prior=stage;}
   if(stage==='DONE'||s.phase==='stopped')break;
  }
  const last=trace.at(-1),garden=name.includes('Garden'),sign=name.startsWith('Blue')?-1:1;
  const summary={name,elapsed:Number(last.telemetry.match(/Elapsed \(s\)\s*:\s*(\S+)/)?.[1]),simulationTime:last.time,
   parked:last.telemetry.includes('Auto result : Parked'),shots:last.shots,hits:last.hits,held:last.held,
   garden:last.collected.garden,flowers:last.collected.flowers,nectar:last.collected.nectarCollected,
   end:last.robot.slice(0,3),stages:[...new Set(trace.map(row=>row.stage))]};
  fs.mkdirSync(out,{recursive:true});fs.writeFileSync(path.join(out,name+'.json'),JSON.stringify({summary,trace},null,2));
  console.log('RESULT',JSON.stringify(summary));
  assert(summary.parked&&summary.elapsed<29.7,'must finish parking before cutoff');
  assert.equal(last.shots,4,'exactly the four-preload opening');
  assert(last.hits>=3,'opening shots reach hive');
  assert(Math.abs(last.robot[0]+1.55*sign)<.025&&Math.abs(last.robot[1]-.64*sign)<.025,'assigned parking slot');
  assert(last.motorPowers.every(power=>Math.abs(power)<1e-8)&&last.intake===0&&last.windmill===0,'stopped drive and intake');
  assert.equal(last.collected.flowers,0);assert.equal(last.collected.nectarCollected,0);
  if(garden){assert.equal(last.collected.garden,4);assert.equal(last.held,4);assert(summary.stages.includes('CAPTURE'));}
  else assert.equal(last.held,0);
 }finally{await act('stop');}
})().catch(error=>{console.error(error);process.exitCode=1;});

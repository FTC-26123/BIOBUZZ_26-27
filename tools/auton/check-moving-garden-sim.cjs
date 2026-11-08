const fs=require('node:fs'),path=require('node:path');
const base=`http://127.0.0.1:${process.env.AUTON_TEST_PORT||8781}/api/`;
const wait=ms=>new Promise(r=>setTimeout(r,ms));
(async()=>{
 const info=await fetch(base+'info').then(r=>r.json());
 async function api(endpoint,data){const r=await fetch(base+endpoint,data?{method:'POST',headers:{'Content-Type':'application/json','X-Sim-Token':info.token},body:JSON.stringify(data)}:{});const j=await r.json();if(!r.ok)throw Error(JSON.stringify(j));return j;}
 const act=(action,data={})=>api('action',{action,...data});
 const out=process.env.AUTON_RESULTS||path.join(__dirname,process.env.RESULT_SET||'results');fs.mkdirSync(out,{recursive:true});
 let allPassed=true;
 for(const name of process.argv.slice(2).length?process.argv.slice(2):['RedGardenMovingShots','BlueGardenMovingShots']){
  const trace=[];let trails={},summary;
  try{
   await act('stop');
   const robot={...info.defaults,alliance:name.startsWith('Blue')?1:0,webcamEnabled:0,
    cameraHeight:.254,cameraPitch:43,cameraForward:.1778,cameraRight:0,cameraYaw:0,cameraRoll:0,
    cameraFps:20,visionEngine:1,angleDeg:86};
   await api('profile',{operation:'apply',data:{version:3,robot}});
   await act('mode',{mode:'match-auto'});await act('reset');
   await act('select',{name:'org.firstinspires.ftc.teamcode.Auton.'+name});
   await act('init');await wait(600);await act('start');
   let prior='',priorShots=0,doneAt=-1;
   for(let i=0;i<310;i++){
    await wait(100);const s=await api('state');
    if(s.phase==='stopped'){console.log(name,'server stopped/reset');break;}
    trace.push(s);
    const stage=s.telemetry?.match(/Auto stage\s*:\s*(.*)/)?.[1]||'initializing';
    if(stage!==prior||s.shots!==priorShots){console.log(name,s.time.toFixed(3),stage,'shots',s.shots,'hits',s.hits,'held',s.held);prior=stage;priorShots=s.shots;}
    if(!s.telemetry.includes('Auto result : Running')&&s.telemetry.includes('Auto result :')&&!s.telemetry.includes('Auto result : Parking')){
     if(doneAt<0)doneAt=Date.now();
     if(Date.now()-doneAt>2200)break;
    }
   }
   trails=await api('shot-trails');
   const last=trace.at(-1);if(!last)throw Error('No running samples');
   const stages=[...new Set(trace.map(s=>s.telemetry.match(/Auto stage\s*:\s*(.*)/)?.[1]||''))];
   const launches=(trails.trails||[]).map(t=>({id:t.id,speed:t.launchSpeed,hit:t.hit}));
   summary={name,elapsed:Number(last.telemetry.match(/Auto elapsed\s*:\s*(.*)/)?.[1]),
    result:last.telemetry.match(/Auto result\s*:\s*(.*)/)?.[1],shots:last.shots,hits:last.hits,
    acknowledged:Number(last.telemetry.match(/Confirmed feed events\s*:\s*(.*)/)?.[1]),
    movingShots:last.movingShots,held:last.held,collected:last.collected,hives:last.hives,
    pose:last.robot.slice(0,3),stages,launches,
    preloadMovingShots:trace.filter((s,i)=>i&&s.shots>trace[i-1].shots&&s.telemetry.includes('Shoot four preloads')).map(s=>({time:s.time,speed:Math.hypot(s.robot[3],s.robot[4]),position:s.robot.slice(0,2)})),
    minFrameClearance:Math.min(...trace.map(s=>Math.max(Math.abs(s.robot[0])-1.04,Math.abs(s.robot[1])-.89))),
    telemetry:last.telemetry};
   summary.farMovingShots=launches.filter(t=>t.id>4&&t.speed>.05).length;
   summary.stoppedOutputs=last.motorPowers.every(p=>Math.abs(p)<1e-8)&&last.intake===0&&last.windmillPower===0;
   summary.noForbiddenPickups=last.collected.flowers===0&&last.collected.nectarCollected===0;
   summary.passed=summary.result==='Parked'&&summary.shots===8&&summary.hits===8&&summary.collected.garden===4&&summary.preloadMovingShots.length===4
    &&stages.some(s=>s.includes('Strafe across'))&&summary.minFrameClearance>0&&summary.stoppedOutputs&&summary.noForbiddenPickups;
   console.log('RESULT',JSON.stringify(summary));
  }catch(error){summary={name,error:String(error)};console.error(summary);}
  finally{allPassed=allPassed&&summary.passed===true;fs.writeFileSync(path.join(out,name+'.json'),JSON.stringify({summary,trace,trails},null,2));await act('stop');}
 }
 if(!allPassed)process.exitCode=1;
})().catch(e=>{console.error(e);process.exitCode=1;});


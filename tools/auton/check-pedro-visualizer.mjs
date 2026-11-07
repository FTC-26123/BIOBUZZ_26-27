import {registerHooks} from 'node:module';
import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {pathToFileURL} from 'node:url';
registerHooks({resolve(specifier,context,next){
  if(specifier.startsWith('.')&&!/\.[a-z]+$/i.test(specifier))return next(new URL(specifier+'.ts',context.parentURL).href,context);
  return next(specifier,context);
}});
const reference=resolve(process.argv[2]||'build/pedro-visualizer-reference');
const {findCollisions}=await import(pathToFileURL(reference+'/src/model/collision.ts'));
const {calculatePathTime}=await import(pathToFileURL(reference+'/src/utils/timeCalculator.ts'));
for(const name of process.argv.length>3?process.argv.slice(3):['RedGardenPark','BlueGardenPark']){
 const doc=JSON.parse(readFileSync('docs/pedro/'+name+'.pp.json','utf8'));
 const prediction=calculatePathTime(doc.startPoint,doc.lines,doc.robot,doc.sequence);
 const collisions=findCollisions(doc,prediction.timeline,{step:.20,angleStep:1});
 console.log(name,'official Pedro visualizer engine:',doc.lines.length,'segments,',collisions.length,'collision intervals');
 if(collisions.length){console.log(JSON.stringify(collisions));process.exitCode=1;}
 const bad=structuredClone(doc);bad.lines=[{kind:'atomic',id:'negative-control',color:'#ff0000',endPoint:{x:70.75,y:70.75},controlPoints:[],heading:{type:'constant',degrees:0}}];bad.sequence=[{kind:'path',lineId:'negative-control'}];
 const negative=findCollisions(bad,calculatePathTime(bad.startPoint,bad.lines,bad.robot,bad.sequence).timeline,{step:.2,angleStep:1});
 if(!negative.some(c=>c.obstacle.kind==='shape'&&c.obstacle.shapeId==='Whole metal assembly'))throw Error('Visualizer negative control did not catch the frame');
}

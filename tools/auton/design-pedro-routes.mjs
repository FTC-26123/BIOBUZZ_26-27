// Route design search for the eight autons, scored with the official Pedro Visualizer engine.
// Usage: node tools/auton/design-pedro-routes.mjs [build/pedro-visualizer-reference]
// Prints the shortest single-Bezier legs that keep: the robot centre >= CENTER_MARGIN outside the
// runtime FrameClearance box, the 18-inch footprint >= FOOT_MARGIN from walls, pots and the
// reserved teammate slot, and the whole footprint on our alliance's half of the field.
import {registerHooks} from 'node:module';
import {resolve} from 'node:path';
import {pathToFileURL} from 'node:url';
registerHooks({resolve(specifier,context,next){
  if(specifier.startsWith('.')&&!/\.[a-z]+$/i.test(specifier))return next(new URL(specifier+'.ts',context.parentURL).href,context);
  return next(specifier,context);
}});
const reference=resolve(process.argv[2]||'build/pedro-visualizer-reference');
const {findCollisions}=await import(pathToFileURL(reference+'/src/model/collision.ts'));
const {calculatePathTime}=await import(pathToFileURL(reference+'/src/utils/timeCalculator.ts'));

const C=70.75,IN=1/.0254,FIELD=141.5;
const KEEP={x0:C-(.67+.34+.03)*IN,x1:C+(.67+.34+.03)*IN,y0:C-(.52+.34+.03)*IN,y1:C+(.52+.34+.03)*IN};
const CENTER_MARGIN=2.0,FOOT_MARGIN=1.0,POT_MARGIN=2.0,TEAMMATE_MARGIN=3.0,MIN_RADIUS=12;
const POT={x:-1.7283*IN+C,y:-.5942*IN+C,r:.08*IN};
const TEAM={x:-1.55*IN+C,y:1.19*IN+C,half:9};
const bez=(P,t)=>{let pts=P.map(p=>[p[0],p[1]]);while(pts.length>1){const n=[];for(let i=1;i<pts.length;i++)n.push([pts[i-1][0]*(1-t)+pts[i][0]*t,pts[i-1][1]*(1-t)+pts[i][1]*t]);pts=n;}return pts[0];};
function samples(P,n=400){const out=[];let len=0,prev=null;for(let i=0;i<=n;i++){const p=bez(P,i/n);if(prev)len+=Math.hypot(p[0]-prev[0],p[1]-prev[1]);out.push({p,s:len});prev=p;}out.forEach(o=>o.f=len?o.s/len:1);return{out,len};}
const shortest=(a,b,f)=>{let d=((b-a)%360+540)%360-180;return a+d*f;};
function corners(x,y,h){const r=h*Math.PI/180,c=Math.cos(r),s=Math.sin(r);return[[9,9],[9,-9],[-9,-9],[-9,9]].map(([u,v])=>[x+u*c-v*s,y+u*s+v*c]);}
function segDist(px,py,ax,ay,bx,by){const dx=bx-ax,dy=by-ay,t=Math.max(0,Math.min(1,((px-ax)*dx+(py-ay)*dy)/(dx*dx+dy*dy)));return Math.hypot(px-ax-t*dx,py-ay-t*dy);}
function polyCircle(poly,cx,cy){let inside=false;for(let i=0,j=3;i<4;j=i++){const[a,b]=poly[i],[c,d]=poly[j];if((b>cy)!==(d>cy)&&cx<(c-a)*(cy-b)/(d-b)+a)inside=!inside;}if(inside)return-1;let m=1e9;for(let i=0;i<4;i++){const[a,b]=poly[i],[c,d]=poly[(i+1)%4];m=Math.min(m,segDist(cx,cy,a,b,c,d));}return m;}
/** Worst-case margins of one leg; heading(f) gives degrees at arc fraction f. */
export function margins(P,heading,{teammate=TEAM,checkTeammate=true}={}){
  const {out,len}=samples(P);let center=1e9,wall=1e9,pot=1e9,team=1e9,half=1e9;
  for(const{p:[x,y],f}of out){
    const h=heading(f),poly=corners(x,y,h);
    const dx=Math.max(KEEP.x0-x,x-KEEP.x1),dy=Math.max(KEEP.y0-y,y-KEEP.y1);center=Math.min(center,Math.max(dx,dy));
    for(const[a,b]of poly){wall=Math.min(wall,a,b,FIELD-a,FIELD-b);half=Math.min(half,C-a);}
    pot=Math.min(pot,polyCircle(poly,POT.x,POT.y)-POT.r);
    if(checkTeammate){const xs=poly.map(q=>q[0]),ys=poly.map(q=>q[1]);
      team=Math.min(team,Math.max(Math.min(...xs)-(teammate.x+teammate.half),(teammate.x-teammate.half)-Math.max(...xs),Math.min(...ys)-(teammate.y+teammate.half),(teammate.y-teammate.half)-Math.max(...ys)));}
  }
  let radius=1e9;for(let i=1;i<out.length-1;i++){const[a,b,c]=[out[i-1].p,out[i].p,out[i+1].p];
    const cross=(b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]),ab=Math.hypot(b[0]-a[0],b[1]-a[1]),bc=Math.hypot(c[0]-b[0],c[1]-b[1]),ac=Math.hypot(c[0]-a[0],c[1]-a[1]);
    if(ab*bc*ac>1e-12&&Math.abs(cross)>1e-12)radius=Math.min(radius,ab*bc*ac/(2*Math.abs(cross)));}
  return{len,center,wall,pot,team,half,radius};
}
const ok=(m,startNearWall=false)=>m.center>=CENTER_MARGIN&&(startNearWall||m.wall>=FOOT_MARGIN)&&m.pot>=FOOT_MARGIN&&m.team>=TEAMMATE_MARGIN&&m.half>=0;

/** Curvature-limited trapezoid travel time (s): forward/backward passes, vmax/accel from the robot block. */
const VMAX=(74.58+60.78)/2,ACC=50,LAT=50;
export function travelTime(P){const{out}=samples(P,300);const n=out.length,v=new Array(n).fill(VMAX);
  for(let i=1;i<n-1;i++){const[a,b,c]=[out[i-1].p,out[i].p,out[i+1].p];const cross=Math.abs((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]));
    const ab=Math.hypot(b[0]-a[0],b[1]-a[1]),bc=Math.hypot(c[0]-b[0],c[1]-b[1]),ac=Math.hypot(c[0]-a[0],c[1]-a[1]);
    if(cross>1e-12)v[i]=Math.min(VMAX,Math.sqrt(LAT*ab*bc*ac/(2*cross)));}
  v[0]=0;v[n-1]=0;for(let i=1;i<n;i++)v[i]=Math.min(v[i],Math.sqrt(v[i-1]**2+2*ACC*(out[i].s-out[i-1].s)));
  for(let i=n-2;i>=0;i--)v[i]=Math.min(v[i],Math.sqrt(v[i+1]**2+2*ACC*(out[i+1].s-out[i].s)));
  let t=0;for(let i=1;i<n;i++){const ds=out[i].s-out[i-1].s,vv=(v[i]+v[i-1])/2;t+=vv>1e-9?ds/vv:0;}return t;}
/** Random-restart local search for the shortest feasible cubic between fixed ends. */
function search(name,A,B,heading,{seed=[],startWall=false,lockFirst=null,iters=15000}={}){
  let best=null,rand=mulberry(42);
  const score=c=>{const P=[A,...c,B];const m=margins(P,heading);
    const wallOk=startWall?wallAfterStart(P,heading):m.wall>=FOOT_MARGIN;
    if(!(m.center>=CENTER_MARGIN&&wallOk&&m.pot>=POT_MARGIN&&m.team>=TEAMMATE_MARGIN&&m.half>=0&&m.radius>=MIN_RADIUS))return null;
    return{c,m,t:travelTime(P)};};
  const tries=seed.length?seed:[[[A[0],A[1]+10],[B[0]+10,B[1]-10]]];
  for(const s of tries){const r=score(s);if(r&&(!best||r.t<best.t))best=r;}
  for(let i=0;i<iters;i++){
    const base=best&&rand()<.7?best.c:[[rnd(rand,10,70),rnd(rand,10,130)],[rnd(rand,10,70),rnd(rand,10,130)]];
    const k=best?Math.max(.05,8*(1-i/iters)):30;
    const c=base.map(p=>[p[0]+(rand()-.5)*k,p[1]+(rand()-.5)*k]);
    if(lockFirst)c[0]=lockFirst(c[0]);
    const r=score(c);if(r&&(!best||r.t<best.t))best=r;
  }
  if(!best)throw Error('No feasible '+name);
  best.c=best.c.map(p=>[+p[0].toFixed(2),+p[1].toFixed(2)]);
  return{name,A,B,controls:best.c,m:margins([A,...best.c,B],heading)};
}
/** Starting against a wall: move away from it at least half as fast as the robot travels until FOOT_MARGIN. */
function wallAfterStart(P,heading){const{out}=samples(P);const w=(x,y,h)=>Math.min(...corners(x,y,h).map(([a,b])=>Math.min(a,b,FIELD-a,FIELD-b)));
  const w0=w(P[0][0],P[0][1],heading(0));for(const{p:[x,y],f}of out){const d=Math.hypot(x-P[0][0],y-P[0][1]),c=w(x,y,heading(f));
    if(c<Math.min(w0+.8*d,FOOT_MARGIN+.5)-.05)return false;}return true;}
function mulberry(a){return()=>{a|=0;a=a+0x6D2B79F5|0;let t=Math.imul(a^a>>>15,1|a);t=t+Math.imul(t^t>>>7,61|t)^t;return((t^t>>>14)>>>0)/4294967296;};}
const rnd=(r,a,b)=>a+(b-a)*r();
const lin=(a,b,end=1)=>f=>shortest(a,b,Math.min(1,f/end));

// ---- Red anchor poses (Pedro visualizer inches). Blue is the 180-degree rotation. ----
const S=[-.36*IN+C,-1.564259*IN+C],SH=90;
const aim=Math.atan2(-.392176-((28.48-70.75)*.0254),-.32385-((57.05-70.75)*.0254));
const K=[(57.05-70.75)*.0254*IN+C-Math.cos(aim),(28.48-70.75)*.0254*IN+C-Math.sin(aim)],KH=aim*180/Math.PI;
const P=[-1.451*IN+C,.64*IN+C];
const GA=[-1.15*IN+C,-1.48*IN+C],GS=[-1.50*IN+C,-1.48*IN+C],GH=255;
const S2=[-.245*IN+C,1.564259*IN+C],K2=[K[0],2*C-K[1]],K2H=360-KH,P2=[127.88-2*(127.88-C),2*C-21.27];

const legs=[];
legs.push(search('Shoot4: shot -> park',K,P,lin(KH,90),{seed:[[[30,28],[15,50]],[[20,26],[18,45]],[[35,25],[10,40]]]}));
legs.push(search('ParkOnly: start -> park',S,P,lin(SH,90),{seed:[[[50,25],[10,30]],[[45,28],[15,35]],[[56,20],[5,25]],[[40,22],[18,40]]],startWall:true}));
legs.push(search('Garden: sweep end -> park',GS,P,lin(GH,270),{seed:[[[20,30],[22,70]],[[18,30],[25,50]],[[22,25],[24,60]]],startWall:true}));
const out={anchors:{S,SH,K,KH,P,GA,GS,GH,S2,K2,K2H,P2},legs};
// Straight legs: verify margins too.
const straight=[['Shoot4: start -> shot',S,K,lin(SH,KH),true],['Garden: shot -> approach (turn done by 60%)',K,GA,lin(KH,GH,.6),false],
  ['Garden: sweep',GA,GS,()=>GH,false],['Basket2: start -> shot',S2,K2,lin(270,K2H),true],['Basket2: shot -> park',K2,P2,lin(K2H,270),false]];
for(const[n,a,b,h,w]of straight){const m=margins([a,b],h,{teammate:n.startsWith('Basket2')?{x:TEAM.x,y:2*C-TEAM.y,half:9}:TEAM});out.legs.push({name:n,A:a,B:b,controls:[],m,wallExempt:w});}
for(const l of out.legs)console.log(l.name.padEnd(46),'t',travelTime([l.A,...l.controls,l.B]).toFixed(2),'len',l.m.len.toFixed(1).padStart(6),'center',l.m.center.toFixed(2),'wall',l.m.wall.toFixed(2),'pot',l.m.pot.toFixed(2),'team',l.m.team.toFixed(2),'half',l.m.half.toFixed(2),'minR',l.m.radius>1e6?'inf':l.m.radius.toFixed(1),JSON.stringify(l.controls));
console.log(JSON.stringify(out.anchors));

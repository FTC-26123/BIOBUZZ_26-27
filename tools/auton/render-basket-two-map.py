"""Render the autonomous setup guide from the bundled official FIRST STEP tessellation."""
import gzip
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.patches import Polygon
import numpy as np

ROOT = Path("../sim-publish").resolve()
with gzip.open(ROOT / "web/assets/field/field.json.gz") as source:
    model = json.load(source)
assert model["rotation"] == [0, 0, 0] and model["translation"] == [0, 0, 0]
triangles, colors, heights = [], [], []
light = np.array([-.35, .85, .4])
light /= np.linalg.norm(light)
for part in model["parts"]:
    vertices = np.array(part["positions"]).reshape(-1, 3)
    indices = np.array(part["indices"]).reshape(-1, 3)
    mesh = vertices[indices]
    normals = np.array(part["normals"]).reshape(-1, 3)[indices].mean(axis=1)
    # Physics X/Y is CAD X/-Z. Height order gives a true overhead CAD view.
    plan = mesh[:, :, [0, 2]].copy()
    plan[:, :, 1] *= -1
    edge_a, edge_b = plan[:, 1] - plan[:, 0], plan[:, 2] - plan[:, 0]
    visible = np.abs(edge_a[:, 0] * edge_b[:, 1] - edge_a[:, 1] * edge_b[:, 0]) > 1e-10
    shade = .55 + .45 * np.abs(normals @ light)
    rgb = np.array(part["color"]) * shade[:, None]
    # STEP colors are linear RGB, so convert for the exported image.
    rgb = np.where(rgb <= .0031308, 12.92 * rgb, 1.055 * rgb ** (1 / 2.4) - .055)
    triangles.append(plan[visible])
    colors.append(np.clip(rgb[visible], 0, 1))
    heights.append(mesh[:, :, 1].mean(axis=1)[visible])
order = np.argsort(np.concatenate(heights), kind="stable")



import csv
from matplotlib.patches import Rectangle
routes={}
for name,phase,x,y,h in csv.reader(Path('build/basket-two-routes.tsv').read_text().splitlines(),delimiter='\t'):
    routes.setdefault(name,[]).append((phase,float(x)+70.75,float(y)+70.75,float(h)))
fig=plt.figure(figsize=(15,11),facecolor='#f8fafc')
ax=fig.add_axes([.055,.12,.60,.77])
ax.add_collection(PolyCollection(np.concatenate(triangles)[order]/.0254+70.75,
    facecolors=.65*np.concatenate(colors)[order]+.35,edgecolors='none',rasterized=True))
ax.set(xlim=(0,141.5),ylim=(0,141.5),aspect='equal',xlabel='Pedro visualizer X (inches)',ylabel='Pedro visualizer Y (inches)')
ax.set_xticks(np.arange(0,142,12));ax.set_yticks(np.arange(0,142,12));ax.grid(alpha=.13)
starts={};shots={}
for name,color in [('RedBasket2Wait15Shoot4Park','#c02655'),('BlueBasket2Wait15Shoot4Park','#146ac0')]:
    rows=routes[name];xy=np.array([(r[1],r[2]) for r in rows]);ax.plot(xy[:,0],xy[:,1],color=color,lw=2.6,zorder=5)
    for a,b in zip(xy,xy[1:]):
        if np.linalg.norm(b-a)>3:ax.annotate('',xy=.65*b+.35*a,xytext=.55*b+.45*a,arrowprops=dict(arrowstyle='-|>',color=color,lw=1.5),zorder=7)
    first=rows[0];last=rows[-1];shot=[r for r in rows if r[0].startswith('Aim')][-1]
    starts[name]=first;shots[name]=shot
    ax.add_patch(Rectangle((first[1]-9,first[2]-9),18,18,ec=color,fc=color,alpha=.13,lw=1.5,zorder=4))
    ax.plot(first[1],first[2],'o',ms=12,mfc=color,mec='white',zorder=9)
    ax.plot(shot[1],shot[2],'*',ms=22,mfc=color,mec='white',zorder=9)
    ax.plot(last[1],last[2],'s',ms=16,mfc=color,mec='white',zorder=9)
    ax.text(last[1],last[2],'P',ha='center',va='center',fontsize=10,weight='bold',color='white',zorder=10)
    tx,ty=(shot[1]+10,shot[2]-13) if name.startswith('Red') else (shot[1]-32,shot[2]+12)
    ax.annotate('Aim + wait\nFeed at 15 s',xy=(shot[1],shot[2]),xytext=(tx,ty),fontsize=11,color=color,weight='bold',
        bbox=dict(fc='white',ec='none',alpha=.94,pad=3),arrowprops=dict(arrowstyle='-',color=color),zorder=11)
for x,y in [(-1.55,1.19),(1.55,-.64)]:
    x=x/.0254+70.75;y=y/.0254+70.75
    ax.add_patch(Rectangle((x-9,y-9),18,18,fc='#dccfe9',ec='#775588',ls='--',alpha=.7,zorder=4))
    ax.text(x,y,'ALLY',ha='center',va='center',fontsize=9,color='#624276',weight='bold',zorder=9)
fig.text(.055,.95,'Basket 2: wait 15 seconds, shoot four, park',fontsize=25,weight='bold',color='#0f172a')
fig.text(.055,.917,'Official FIRST BIOBUZZ CAD  |  Opposite-wall starts  |  Teammate must raise Basket 2 before firing',fontsize=12,color='#475569')
key=fig.add_axes([.70,.15,.285,.73]);key.axis('off')
lines=[('COLOR KEY',13,'#0f172a'),('Red Basket 2 Wait 15',14,'#c02655'),('Blue Basket 2 Wait 15',14,'#146ac0'),('START CENTERS (inches)',12,'#0f172a')]
for name in starts:
    _,x,y,h=starts[name];lines.append((f'{"Red" if name.startswith("Red") else "Blue"}: ({x:.3f}, {y:.3f}), {h:.0f} degrees',11,'#334155'))
lines.append(('SHOOTING MARKS (inches)',12,'#0f172a'))
for name in shots:
    _,x,y,h=shots[name];lines.append((f'{"Red" if name.startswith("Red") else "Blue"}: ({x:.4f}, {y:.4f})',11,'#334155'))
lines += [('SEQUENCE',12,'#0f172a'),('1. Drive clear of wall; aim for Basket 2',11,'#334155'),('2. Spin flywheel; all feeders OFF',11,'#334155'),('3. At 15 s + settled aim: feed four',11,'#334155'),('4. Stop mechanisms; drive to marked park',11,'#334155'),('TEAMMATE REQUIRED',12,'#0f172a'),('Basket 2 starts down.',11,'#334155'),('Partner must tip Basket 1 before 15 s.',11,'#334155'),('No camera / basket-state sensor.',11,'#334155'),('DEFAULT SHOOTER',12,'#0f172a'),('1500 ticks/s; same hive distance',11,'#334155'),('as the current Basket 1 mark.',11,'#334155'),('MARKERS',12,'#0f172a'),('Circle: start | Star: aim and wait',11,'#334155'),('P: park | Dashed: teammate space',11,'#334155')]
y=1
for text,size,color in lines:
    key.text(0,y,text,fontsize=size,color=color,weight='bold' if size>=12 else 'normal',va='top',transform=key.transAxes)
    y-=.043 if size<13 else .058
fig.text(.055,.055,'The first 15 seconds include driving, aiming and flywheel spin-up. No intake, transfer or windmill feed is permitted before the delay.',fontsize=10.3,color='#475569')
fig.text(.055,.028,'Field CAD: official FIRST BIOBUZZ STEP v26-27.2, copyright FIRST and original designers. Paths show robot pose origins.',fontsize=9,color='#64748b')
fig.savefig('docs/basket-two-wait15-map.png',dpi=170,facecolor=fig.get_facecolor())

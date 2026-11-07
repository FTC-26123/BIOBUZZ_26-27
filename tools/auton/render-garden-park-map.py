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
for name,phase,x,y,h in csv.reader(Path('build/garden-park-routes.tsv').read_text().splitlines(),delimiter='\t'):
    routes.setdefault(name,[]).append((phase,float(x)+70.75,float(y)+70.75,float(h)))
fig=plt.figure(figsize=(15,11),facecolor='#f8fafc')
ax=fig.add_axes([.055,.12,.60,.77])
plan=np.concatenate(triangles)[order]/.0254+70.75
rgb=.65*np.concatenate(colors)[order]+.35
ax.add_collection(PolyCollection(plan,facecolors=rgb,edgecolors='none',rasterized=True))
ax.set(xlim=(0,141.5),ylim=(0,141.5),aspect='equal',xlabel='Pedro visualizer X (inches)',ylabel='Pedro visualizer Y (inches)')
ax.set_xticks(np.arange(0,142,12));ax.set_yticks(np.arange(0,142,12));ax.grid(alpha=.13)
for name,color in [('RedGardenPark','#c02655'),('BlueGardenPark','#146ac0')]:
    rows=routes[name];xy=np.array([(r[1],r[2]) for r in rows]);ax.plot(xy[:,0],xy[:,1],color=color,lw=2.4,alpha=.9,zorder=5)
    for a,b in zip(xy,xy[1:]):
        if np.linalg.norm(b-a)>4:
            m=a*.4+b*.6;end=a*.35+b*.65
            ax.annotate('',xy=end,xytext=m,arrowprops=dict(arrowstyle='-|>',color=color,lw=1.5),zorder=6)
    first=rows[0];ax.plot(first[1],first[2],'o',ms=12,mfc=color,mec='white',zorder=8)
    last=rows[-1];ax.plot(last[1],last[2],'s',ms=15,mfc=color,mec='white',zorder=8)
    ax.text(last[1],last[2],'P',ha='center',va='center',fontsize=9,color='white',weight='bold',zorder=9)
    stage_ends={row[0]:row for row in rows}
    for phase,x,y,h in stage_ends.values():
        if 'Shoot four' in phase or 'Raised basket' in phase:
            ax.plot(x,y,'*',ms=19,mfc=color,mec='white',zorder=8)
        if 'floor sweep' in phase:
            ax.plot(x,y,'D',ms=9,mfc='#d6ac13',mec='white',zorder=7)
for x,y in [(-1.55,1.19),(1.55,-1.19)]:
    x=x/.0254+70.75;y=y/.0254+70.75
    ax.add_patch(Rectangle((x-9,y-9),18,18,fc='#dccfe9',ec='#775588',lw=1.5,ls='--',alpha=.7,zorder=4))
    ax.text(x,y,'ALLY',ha='center',va='center',fontsize=9,color='#624276',weight='bold',zorder=9)
fig.text(.055,.95,'Shoot four, collect our garden, and park',fontsize=26,weight='bold',color='#0f172a')
fig.text(.055,.917,'Official FIRST BIOBUZZ CAD  |  Own garden only  |  Carry collected pollen into TeleOp',fontsize=12,color='#475569')
key=fig.add_axes([.70,.15,.285,.73]);key.axis('off')
lines=[('COLOR KEY',13,'#0f172a'),('Red Shoot 4 + Garden and Park',14,'#c02655'),('Blue Shoot 4 + Garden and Park',14,'#146ac0'),
('STARTS (visualizer inches)',12,'#0f172a'),('Red: (56.577, 9.165), heading 90',11,'#334155'),('Blue: (84.923, 132.335), heading 270',11,'#334155'),
('SEQUENCE',12,'#0f172a'),('1. Same four-preload opening',12,'#334155'),('2. Sweep near garden (four pollen)',12,'#334155'),('3. Retreat clear of the fixed flower pot',12,'#334155'),('4. Drive outside the metal frame',12,'#334155'),('5. Park with garden pollen for TeleOp',12,'#334155'),
('MARKERS',12,'#0f172a'),('Circle: start | Star: shot position',11,'#334155'),('Gold diamond: end of garden sweep',11,'#334155'),('P: park | Dashed box: ally space',11,'#334155'),
('NO FLOWERS / NO CAMERAS',12,'#0f172a'),('Shooter class default: 1500 ticks/s',11,'#334155'),('No opposite-garden visit or second shot.',11,'#334155'),('Existing Intake subsystem, power 0.80.',11,'#334155'),('Original Shoot 4 and Park stays available.',11,'#334155')]
y=1
for text,size,color in lines:
    key.text(0,y,text,fontsize=size,color=color,weight='bold' if size>=12 else 'normal',va='top',transform=key.transAxes)
    y-=.048 if size<13 else .062
fig.text(.055,.055,'Paths show robot centers; two alternate programs are overlaid. Both use the near parking slot and reserve the farther slot for an ally.',fontsize=10.3,color='#475569')
fig.text(.055,.028,'Field CAD: official FIRST BIOBUZZ STEP v26-27.2, copyright FIRST and original designers. Route overlays are generated from robot route data.',fontsize=9,color='#64748b')
fig.savefig('docs/garden-park-map.png',dpi=170,facecolor=fig.get_facecolor())

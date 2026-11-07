"""Export the production route TSV into Pedro Visualizer 2.0 projects.
Run run-garden-park-sim.ps1 first; it exports the actual Java route data.
"""
import csv
import json
import math
import sys
from pathlib import Path

routes = {}
for name, phase, x, y, heading in csv.reader(Path(sys.argv[1] if len(sys.argv)>1 else 'build/garden-park-routes.tsv').read_text().splitlines(), delimiter='\t'):
    routes.setdefault(name, []).append((phase, float(x) + 70.75, float(y) + 70.75, float(heading)))
output = Path('docs/pedro')
output.mkdir(exist_ok=True)
for name, rows in routes.items():
    lines, sequence, shapes = [], [], []
    previous_heading = rows[0][3]
    for index, (phase, x, y, heading) in enumerate(rows[1:]):
        ident = f'{name}-{index}'
        lines.append(dict(kind='atomic', id=ident, name=phase, endPoint=dict(x=x, y=y), controlPoints=[],
            heading=dict(type='linear', startDeg=previous_heading, endDeg=heading),
            color='#e15759' if name.startswith('Red') else '#4e79a7', waitBeforeMs=0, waitAfterMs=0))
        sequence.append(dict(kind='path', lineId=ident))
        previous_heading = heading
    def shape(label, points, color):
        shapes.append(dict(id=label, name=label, vertices=[dict(x=x/.0254+70.75, y=y/.0254+70.75) for x, y in points],
            color=color, fillColor=color+'25'))
    shape('Whole metal assembly', [(-.67,-.52),(.67,-.52),(.67,.52),(-.67,.52)], '#dc2626')
    for index, (x, y) in enumerate([(-.5942,1.7283),(1.7283,.5942),(.5942,-1.7283),(-1.7283,-.5942)]):
        shape(f'Fixed flower pot {index+1}', [(x+.08*math.cos(a*math.pi/8), y+.08*math.sin(a*math.pi/8)) for a in range(16)], '#b87913')
    sign = -1 if name.startswith('Blue') else 1
    x, y = -1.55*sign, (.64 if name == "BlueBasket2Wait15Shoot4Park" else 1.19)*sign
    shape('Reserved teammate space', [(x-.2286,y-.2286),(x+.2286,y-.2286),(x+.2286,y+.2286),(x-.2286,y+.2286)], '#8959a8')
    doc = dict(version='2.0.0', startPoint=dict(x=rows[0][1], y=rows[0][2], headingDeg=rows[0][3]), lines=lines,
        sequence=sequence, shapes=shapes, fieldPoints=[], activePaths=[], robot=dict(
            forwardVelocity=74.57640736619945, strafeVelocity=60.78246577619209, aVelocity=math.pi, kFriction=.1,
            rLength=18, rWidth=18, safetyMargin=0, maxAcceleration=50, maxDeceleration=50))
    (output/f'{name}.pp.json').write_text(json.dumps(doc, indent=2)+'\n')

"""Export the production route TSV (RouteExport.java) into Pedro Visualizer 2.0 projects.
Each row after a program's Start row is one continuous Pedro leg: a line, or one Bezier curve
(controls "x,y;x,y"), with linear heading that finishes at turnDoneAt of the leg.
Run tools/auton/export-pedro.ps1, which compiles the route code and calls this script.
"""
import csv
import json
import math
import sys
from pathlib import Path

routes = {}
for row in csv.reader(Path(sys.argv[1] if len(sys.argv) > 1 else 'build/pedro-routes.tsv').read_text().splitlines(), delimiter='\t'):
    name, phase, x, y, heading = row[:5]
    controls = [tuple(float(v) + 70.75 for v in c.split(',')) for c in (row[5].split(';') if len(row) > 5 and row[5] else [])]
    turn_done = float(row[6]) if len(row) > 6 and row[6] else 1.0
    routes.setdefault(name, []).append((phase, float(x) + 70.75, float(y) + 70.75, float(heading), controls, turn_done))
output = Path('docs/pedro')
output.mkdir(exist_ok=True)
for name, rows in routes.items():
    lines, sequence, shapes = [], [], []
    previous_heading = rows[0][3]
    for index, (phase, x, y, heading, controls, turn_done) in enumerate(rows[1:]):
        ident = f'{name}-{index}'
        if turn_done >= 1:
            rule = dict(type='linear', startDeg=previous_heading, endDeg=heading)
        else:
            rule = dict(type='piecewise', piecewiseHeading=dict(segments=[
                dict(startProgress=0, endProgress=turn_done, interpolationType='linear',
                     parameters=dict(startDeg=previous_heading, endDeg=heading)),
                dict(startProgress=turn_done, endProgress=1, interpolationType='constant',
                     parameters=dict(degrees=heading))]))
        lines.append(dict(kind='atomic', id=ident, name=phase, endPoint=dict(x=x, y=y),
            controlPoints=[dict(x=cx, y=cy) for cx, cy in controls], heading=rule,
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
    x, y = -1.55*sign, (.64 if 'Basket2' in name else 1.19)*sign
    shape('Reserved teammate space', [(x-.2286,y-.2286),(x+.2286,y-.2286),(x+.2286,y+.2286),(x-.2286,y+.2286)], '#8959a8')
    doc = dict(version='2.0.0', startPoint=dict(x=rows[0][1], y=rows[0][2], headingDeg=rows[0][3]), lines=lines,
        sequence=sequence, shapes=shapes, fieldPoints=[], activePaths=[], robot=dict(
            forwardVelocity=74.57640736619945, strafeVelocity=60.78246577619209, aVelocity=math.pi, kFriction=.1,
            rLength=18, rWidth=18, safetyMargin=0, maxAcceleration=50, maxDeceleration=50))
    (output/f'{name}.pp.json').write_text(json.dumps(doc, indent=2)+'\n')

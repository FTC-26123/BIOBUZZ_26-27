# Flowerless autonomous setup

There are four selectable autonomous programs under **26123 Flowerless**.

| Red | Blue | Full sequence |
| --- | --- | --- |
| Red Basket 1 Shoot 4 and Park | Blue Basket 1 Shoot 4 and Park | Drive to the fixed shooting mark, use the team Shooter default for four preloads, then park. |
| Red Park Only | Blue Park Only | Leave the separate Park Only wall mark and drive around the frame to park. |

![Overhead rendering of the official FIRST BIOBUZZ field CAD, showing Basket 1 shooting starts, separate Park Only starts, inward headings, and final parking points.](autonomous-start-map.png)

The field image uses the official FIRST BIOBUZZ STEP CAD v26-27.2 bundled with the simulator. Colored footprints, arrows, basket labels, and parking markers are setup overlays. Source attribution and conversion details are in [the simulator field provenance](https://github.com/d-mankotia/BIOBUZZ-sim/blob/main/FIELD-SOURCES.md).

| Routine family | Red robot center (m) | Red Pedro heading | Blue robot center (m) | Blue Pedro heading |
| --- | --- | --- | --- | --- |
| Shoot 4 and Park | (-0.360, -1.564259) | 90 degrees | (+0.360, +1.564259) | 270 degrees |
| Park Only | (-0.900, -1.564259) | 90 degrees | (+0.900, +1.564259) | 270 degrees |

Coordinates refer to the robot's pose origin at the field center convention. Positive X points right and positive Y points up in the map. All starting arrows point into the field. Place the robot at the exact selected mark before Start. Initialize sets Pinpoint to that mark.

## Fixed shooting positions

These coordinates use the Pedro visualizer's bottom-left origin, in inches, on its 141.5-inch playable field. Robot code uses field-centered inches, so it subtracts 70.75 from both coordinates.

| Alliance | Visualizer X, Y (in) | Robot X, Y (in) | Heading |
| --- | --- | --- | --- |
| Red | (57.05, 28.48) | (-13.70, -42.27) | About 87.97 degrees |
| Blue | (84.45, 113.02) | (+13.70, +42.27) | About 267.97 degrees |

Blue is a 180-degree rotation about field center. Starting marks above are unchanged. Shoot 4 and Park clears the wall, drives to its fixed mark facing Basket 1, stops, calls the existing team Shooter class, and then parks outside the metal frame. Shooter uses its default 1500 ticks/second preset and existing PIDF, voltage compensation and feeder readiness. This value is encoder ticks/second, not a new RPM calculation.

There are no camera requirements for the four selectable autons. Recovery remains unregistered. Release events are estimated from recovered flywheel load dips and intake encoder travel. Four estimated releases end shooting; a 10-second shooting timeout or the return-path reserve leads to parking even if fewer releases were observed. Physical robot trials must validate feeding and shot accuracy at the supplied mark.

See [hardware and feeder setup](POLLEN-AUTONOMOUS.md).

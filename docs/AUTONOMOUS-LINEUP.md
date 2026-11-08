# Flowerless autonomous setup

There are eight selectable autonomous programs under **26123 Flowerless**. Existing routine choices remain available.

| Red | Blue | Full sequence |
| --- | --- | --- |
| Red Basket 1 Shoot 4 and Park | Blue Basket 1 Shoot 4 and Park | Drive to the fixed shooting mark, use the team Shooter default for four preloads, then park. |
| Red Park Only | Blue Park Only | Start on the same wall mark as Shoot 4 and Park, then drive one continuous curve around the frame corner to park. |
| Red Shoot 4 + Garden and Park | Blue Shoot 4 + Garden and Park | Same four-preload opening, collect only our garden, then park carrying the pollen for TeleOp. |
| Red Basket 2 Wait 15 Shoot 4 and Park | Blue Basket 2 Wait 15 Shoot 4 and Park | Start at the opposite wall, aim/spin up during the first 15 seconds with feeders off, then shoot four into raised Basket 2 and park. Teammate required. |

The delayed Basket 2 routines have [different starting marks, a setup map and teammate requirements](BASKET-TWO-WAIT15.md). Do not place them at the Basket 1 starts.

The two new garden routes have their own [route map, simulation results and setup requirements](GARDEN-PARK.md). The four original routes are shown below.

![Official FIRST BIOBUZZ CAD with all four colored autonomous routes, their starts and shooting marks, our parking slots, and reserved teammate spaces.](autonomous-start-map.png)

The field image uses the official FIRST BIOBUZZ STEP CAD v26-27.2 bundled with the simulator. Colored footprints, arrows, basket labels, and parking markers are setup overlays. Source attribution and conversion details are in [the simulator field provenance](https://github.com/d-mankotia/BIOBUZZ-sim/blob/main/FIELD-SOURCES.md).

| Routine family | Red robot center (m) | Red Pedro heading | Blue robot center (m) | Blue Pedro heading |
| --- | --- | --- | --- | --- |
| Shoot 4 and Park | (-0.360, -1.564259) | 90 degrees | (+0.360, +1.564259) | 270 degrees |
| Shoot 4 + Garden and Park | (-0.360, -1.564259) | 90 degrees | (+0.360, +1.564259) | 270 degrees |
| Park Only | (-0.360, -1.564259) | 90 degrees | (+0.360, +1.564259) | 270 degrees |
| Basket 2 Wait 15 | (-0.245, +1.564259) | 270 degrees | (+0.245, -1.564259) | 90 degrees |

Coordinates refer to the robot's pose origin at the field center convention. Positive X points right and positive Y points up in the map. All starting arrows point into the field. Place the robot at the exact selected mark before Start. Initialize sets Pinpoint to that mark.

## Parking with a teammate

Every routine parks in the half of our parking zone nearer the wall it started from; the teammate parks in the half nearer theirs. The Basket 1, Garden and Park Only routines start on the far wall and use **our near slot**. Basket 2 Wait 15 starts on the parking-zone wall, so it parks at Blue **(127.88, 21.27)** / Red **(13.62, 120.23)** and leaves the near slot Blue **(131.77, 45.55)** / Red **(9.73, 95.95)** for the teammate. Positions below use Pedro visualizer inches. Blue mirrors Red by a 180-degree rotation about field center.

| Alliance | Our robot parks (X, Y) | Reserved teammate center (X, Y) | Heading |
| --- | --- | --- | --- |
| Red | (9.73, 95.95) | (9.73, 117.60) | 90 degrees |
| Blue | (131.77, 45.55) | (131.77, 23.90) | 270 degrees |

The robot-code parking centers are Red (-1.451, +0.64) m and Blue (+1.451, -0.64) m (about 4 inches off the wall); Blue Basket 2 Wait 15 uses its own point above. The reserved teammate centers are Red (-1.55, +1.19) m and Blue (+1.55, -1.19) m. Two 18-inch square robots have a 3.65-inch gap at these marks. The routes avoid the reserved footprint even when a teammate is already parked there. The dashed boxes are suggested teammate spaces, not an autonomous command for their robot. Coordinate the teammate's approach and confirm their actual footprint and the event's parking requirements.

The original four route colors identify alternative programs; only one runs on our robot. Circles mark starting poses, stars mark shooting positions, and P squares mark our own final slots. Shoot 4 + Garden and Park uses the same parking centers, facing 270 degrees Red / 90 degrees Blue. It leaves the same 3.65-inch gap to the reserved teammate slot.

[Download the printable route and parking map](autonomous-route-map.pdf).

## Fixed shooting positions

These coordinates use the Pedro visualizer's bottom-left origin, in inches, on its 141.5-inch playable field. Robot code uses field-centered inches, so it subtracts 70.75 from both coordinates.

| Alliance | Visualizer X, Y (in) | Robot X, Y (in) | Heading |
| --- | --- | --- | --- |
| Red | (57.0146, 27.4806) | (-13.7354, -43.2694) | About 87.97 degrees |
| Blue | (84.4854, 114.0194) | (+13.7354, +43.2694) | About 267.97 degrees |

Both shooting families now stop exactly one inch backwards along the aim line from the previous marks, away from the hive. Blue is a 180-degree rotation about field center. Starting marks above are unchanged. Shoot 4 and Park drives one straight leg from the wall to its fixed mark facing Basket 1 (turning the last 2 degrees on the way), stops, calls the existing team Shooter class, and then parks in one continuous curve outside the metal frame. Shooter uses its default 1500 ticks/second preset and existing PIDF, voltage compensation and feeder readiness. This value is encoder ticks/second, not a new RPM calculation.

There are no camera requirements for any selectable auton. Recovery remains unregistered. Release events are estimated from recovered flywheel load dips and intake encoder travel. In the original Shoot 4 and Park, four estimated releases end shooting; a 14-second shooting timeout or the return-path reserve leads to parking even if fewer releases were observed. Physical robot trials must validate feeding and shot accuracy at the supplied mark. The new garden program uses the same opening, then reserves time to park with the collected pollen for TeleOp.

See [hardware and feeder setup](POLLEN-AUTONOMOUS.md).

## Continuous routes

Every drive is one Pedro path (`AutoPaths.Leg`): a straight line or a single Bezier curve with linear heading. The robot no longer stops (under 3 in/s and within 2 degrees) at every corner waypoint. It stops only where it must: the shot mark, the garden approach, the end of the sweep and the parking slot.

| Program (per alliance) | Legs before -> now | Pedro Visualizer drive time before -> now |
| --- | --- | --- |
| Basket 1 Shoot 4 and Park | 7 -> 2 | 6.04 s -> 4.00 s |
| Park Only | 3 -> 1 | 4.62 s -> 2.87 s |
| Shoot 4 + Garden and Park | 12 -> 4 | 10.00 s -> 6.55 s |
| Basket 2 Wait 15 Shoot 4 and Park | 5 -> 2 | 3.73 s -> 3.11 s |

The curve control points come from `tools/auton/design-pedro-routes.mjs`. It searches for the fastest curvature-limited Bezier legs and checks them with the official Pedro Visualizer engine. Every designed leg keeps:

- the robot center at least 2 inches outside the `FrameClearance` box;
- the 18-inch footprint at least 2 inches from the fixed flower pot and at least 3 inches from the reserved teammate slot;
- the whole footprint on our alliance's half of the field.

Legs that start at a wall move away from it before sliding sideways. Starting marks, shot marks and both parking slots are unchanged.

If the robot is more than 4 inches from a leg's planned start (for example, after a timeout), or the shifted curve would not clear the frame, the OpMode falls back to the original stop-and-go `FrameClearance` corridor to the same destination. If Garden skips the garden at the shot mark, it parks along the Basket 1 curve.

In the simulator, running the actual OpModes, unchanged Shooter/Intake and tuned Pedro follower, Basket 1 now reaches its shot mark in 0.57 s instead of 1.14 s and parks in 1.87 s instead of 2.40 s after shooting. Park Only finishes in 2.05 s. All eight parked within 4 cm of their assigned slots. Release counts and shooting time still vary from run to run; physical robot trials remain required.

`tools/auton/AutoPathChecks.java` re-checks every leg, including the same legs started up to 2 inches off the mark.

## Pedro Pathing Visualizer projects

`docs/pedro/` has one Pedro Visualizer 2.0 project for each of the eight autonomous programs, generated from the same route code the OpModes call, so the start pose, every leg, every Bezier control point and every heading are exactly what the robot is commanded to follow:

| Program | Project |
| --- | --- |
| Red / Blue Basket 1 Shoot 4 and Park | `RedBasket1Shoot4Park.pp.json`, `BlueBasket1Shoot4Park.pp.json` |
| Red / Blue Basket 2 Wait 15 Shoot 4 and Park | `RedBasket2Wait15Shoot4Park.pp.json`, `BlueBasket2Wait15Shoot4Park.pp.json` |
| Red / Blue Shoot 4 + Garden and Park | `RedGardenPark.pp.json`, `BlueGardenPark.pp.json` |
| Red / Blue Park Only | `RedParkOnly.pp.json`, `BlueParkOnly.pp.json` |

Open one in the Pedro Visualizer with Open project. Each project also draws the whole metal assembly, the four fixed flower pots and the reserved teammate parking space. The shot marks (the end of the leg before "Park" in the shooting autons) are where the OpMode holds position to shoot. All eight pass the visualizer's own collision engine with zero collision intervals.

Regenerate all eight after changing any route (needs the simulator, with your TeamCode loaded, for the SDK classes):

```powershell
powershell -File tools/auton/export-pedro.ps1
```

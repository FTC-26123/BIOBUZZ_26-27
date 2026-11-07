# Flowerless autonomous setup

There are six selectable autonomous programs under **26123 Flowerless**.

| Red | Blue | Full sequence |
| --- | --- | --- |
| Red Basket 1 Shoot 4 and Park | Blue Basket 1 Shoot 4 and Park | Shoot the four preloads into Basket 1, then park. |
| Red Basket 1 Recover and Shoot | Blue Basket 1 Recover and Shoot | Shoot four, move forward, observe Basket 1 reaching its lower stop, collect loose pollen, wait for the teammate to raise Basket 1, shoot again, then park. |
| Red Park Only | Blue Park Only | Leave the separate Park Only wall mark and drive around the frame to park. |

![Overhead rendering of the official FIRST BIOBUZZ field CAD, showing Basket 1 shooting starts, separate Park Only starts, inward headings, and final parking points.](autonomous-start-map.png)

The field image uses the official FIRST BIOBUZZ STEP CAD v26-27.2 bundled with the simulator. Colored footprints, arrows, basket labels, and parking markers are setup overlays. Source attribution and conversion details are in [the simulator field provenance](https://github.com/d-mankotia/BIOBUZZ-sim/blob/main/FIELD-SOURCES.md).

| Routine family | Red robot center (m) | Red Pedro heading | Blue robot center (m) | Blue Pedro heading |
| --- | --- | --- | --- | --- |
| Shoot 4 and Park | (-0.360, -1.564259) | 90 degrees | (+0.360, +1.564259) | 270 degrees |
| Recover and Shoot | (-0.360, -1.564259) | 90 degrees | (+0.360, +1.564259) | 270 degrees |
| Park Only | (-0.900, -1.564259) | 90 degrees | (+0.900, +1.564259) | 270 degrees |

Coordinates refer to the robot's pose origin at the field center convention. Positive X points right and positive Y points up in the map. All starting arrows point into the field. Place the robot at the exact selected mark before Start. Initialize sets Pinpoint to that mark.

All scoring programs start with four pollen preloads. The first volley advances only after four confirmed feed events. A failed shot or missing observation can lead to parking with remaining preloads.

The recovery program uses loose floor pollen from the first tip. Flower-contact areas are excluded from detector pursuit. The camera search is bounded and yields to parking when it cannot acquire targets. It uses fresh expected-basket Limelight measurements to calculate shooting speed for the 86-degree launcher. The USB webcam verifies that Basket 1 has reached the lower stop before recovery and has returned to raised height before the second volley. The teammate must shoot the opposite basket to raise Basket 1 again.

Scoring stays within the Basket 1 quadrant. The return to the loading zone travels outside the metal frame. All routines reserve time for parking and stop at their destination or their deadline. Agree on the shared parking area with the teammate.

See [hardware, camera, and feeder setup](POLLEN-AUTONOMOUS.md). The new sequences require physical robot trials.

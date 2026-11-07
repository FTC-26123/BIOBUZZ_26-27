# Basket autonomous lineup

Choose **Red Basket 1**, **Red Basket 1 Team**, or **Red Basket 2 Team**, or the corresponding Blue option. **Basket 1** is the basket that starts raised. **Basket 2** starts lowered and rises after the opening tip.

![Overhead rendering of the official FIRST BIOBUZZ field CAD with each alliance's basket starts, robot headings, and parking assignments.](autonomous-start-map.png)

The field image uses the bundled official FIRST BIOBUZZ STEP CAD v26-27.2. Colored robot footprints, arrows, basket names, and parking markers are setup overlays. Field coordinates and geometry retain the CAD orientation. Source attribution and conversion details are in [the simulator field provenance](https://github.com/d-mankotia/BIOBUZZ-sim/blob/main/FIELD-SOURCES.md).

| Selection | Red robot center and Pedro heading | Blue robot center and Pedro heading |
| --- | --- | --- |
| Basket 1 / Basket 1 Team | (-0.36, -1.564 m), 90Â° | (+0.36, +1.564 m), 270Â° |
| Basket 2 Team | (-0.90, +1.564 m), 270Â° | (+0.90, -1.564 m), 90Â° |
| Park Only | Basket 1 start when alliance is Red | Basket 1 start when alliance is Blue |

Coordinates are field-centered. 0Â° faces +X and 90Â° faces +Y. Place the physical robot at the selected mark before pressing Start. Initialize sets the Pinpoint pose to that mark.

- **Basket 1** opens on the raised basket and runs the solo flower and garden route across the alliance half. It reserves time to park.
- **Basket 1 Team** opens with three preloads. It stays in the opening quadrant for scoring and looks for fallen pollen with Limelight neural pipeline 7 and the `yellow_pollen` label. It uses the local garden as a fallback. After the other basket tips, the robot can fire into Basket 1 again when the webcam confirms it is raised.
- **Basket 2 Team** starts by its assigned quadrant and waits for Basket 2 to rise. It uses the same loose-pollen search and checks the local pot as a fallback. The scoring robot never feeds into a lowered basket.
- **Park Only** defaults to Red. After Init and before Start, press **X** to select Blue or **B** to select Red. It clears the wall, drives around the frame, and stops in the loading zone with the scoring mechanisms idle.

The two Team routines use separate spots in the loading zone. Basket 1 Team leaves its scoring quadrant when it heads there to park. Agree on basket roles and parking slots with the other driver before selecting a Team routine. Basket 2 Team parks without shooting if the other robot never opens its basket.

These are complete FTC autonomous OpModes. Press Init and then Start once. Each scoring routine handles preloads, collection, repeat feeding, and its timed return to park. The Team routines repeat recovery and shooting when their basket is available. Fresh Limelight measurements set shooting speed for the fixed 86-degree launcher, and the webcam verifies basket availability. Configure Limelight tag pipeline 1 for shooting and neural pipeline 7 for yellow_pollen collection. An unavailable target or time pressure leads to parking.

See [hardware configuration and calibration](POLLEN-AUTONOMOUS.md). Physical robot and paired-robot field trials remain required.

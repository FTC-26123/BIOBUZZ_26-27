# Independent geometry and tolerance review

Reviewer: geometry_review subagent. Production code was reviewed read-only. At the parent's request, regression edits were confined to `tools/auton/check-trained-arrival.py` and `tools/auton/ShiftedRouteChecks.java`.

## Shooting mark calculation

To move a mark one inch farther from the basket without changing its aim line, subtract the unit aim vector: `delta = (-cos(heading), -sin(heading))` inches.

- Canonical red shot 0, heading 1.535403: approximately (-0.035386, -0.999374) inches.
- Canonical red shots 6 and 12, heading 4.747783: approximately (-0.035387, +0.999374) inches.
- Blue rotates these positions and offsets by 180 degrees.

Both incoming endpoint/last control and outgoing start/first control must shift together. The production `curvePoint()` implementation does this for curve pairs 0/1, 6/7, and 12/13. Regression checks independently confirm one inch of backward displacement, zero lateral displacement, and connected adjacent endpoints.

## Flower entry and real simulator geometry

The old approach had distance and yaw cutoffs despite being described as time based. The reviewed replacement commands 65% forward for 0.9 seconds, with bounded heading steering. Ordinary travel distance and contact yaw do not terminate the push before that timer; physical collision, invalid odometry, hard deadline and other genuine interlocks remain separate.

Simulator profile dimensions read from the user's simulator:

- Chassis length: 0.4576660022 m, about 18.018 inches.
- Chassis width: 0.400591312 m, about 15.771 inches.
- Forward body offset: 0.02483252995 m, about 0.978 inches.
- Chassis nose from odometry origin: about 9.986832 inches.
- Flower collider radius: 2.5 inches; its center's forward coordinate is 68.043307 inches.
- With the flower inside the front-face width, the aligned chassis contacts the flower near center coordinate 55.556475 inches. Continuing motor power cannot drive through that infinite-mass collider.
- Flower pickup's simulated throat accepts forward separation 0.15–0.34 m and lateral separation below 0.17 m. The 3-inch robot-right offset is within that lateral throat.

The physical ramp is not part of the simulator chassis collision shape. Collision engagement and continued motion can therefore be tested, but the simulator cannot establish real ramp impact force, flex, or guaranteed mechanical release of balls.

The reviewed production wall guard uses a conservative 10-inch chassis nose and 18-inch width at the selected flower wall, retaining the complete ramp footprint for frame and other-wall checks. At 15 degrees of yaw this nose reaches 11.988630 inches forward, so a 58.5-inch center leaves about 0.096 inches of modeled wall margin. At 30 degrees, the permissible forward coordinate is smaller. Geometry tests accept inboard 30-degree recovery and correctly reject 30-degree contact near the wall; widening a contact corridor must not bypass the chassis wall guard.

## Other movement cancellation and continuation checks

The regression suite exercises:

- Refreshing odometry while waiting for ramp-down confirmation.
- Stable readiness before departure, small startup pose error, bounded never-ready timeout and arm faults.
- Full red/blue state sequences through both flower pushes, collection, retreat, all three shot batches and parking.
- Guarantee that timer expiry cancels manual motor power even when physical position is stalled.
- Small contact yaw retaining timed forward power and receiving steering in the correcting direction.
- Rejecting a flower entry before it starts when there is insufficient time for a complete push, collection and parking.
- Avoiding soft parking-reserve cancellation in the middle of an admitted push or collection.
- Per-flower bounded ramp reseat, a relative reseat timeout, successful resume and rejection of repeated failed reseating.
- Three-inch retreat/normal path handoffs, conservative parking-planner margin escape, emergency retreat and parking waypoint timeout.

## Verification results

Latest locally executed suite at handoff:

- 288 scripted route/startup behavior assertions passed.
- 13 assertions against the actual Shooter recovery/cadence logic passed.
- 82,092 geometry samples using the actual installed Pedro core 3.0.1 curves passed, including broadened contact corners, noisy departure and measured-pose parking escape.

These assertions supplement, rather than replace, live dynamics tests. The live .65/.9 traces supplied by sim_test are:

- `work/sim-auton-review/p0.65-t0.9-RedTrainedTwoTipPark.json`
- `work/sim-auton-review/p0.65-t0.9-BlueTrainedTwoTipPark.json`

Both traces' final non-reset telemetry reports Parked at about 22.7 autonomous seconds and 12 launches. Plotting uses actual `state.robot` coordinates, not target waypoints, and excludes the match-reset zero-position row.

Rendered and visually inspected path figure:

`C:/Users/deepa/Documents/Codex/2026-11-12/https-github-com-ftc-26123-biobuzz/outputs/auton-path-validation.png`

The figure includes both alliances, field walls, actual flower centers/radii, hive frame envelope/supports, and motion states. Shooting batches 2 and 3 share the same mark and are labeled together.

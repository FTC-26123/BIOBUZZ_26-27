# Independent stop/wait review

Reviewed `robot-fixes/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auton/TrainedFlowerAuto.java` and `FlowerArm.java`, baseline V4. Read-only production review; report is the only file written by this reviewer.

| Stage / condition | Existing action | Assessment / required treatment |
|---|---|---|
| Missing flower servo or either switch | Startup DONE | Legitimate hardware fault. Preserve and report which device is missing. |
| Both arm switches, opposite switch after 2 s, arm travel timeout | Arm FAULT then whole auton DONE | Legitimate hardware protection. Simulator must actually supply both switches and CRServo, not bypass them. |
| LOWER_RAMP, down switch or Pinpoint not ready | Stay stationary; update localizer; require 0.15 s continuous readiness | Correct basic wait. Test down-first, ready-first, switch bounce, delayed readiness and never-ready. |
| Startup clock reaches 6 s | DONE | Absolute timeout includes arm travel. If down arrives at 5.9 s, readiness settle cannot finish. Prefer separate bounded readiness timeout after arm completion, or a total allowance explicitly including max arm travel + ready settle. |
| Odometry frame resets while calibrating | May become Missed curve start or clearance abort after readiness | Potential untested startup trap: pose is assigned once in INIT, before READY. Check real PinpointLocalizer behavior; if calibration resets assigned pose, re-establish the known start pose once at stable startup readiness. Never reset pose later to hide tracking errors. |
| Live Pinpoint non-READY after departure | Immediate DONE | Legitimate lost-localization protection. A transient could be debounced only while motors remain stopped, not ignored while driving. |
| Nonfinite pose / own-half / frame / wall / other-flower clearance | Immediate DONE | Preserve genuine faults. Split pose-status and geometry reasons in telemetry to identify the failing branch. |
| Start rear wall | V4 permits 0.75-in overlap only first departure corridor, within 3° | Existing exception addresses 0.016-in nominal rear margin. Ensure simulator tracking noise tests cover both signs and corridor exit. |
| Selected-flower contact corridor | >58.585 forward, >1.75 lateral, >6° yaw loses exception and may immediately DONE | Major conflict with high-speed impact. The model's 15-in ramp-tip footprint should not cancel normal physical flower contact. Base valid contact bounds on actual simulator chassis/flower geometry, while retaining frame and unrelated walls. Test yaw/lateral response after impact. |
| Normal path start >3.1 in away | Divert to PARK | Matches 3-in preceding arrival radius plus margin. When shifting a shooting mark, update all adjoining curve starts and controls or this will trip. |
| Flower push path start >1.1 in away | Divert to PARK | Consistent with 0.75-in staging arrival, but physical follower error and stage telemetry must be tested. |
| Flower staging arrival | <0.75 in, <2°, <2.5 in/s; otherwise wait up to 6 s then park | Tightest remaining travel gate. Could explain stopping in front of flower despite 3-in normal tolerance. Consider a bounded settled-stage alternative with lateral/yaw tests in simulator; do not claim global 3-in tolerance covers it. |
| Normal travel/retreat arrival | <3 in, <3°, <4.5 in/s | Consistent handoff radius; dynamic tests must include final speed/heading that never settle, not just scripted zero-velocity poses. |
| DRIVE elapsed >6 s | PARK | Legitimate bounded stall handling; verify long cross-field leg under reduced traction can finish. |
| PUSH power >0.45 or duration >3 s | DONE | Must update validation if new requested power is 0.65. Otherwise changing the constant alone guarantees failure. |
| PUSH yaw >3° | End push early -> collect/retreat | Conflicts with requested truly timed impact and likely ends contact early. Remove as routine success cutoff; retain only broader genuine hazard protection. |
| PUSH forward >=57.835 or travel >=15 in | End push early -> collect/retreat | Conflicts with true time-based entry. Remove as precision/success cutoff; any exceptional guard must be an explicit hazard, not a hidden old target. |
| PUSH timer | Stop motors, collect 2 s, measured straight retreat | Intended normal behavior. At higher speed, verify stop happens by timer plus one loop and no manual command survives COLLECT/PARK/STOP. |
| Parking reserve `now + reserve >=29.2` in PUSH/COLLECT | Interrupt to PARK | Hidden timer cancellation. For true timed push, check affordability before entering it, reserving push + collect + retreat + park; once admitted, finish that bounded step unless hard deadline/fault. |
| COLLECT 2 s | Drive retreat from measured pose | Correct. Verify the deepest actual collision-limited pose, yawed pose, and a short-of-contact pose all generate a legal retreat. |
| Shot aim | <1 in, <3°, <4.5 in/s continuously 0.25 s | Deliberate stationary shooting wait. Preserve successful shooting gates while moving marks farther; simulator aim errors can still time out. |
| Shot batch 4 estimates OR 2 s permitted FEED | Next leg | Existing successful shooting behavior; keep unchanged. Timed feed is not ball-count confirmation. |
| Shot batch >8 s | PARK | Preserve bounded shooting failure, test low RPM / no ball / uncounted shot. |
| Fallback waypoint <3 in, <3°, <3 in/s | Next waypoint, replan after emergency flower retreat | Correct bounded arrival in principle; path corners can be cut within 3 in, so validate each measured join against frame. |
| Fallback waypoint >6 s | DONE | Explicit final stall fault; do not confuse with normal parked completion. |
| Path preflight samples find any invalid pose | DONE | Legitimate route invalidity; actual Pedro curves must be used, not linear substitutes. Pure nominal sampling is insufficient for impact/turnaround. |
| FrameClearance.path throws | No catch at planner call sites | Existing halo escape reduces one case, but exceptions can still end OpMode without friendly stop/report if current pose/planner assumptions differ. Catch planning failure, stop all motors and report reason. Never silently skip the failed path. |
| Clock >=29.7 s | DONE immediately | Hard match deadline; preserve even during timed push. |
| Completed final leg / no fallback waypoints | Parked DONE | Intended final stationary state. |
| User/SDK STOP | Stops drive, intake, shooter and arm | Mandatory. Test from every state, especially manual PUSH. |

## Essential high-speed regression matrix

- Both alliances, both pots; current 3-in robot-right lineup; new one-inch-farther shooting marks.
- Fresh start with arm already down, arm up, delayed down switch, delayed Pinpoint, stale INIT pose, switch fault.
- Full 0.65-power timed impact with normal / low traction / higher battery response, bounce, 3–6° yaw and side deflection.
- Timer ends at collision-limited pose, then collection, measured retreat, next shooting batch and final park.
- Late start: do not begin an unaffordable push and then cancel halfway because parking reserve crosses its threshold.
- Every legitimate fault has an identifiable final state/reason and all output power zero. No generic “Pose / clearance” ambiguity.
- Actual simulator physical collision/pickup and full robot timing are required. Existing 246 scripted checks teleport poses and feed synthetic shot events; they prove transitions, not closed-loop pathing or flower tipping.

## Review of forthcoming V5 diff

- Power 0.65 and 0.9-second timer are now accepted by the updated 0.8/1.5 validator. Removing the ordinary yaw and distance cutoffs meets the requested timing behavior.
- The one-inch shooting offset mapping correctly changes all three shooting endpoints and their adjoining path controls/starts. The chosen direction is backward along the existing robot aim heading, preserving aim angle.
- **Still actionable:** the soft parking reserve runs before PUSH/COLLECT. Do an affordability check before entry, including the full push, two-second collection, retreat from projected contact and parking; once admitted, do not cancel the short timer merely because the soft reserve threshold crosses. Keep the 29.7-second hard deadline and genuine faults.
- **Still actionable:** a contact corridor permitting forward=62 inches and yaw=15 degrees does not entirely fit an eight-inch tip-overlap allowance: `15 cos(15°)+9 sin(15°)=16.818`, so the front corner is at 78.818 inches versus wall 70.585 (8.233-inch overlap). A pose declared in corridor can still hard-stop. Prefer the simulator's measured physical chassis/ramp collision geometry to simply increasing this magic allowance.
- **Still actionable:** touchDown can open after the arm is nudged by contact. Existing DOWN state does not re-lower automatically, and the next flower's `isAtDown()` check ends the whole auton. At a safe stationary stage, issue a bounded down/reseat command and service the real arm controller before entry; retain both-switch and travel-timeout faults.
- A fixed six-second overall startup wait can preempt a valid slowly lowered arm plus the required 0.15-second readiness settle. A deadline derived from max arm travel plus a bounded Pinpoint-ready wait makes the contract clearer.
- Inspected installed Pedro revhub 3.0.1 bytecode: `PinpointLocalizer.update()` always refreshes the driver, and `setPose()` writes hardware `setPosition()`. Constructor recalibration sleeps 500 ms. A calibration race resetting the initial hardware pose is a test hypothesis, not a confirmed software reset; do not add unconditional late odometry snapping without evidence.

## Final V5 follow-up review

- The revised physical-body wall check uses a conservative 10-inch nose/18-inch width only for the selected contacted wall, retaining the full ramp footprint for the frame and unrelated walls. At the largest accepted forward coordinate 58.5 and 15-degree yaw, the wall-facing chassis corner is `58.5 + 10 cos(15°) + 9 sin(15°) = 70.4886` inches, within the 70.585-inch modeled wall. This resolves the earlier eight-inch magic-padding contradiction. Nominal margin is only about 0.096 inches at that extreme, so actual simulation perturbations remain necessary.
- Admission budgeting and the exclusion of PUSH/COLLECT from the soft reserve prevent halfway cancellation. **One remaining fix requested:** `reserve()` is evaluated at the pre-push staging pose, so it excludes approximately one second of future retreat from actual contact. The max center movement is about 15.2 inches (0.386 m), which takes 0.965 s at the reserve model's 0.4 m/s; include lateral variation/margin (~1.1–1.25 s) or evaluate the reserve at projected contact. Hard deadline remains mandatory.
- Per-flower ramp reseating is bounded to one attempt per leg, retains arm fault protections, and resets its own readiness deadline correctly. Readiness budget now includes configured max arm travel plus three seconds. No blocking control-flow problem found there.
- The timer is still subject to genuine localization/geometry/arm faults and the hard deadline; this is intentional and should be described as a timer-only normal completion criterion, not an unconditional motor command regardless of faults.

## Final production audit — steering/contact revision

Reviewed the shared file after the 0.65-power / 0.9-second timed push, bounded heading steering, 30-degree contact corridor, physical contacted-wall chassis check, admission retreat margin, and ramp reseat changes.

| Current stop/wait | Final classification |
|---|---|
| 0.9-second PUSH timer | Sole ordinary entry completion condition. No nominal distance or yaw-success cutoff remains. |
| Heading steering | Error correction capped at ±0.25, not an early stop. Wrapper cap during PUSH is power + 0.25, capped at 1.0, so 0.65 forward plus maximum steering is not unintentionally scaled below forward power. |
| Selected contact wall | Checks physical conservative 10-inch chassis nose/18-inch width instead of the 15-inch ramp tip. Still rejects actual modeled chassis penetration. A broad 30-degree corridor is not permission for every corridor pose to violate this independent chassis boundary. |
| Admission budget | Includes full push + collection + 1.25-second projected retreat allowance + staging parking reserve. The previous missing future-retreat allowance is fixed. |
| Soft parking reserve | Excluded during admitted PUSH and COLLECT. Still applies before/after them. Hard deadline and genuine faults still interrupt. |
| Arm reseat | One bounded attempt per flower leg; real switch/fault controller remains active. New readiness deadline is measured from each reseat. |
| Readiness deadline | Uses max(6 seconds, configured arm max travel + 3 seconds); touchDown and Pinpoint READY must settle 0.15 seconds. Global 29.7-second deadline still wins. |
| Staging tolerance | Still 0.75 inch / 2 degrees / 2.5 in/s, with six-second timeout; intentionally tighter than normal 3-inch travel. Must be included in live scenario coverage. |
| All other stops | Original inventory's hardware faults, invalid localization, true geometry violations, normal travel/shot/parking timeouts, invalid push configuration, SDK STOP, and normal Parked completion remain. |

### Independently verified baseline simulator failures

Parsed both full JSON traces rather than the reset final row. `RedTrainedTwoTipPark.json`: first PUSH at simulator time 6.36 s, DONE at 6.795 s, only one flower ball collected; heading error 15.1924 degrees and “Pose / clearance interlock.” `BlueTrainedTwoTipPark.json`: PUSH 6.31 s, DONE 6.74 s, one flower ball collected; heading error 15.5070 degrees with the same reason. Both fail approximately 0.43 seconds into a requested 0.9-second entry, confirming the old contact/yaw envelope caused premature cancellation. These are baseline failures, not results for the final 30-degree/steering revision.

### Coverage limitations to retain in delivery

- Simulator uses the actual autonomous controller/Pedro and subsystems, but required test hardware adapters provide transfer motor and arm servo/touch switches; Robot Settings persistence is stubbed. Saved real-robot settings and electrical faults therefore remain outside live coverage.
- A match-end reset erases simulator robot/shooting counters. Assess the last nonempty telemetry frame before reset, not the final JSON row.
- Scripted route assertions teleport localization and estimated shot events; they do not replace the closed-loop simulator traces.
- Real robot impact behavior, servo/load flex and ball pickup still require a field trial. A simulator pass is not a hardware guarantee.
- Exception injection for a failed parking planner and invalid velocity samples is not currently covered. Pose nonfiniteness is checked, but finite pose with invalid measured velocity can still prevent arrival or lead to a controller/SDK error.

Audit signed: independent `stop_review` sub-agent. Production files were not edited by this reviewer.

### Independently verified final closed-loop traces

Parsed all six `p0.55-t0.9-*`, `p0.65-t0.9-*`, and `p0.75-t0.9-*` JSON files and selected the final nonempty controller telemetry before the match-end reset.

| Forward power | Alliance | Controller elapsed at Parked | Shots / hits | Flower balls collected | Batches | Unintended stop |
|---|---|---:|---:|---:|---:|---|
| 0.55 | Red | 22.7189 s | 12 / 11 | 8 | 3 | None |
| 0.55 | Blue | 22.6934 s | 12 / 11 | 8 | 3 | None |
| **0.65** | **Red** | **22.7064 s** | **12 / 11** | **8** | **3** | **None** |
| **0.65** | **Blue** | **22.7098 s** | **12 / 11** | **8** | **3** | **None** |
| 0.75 | Red | 22.8040 s | 12 / 11 | 8 | 3 | None |
| 0.75 | Blue | 22.6549 s | 12 / 11 | 8 | 3 | None |

Both flower visits in every run transitioned PUSH → COLLECT with `Timed flower push complete`, followed by retreat and continued routing. All runs reached final curve 16 / `Parked`, `Abort details: None`, `Parking reason: None`. No startup or tolerance cancellation appears in these traces. The observed successful collection is eight flower balls, not merely a scripted release count.

These data support the final timed-contact/steering route at default settings and bracketed power. They **do not** support a claim of perfect scoring: the simulator reports eleven hits from twelve shots in every run. JSON state polling is coarser than the controller loop, so transition sample differences should not be used as precise timer-duration measurements.

### Reduced-grip follow-up, independently verified

Parsed `p0.65-t0.9-lowgrip12v-RedTrainedTwoTipPark.json` and the corresponding Blue JSON. With requested physical profile 12 V, forward grip 0.8 and strafe grip 0.6, red parks at 22.8396 controller seconds and blue at 22.8047. Both report 12 shots, 11 hits, eight flower balls collected and no parking/abort reason. This brings verified successful closed-loop runs to eight. Production still has the +1.25-second admission retreat allowance and ±0.25 steering; obsolete distance fields/telemetry have been removed.

### Lower-voltage follow-up, independently verified

Parsed `p0.65-t0.9-lowvoltage10v5-RedTrainedTwoTipPark.json` and the corresponding Blue JSON. With requested 10.5 V physical supply, red parks at 23.6412 controller seconds and blue at 23.5520. Both report 12 shots, 11 hits, eight flower balls collected and no abort/parking reason. Total verified successful final-revision live runs: **ten**, covering both alliances, forward power 0.55/0.65/0.75, reduced grip and lower physical supply. All completed both flower pickups and all three shot batches before the 29.7-second deadline.

Battery coverage caveat: SDK telemetry reports 14 V while the physical profile is 12 V or 10.5 V. The simulator's reported sensor voltage does not match the requested physics supply, so these are physical motor-response tests, not validated tests of real battery-voltage compensation. The mismatch remains present in both 10.5 V trace files.

Live initial-placement perturbations and delayed physical sensor runs have not been executed; scripted tests cover the latter state transitions. Keep these coverage distinctions in the final user-facing report. No remaining blocking production control-flow defect was found in this review; actual robot dimensions and sensor/persisted-settings behavior remain field-validation requirements.

package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auton.cycles.AutoPaths;
import org.firstinspires.ftc.teamcode.Auton.cycles.FrameClearance;
import org.firstinspires.ftc.teamcode.Auton.cycles.SideRoute;
import org.firstinspires.ftc.teamcode.subsystems.FlowerArm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShotConfig;
import org.firstinspires.ftc.teamcode.subsystems.ShotTable;
import org.firstinspires.ftc.teamcode.subsystems.TargetTracker;

import java.util.Collections;
import java.util.Iterator;
import java.util.List;

/**
 * THE autonomous program. One entry on the Driver Station; during INIT the drive team picks, on gamepad 1:
 * <pre>
 *   X = BLUE          B = RED
 *   Dpad Up = UP side (our basket is raised: shoot straight away)
 *   Dpad Down = DOWN side (our basket is down: wait for the other side to tip it)
 *   Dpad Left / Right = plan: shoot + flower + park / shoot + park / park only
 * </pre>
 *
 * What it does:
 * <pre>
 *   UP side:    drive to the shot mark (flywheel spinning up on the way) -> aim -> shoot the balls
 *               -> flower (arm down, drive in, intake, back out) -> park
 *   DOWN side:  drive to the shot mark -> WAIT until the camera sees our basket come up (or the wait runs out)
 *               -> shoot -> flower if there is still time -> park
 * </pre>
 * In every case the robot keeps track of how long the drive to its parking slot takes and leaves for it in
 * time; whatever it is doing is cut short if needed. All numbers are in {@link AutoConfig}; all field positions
 * are in {@link SideRoute}.
 *
 * Optional parts switch themselves off when they are not available:
 * camera aim/speed/tip detection needs ShotConfig.AUTO_SHOT_ENABLED and a Limelight; the flower needs
 * FlowerArm.ENABLED and the servo.
 */
@Autonomous(name = "BIOBuzz Auto (pick alliance + side)", group = "26123")
public class BioBuzzAuto extends OpMode {

    private static final String[] PLAN_NAMES = {"Shoot + flower + park", "Shoot + park", "Park only"};
    private static final int PLAN_FULL = 0, PLAN_SHOOT_PARK = 1, PLAN_PARK_ONLY = 2;

    // The choices are kept between runs so a re-init does not lose them.
    private static boolean blue = AutoConfig.DEFAULT_BLUE;
    private static boolean upSide = AutoConfig.DEFAULT_UP_SIDE;
    private static int plan = AutoConfig.DEFAULT_PLAN;

    private enum State { APPROACH, WAIT_FOR_TIP, SHOOT, TO_FLOWER, ARM_DOWN, PUSH_IN, COLLECT, BACK_OUT, LATE_BALLS, PARK, DONE }

    private final ElapsedTime timer = new ElapsedTime();
    private final Intake intake = new Intake();
    private final Shooter shooter = new Shooter();
    private final FlowerArm arm = new FlowerArm();
    private final Limelight limelight = new Limelight();
    private final TargetTracker tracker = new TargetTracker();
    private final ShotTable table = new ShotTable(ShotConfig.SHOT_TABLE);

    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private SideRoute route;
    private boolean cameraReady = false;
    private boolean started = false;
    private boolean mechanismsReady = false;

    private State state = State.DONE;
    private String result = "Ready", note = "";
    private double stateStarted = 0;

    // driving
    private Iterator<AutoPaths.Leg> legs = Collections.emptyIterator();
    private Pose target;
    private double legStarted = 0;

    // shooting
    private double aimHeading, lastAimUpdate, lastShotAt, feedingSince, raisedSince = -1;
    private int lastShotCount;
    private boolean sawBasket, aimGaveUp;

    // camera position fix (see AutoConfig "CAMERA POSITION FIX")
    private Pose shotHold;                    // where the robot holds while shooting (the shot mark, or where it really is)
    private final double[] fixDx = new double[32], fixDy = new double[32];
    private int fixCount;
    private boolean fixDone;
    private String fixText = "not measured yet";

    // =====================================================================
    // INIT: hardware and the menu
    // =====================================================================

    @Override
    public void init() {
        follower = org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        intake.init(hardwareMap);
        shooter.init(hardwareMap, intake);
        arm.init(hardwareMap);   // does not move the arm
        mechanismsReady = true;
        cameraReady = AutoConfig.USE_CAMERA && ShotConfig.AUTO_SHOT_ENABLED
                && limelight.initSafely(hardwareMap, ShotConfig.APRILTAG_PIPELINE);
        started = false;
        state = State.DONE;
        result = "Ready";
        applyChoice();
    }

    /** Builds the route for the current alliance/side and tells the odometry where the robot is standing. */
    private void applyChoice() {
        // AutoConfig decides which wall mark is the UP side and which flower that start uses.
        boolean basketOneStart = upSide != AutoConfig.SWAP_START_WALLS;
        int flowerNumber = basketOneStart ? AutoConfig.FLOWER_FOR_BASKET1_START : AutoConfig.FLOWER_FOR_BASKET2_START;
        route = new SideRoute(blue, basketOneStart, flowerNumber == 2 ? AutoConfig.FLOWER_2 : AutoConfig.FLOWER_1,
                AutoConfig.FLOWER_DRIVE_IN_BACKWARDS, AutoConfig.FLOWER_STANDOFF_IN);
        follower.setPose(route.initial);
        follower.update();
    }

    @Override
    public void init_loop() {
        boolean changed = false;
        if (gamepad1.xWasPressed()) { blue = true; changed = true; }
        if (gamepad1.bWasPressed()) { blue = false; changed = true; }
        if (gamepad1.dpadUpWasPressed()) { upSide = true; changed = true; }
        if (gamepad1.dpadDownWasPressed()) { upSide = false; changed = true; }
        if (gamepad1.dpadRightWasPressed()) plan = (plan + 1) % PLAN_NAMES.length;
        if (gamepad1.dpadLeftWasPressed()) plan = (plan + PLAN_NAMES.length - 1) % PLAN_NAMES.length;
        if (changed) applyChoice();

        telemetry.addLine("=== CHECK BEFORE PLAY (gamepad 1) ===");
        telemetry.addData("ALLIANCE  (X blue / B red)", blue ? "BLUE" : "RED");
        telemetry.addData("SIDE  (Dpad Up / Down)", upSide ? "UP - shoot straight away" : "DOWN - wait for the tip");
        telemetry.addData("PLAN  (Dpad Left / Right)", PLAN_NAMES[plan]);
        telemetry.addData("Start mark", "the old \"%s\" wall mark", upSide != AutoConfig.SWAP_START_WALLS ? "Basket 1" : "Basket 2");
        telemetry.addLine("Load " + AutoConfig.BALLS + " balls by hand.");
        telemetry.addData("Place robot (field m)", "%.3f, %.3f  heading %.0f deg", route.initial.x() * .0254,
                route.initial.y() * .0254, Math.toDegrees(route.initial.heading()));
        telemetry.addData("Camera", cameraReady ? "ON (aim, speed, tip detection)"
                : upSide ? "off - shooting on odometry and the fixed speed"
                : AutoConfig.DOWN_NO_CAMERA_SHOOT_AT_SEC >= 0
                    ? "off - will shoot at " + AutoConfig.DOWN_NO_CAMERA_SHOOT_AT_SEC + " s WITHOUT seeing the tip"
                    : "off - cannot see the tip, will NOT shoot");
        telemetry.addData("Flower arm", arm.isAvailable() ? "ready" : arm.getStatus() + " - flower step is skipped");
        telemetry.update();
    }

    // =====================================================================
    // PLAY
    // =====================================================================

    @Override
    public void start() {
        timer.reset();
        started = true;
        applyChoice();
        arm.up();
        if (cameraReady) limelight.start();
        shooter.resetShotCount();
        if (plan == PLAN_PARK_ONLY) {
            drive(route.parkOnlyLegs(follower.pose()));
            enter(State.PARK, "Park only");
            return;
        }
        // Flywheel spins up on the way to the shot mark; nothing is fed until the robot is there.
        // (DOWN side with DOWN_SPIN_FLYWHEEL_WHILE_WAITING = false: it stays off until the tip is seen.)
        shooter.setAutoTarget(upSide ? AutoConfig.UP_SHOT_SPEED : AutoConfig.DOWN_SHOT_SPEED);
        if (upSide || AutoConfig.DOWN_SPIN_FLYWHEEL_WHILE_WAITING) shooter.start();
        drive(route.approachLegs(follower.pose()));
        enter(State.APPROACH, "Driving to the shot mark");
    }

    @Override
    public void loop() {
        if (!started || state == State.DONE) { report(); return; }
        double now = timer.seconds();
        if (now >= AutoConfig.HARD_STOP_SEC) { finish("End of autonomous"); return; }

        follower.update();
        Pose p = follower.pose();
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY
                || !Double.isFinite(p.x() + p.y() + p.heading())) { finish("Odometry fault - stopped"); return; }
        if (!FrameClearance.clear(p.x() * .0254, p.y() * .0254)) { finish("Too close to the frame - stopped"); return; }

        if (cameraReady) {
            limelight.update();
            tracker.update(limelight.getLastResult(), blue, now);
        }

        // THE PARKING RULE: whatever is going on, leave in time to be parked by PARK_BY_SEC.
        if (state != State.PARK && now >= AutoConfig.PARK_BY_SEC - AutoConfig.PARK_MARGIN_SEC
                - SideRoute.seconds(parkPlan(p))) {
            beginPark(p, "Out of time: going to park");
        }

        switch (state) {
            case APPROACH:     approach(now, p); break;
            case WAIT_FOR_TIP: waitForTip(now, p); break;
            case SHOOT:        shoot(now, p); break;
            case TO_FLOWER:    shooter.update(); flowerArrive(now, p); break;
            case ARM_DOWN:     shooter.update(); flowerArmDown(now); break;
            case PUSH_IN:      shooter.update(); flowerPushIn(now); break;
            case COLLECT:      shooter.update(); flowerCollect(now); break;
            case BACK_OUT:     shooter.update(); flowerBackOut(now, p); break;
            case LATE_BALLS:   shooter.update(); if (now - stateStarted >= AutoConfig.FLOWER_INTAKE_EXTRA_SEC) beginPark(p, "Flower done: parking"); break;
            case PARK:         shooter.update(); if (driveDone(now, p)) finish("Parked"); break;
            default: break;
        }
        report();
    }

    // =====================================================================
    // Steps
    // =====================================================================

    private void approach(double now, Pose p) {
        shooter.updateSpinupOnly();   // flywheel only (if started); intake, transfer and windmill stay off
        if (!driveDone(now, p)) return;
        follower.hold(route.shot);
        if (upSide) beginShooting(now, "Shooting");
        else enter(State.WAIT_FOR_TIP, "Waiting for our basket to come up");
    }

    // ---------------- flower sequence (order and intake timing come from AutoConfig) ----------------

    /** At the stand-off point in front of the flower. */
    private void flowerArrive(double now, Pose p) {
        if (!driveDone(now, p)) return;
        if (AutoConfig.FLOWER_ARM_DOWN_BEFORE_DRIVE_IN) {
            arm.down();
            enter(State.ARM_DOWN, "Lowering the flower arm");
        } else {
            beginPushIn();
        }
    }

    private void beginPushIn() {
        if (AutoConfig.FLOWER_INTAKE_WHILE_DRIVING_IN) intake.start();
        enter(State.PUSH_IN, "Driving into the flower");
    }

    /** Waiting for the arm to finish swinging down (at the stand-off point, or against the flower). */
    private void flowerArmDown(double now) {
        if (now - stateStarted < AutoConfig.FLOWER_ARM_DOWN_SEC) return;
        if (AutoConfig.FLOWER_ARM_DOWN_BEFORE_DRIVE_IN) beginPushIn();
        else beginCollect();
    }

    private void flowerPushIn(double now) {
        if (!slide(now, route.flowerStandoff, route.flower)) return;
        if (AutoConfig.FLOWER_ARM_DOWN_BEFORE_DRIVE_IN) {
            beginCollect();
        } else {
            arm.down();
            enter(State.ARM_DOWN, "Lowering the flower arm at the flower");
        }
    }

    private void beginCollect() {
        intake.start();
        enter(State.COLLECT, "Collecting from the flower");
    }

    private void flowerCollect(double now) {
        if (now - stateStarted < AutoConfig.FLOWER_COLLECT_SEC) return;
        if (!AutoConfig.FLOWER_INTAKE_WHILE_BACKING_OUT) intake.stop();
        enter(State.BACK_OUT, "Backing out of the flower");
    }

    private void flowerBackOut(double now, Pose p) {
        if (!slide(now, route.flower, route.flowerStandoff)) return;
        if (AutoConfig.FLOWER_INTAKE_EXTRA_SEC > 0) {
            intake.start();
            enter(State.LATE_BALLS, "Intake running for late balls");
        } else {
            beginPark(p, "Flower done: parking");
        }
    }

    /** DOWN side: hold the shot mark with the flywheel ready until the basket is seen UP, or time runs out. */
    private void waitForTip(double now, Pose p) {
        if (AutoConfig.DOWN_SPIN_FLYWHEEL_WHILE_WAITING) shooter.updateSpinupOnly();
        else shooter.update();   // flywheel off; it is started when the tip is seen
        if (cameraReady) {
            boolean raised = seesOurBasket() && tracker.getTagHeightInches() >= AutoConfig.BASKET_UP_TAG_HEIGHT_IN;
            if (!raised) raisedSince = -1;
            else if (raisedSince < 0) raisedSince = now;
            note = !tracker.isVisible() ? "basket tags not in view"
                    : String.format(java.util.Locale.US, "tag height %.1f in (UP above %.1f)",
                            tracker.getTagHeightInches(), AutoConfig.BASKET_UP_TAG_HEIGHT_IN);
            if (raisedSince >= 0 && now - raisedSince >= AutoConfig.TIP_CONFIRM_SEC) {
                beginShooting(now, "Basket is UP: shooting");
                return;
            }
        } else if (AutoConfig.DOWN_NO_CAMERA_SHOOT_AT_SEC >= 0 && now >= AutoConfig.DOWN_NO_CAMERA_SHOOT_AT_SEC) {
            beginShooting(now, "No camera: timed shot");
            return;
        }
        // Give up when the wait limit is reached, or when too little time would be left to shoot and still park.
        double lastUsefulStart = AutoConfig.PARK_BY_SEC - AutoConfig.PARK_MARGIN_SEC
                - SideRoute.seconds(route.parkLegs(route.shot)) - AutoConfig.MIN_SHOOT_SEC;
        if (now >= Math.min(AutoConfig.DOWN_MAX_WAIT_SEC, lastUsefulStart)) beginPark(p, "Basket never came up: parking");
    }

    private void beginShooting(double now, String why) {
        aimHeading = route.shot.heading();
        lastAimUpdate = -1;
        lastShotAt = now;
        feedingSince = -1;
        lastShotCount = shooter.getShotCount();
        sawBasket = false;
        aimGaveUp = false;
        shotHold = route.shot;
        fixCount = 0;
        fixDone = false;
        shooter.start();   // does nothing if the flywheel is already running for this volley
        follower.hold(route.shot);
        enter(State.SHOOT, why);
    }

    private void shoot(double now, Pose p) {
        double sinceStart = now - stateStarted;
        if (checkPosition(p)) return;   // odometry was just corrected; carry on next loop with the new position
        boolean onMark = near(p, new Pose(shotHold.x(), shotHold.y(), aimHeading),
                AutoConfig.ON_MARK_INCHES, AutoConfig.ON_MARK_DEGREES, 6);
        boolean permit = onMark;

        if (cameraReady && seesOurBasket() && !aimGaveUp) {
            sawBasket = true;
            double error = tracker.getAimErrorDeg();
            boolean aimed = Math.abs(error) <= ShotConfig.AIM_FIRE_TOLERANCE_DEG;
            if (!aimed && sinceStart > AutoConfig.CAMERA_AIM_GIVE_UP_SEC) {
                // The camera could not line it up (wrong sign, bad view...). Shoot from the planned heading.
                aimGaveUp = true;
                aimHeading = route.shot.heading();
                follower.hold(new Pose(shotHold.x(), shotHold.y(), aimHeading));
                note = "camera aim gave up: using the planned heading";
            } else {
                if (now - lastAimUpdate >= 0.1) {
                    lastAimUpdate = now;
                    double wanted = p.heading() - AutoConfig.CAMERA_TURN_SIGN * Math.toRadians(error);
                    double fromPlan = wrap(wanted - route.shot.heading());
                    double limit = Math.toRadians(AutoConfig.CAMERA_AIM_MAX_DEG);
                    aimHeading = route.shot.heading() + Math.max(-limit, Math.min(limit, fromPlan));
                    follower.hold(new Pose(shotHold.x(), shotHold.y(), aimHeading));
                }
                permit = permit && aimed;
                note = String.format(java.util.Locale.US, "camera aim error %+.1f deg", error);
                if (table.isReady() && tracker.hasDistance()) {
                    double step = Math.max(1, ShotConfig.AUTO_SPEED_STEP);
                    double speed = Math.round(table.speedFor(tracker.getDistanceInches()) / step) * step;
                    if (Math.abs(speed - shooter.getAutoBaseSpeed()) >= step) shooter.setAutoTarget(speed);
                }
            }
        } else if (cameraReady && !sawBasket && sinceStart < AutoConfig.CAMERA_WAIT_SEC) {
            permit = false;   // give the camera a moment to find the basket before shooting blind
        }

        shooter.setFeedPermit(permit);
        shooter.update();

        int shots = shooter.getShotCount();
        boolean feeding = "FEED".equals(shooter.getFeedStage());
        if (shots != lastShotCount) { lastShotCount = shots; lastShotAt = now; feedingSince = -1; }
        if (!feeding) feedingSince = -1;
        else if (feedingSince < 0) feedingSince = now;

        if (shots >= AutoConfig.BALLS) {
            if (now - lastShotAt >= AutoConfig.AFTER_LAST_SHOT_SEC) afterShooting(now, p, "All balls shot");
        } else if (feedingSince >= 0 && now - feedingSince >= AutoConfig.NO_SHOT_GIVE_UP_SEC) {
            afterShooting(now, p, "No more balls coming (" + shots + " counted)");
        } else if (sinceStart >= AutoConfig.SHOOT_BUDGET_SEC) {
            afterShooting(now, p, "Shooting time used (" + shots + " counted)");
        }
    }

    // ---------------- camera position fix ----------------

    /**
     * While the robot is still and the camera sees our raised basket, compares "where the tags say the robot is"
     * with the odometry. After enough agreeing frames the difference is shown, and applied to the odometry if
     * AutoConfig.CAMERA_FIX_POSITION is on. The robot is NOT driven back to the shot mark: it keeps shooting
     * from where it is (the camera aim and distance already allow for that); what is corrected is every drive
     * after the shots.
     *
     * @return true in the one loop where the odometry was changed
     */
    private boolean checkPosition(Pose p) {
        if (fixDone || !cameraReady || !seesOurBasket()) return false;
        int[] ids = tracker.getTagIds();
        if (ids.length < AutoConfig.CAMERA_FIX_MIN_TAGS) return false;
        com.pedropathing.math.Velocity v = follower.velocity();
        if (Math.hypot(v.vx, v.vy) > 2 || Math.abs(v.omega) > 0.15) return false;   // only while standing still

        // Each tag gives one estimate of the robot position: tag position minus the measured offset to it.
        double[] ahead = tracker.getTagForwardMeters(), right = tracker.getTagRightMeters();
        double c = Math.cos(p.heading()), s = Math.sin(p.heading()), x = 0, y = 0;
        for (int i = 0; i < ids.length; i++) {
            double[] tag = tagFieldMeters(ids[i]);
            x += tag[0] - ahead[i] * c - right[i] * s;
            y += tag[1] - ahead[i] * s + right[i] * c;
        }
        fixDx[fixCount] = x / ids.length / .0254 - p.x();
        fixDy[fixCount] = y / ids.length / .0254 - p.y();
        fixCount++;
        int wanted = Math.max(1, Math.min(fixDx.length, AutoConfig.CAMERA_FIX_SAMPLES));
        if (fixCount < wanted) return false;

        fixDone = true;
        double[] dxs = java.util.Arrays.copyOf(fixDx, fixCount), dys = java.util.Arrays.copyOf(fixDy, fixCount);
        java.util.Arrays.sort(dxs);
        java.util.Arrays.sort(dys);
        double dx = dxs[fixCount / 2], dy = dys[fixCount / 2], size = Math.hypot(dx, dy);
        double spread = Math.max(dxs[fixCount - 1] - dxs[0], dys[fixCount - 1] - dys[0]);
        fixText = String.format(java.util.Locale.US, "camera - odometry = (%+.1f, %+.1f) in", dx, dy);
        if (spread > AutoConfig.CAMERA_FIX_SPREAD_IN) { fixText += ": frames disagree, ignored"; return false; }
        if (size > AutoConfig.CAMERA_FIX_MAX_IN) { fixText += ": too large to believe, ignored"; return false; }
        if (size < AutoConfig.CAMERA_FIX_MIN_IN) { fixText += ": already right"; return false; }
        if (!AutoConfig.CAMERA_FIX_POSITION) { fixText += ": shown only (CAMERA_FIX_POSITION is off)"; return false; }

        // Correct the odometry, and hold the spot the robot is really on so it does not move mid-volley.
        Pose corrected = new Pose(p.x() + dx, p.y() + dy, p.heading());
        follower.setPose(corrected);
        shotHold = new Pose(corrected.x(), corrected.y(), aimHeading);
        follower.hold(shotHold);
        fixText += ": APPLIED";
        return true;
    }

    /** Field position (metres) of one of our basket's tags with the basket UP. */
    private double[] tagFieldMeters(int id) {
        int first = route.basketFirstTag(ShotConfig.BLUE_FIRST_TAG, ShotConfig.RED_FIRST_TAG, ShotConfig.TAGS_PER_BASKET);
        int index = Math.max(0, Math.min(AutoConfig.BASKET_TAG_OFFSETS_IN.length - 1, id - first));
        double alliance = blue ? -1 : 1, end = route.basketOneStart ? -1 : 1;
        return new double[] {
                alliance * (AutoConfig.BASKET_TAG_CENTER_X_M - end * AutoConfig.BASKET_TAG_OFFSETS_IN[index] * .0254),
                alliance * end * AutoConfig.BASKET_TAG_Y_UP_M};
    }

    /** Shooting is over: flower if it is wanted, possible and fits in the time left; otherwise park. */
    private void afterShooting(double now, Pose p, String why) {
        shooter.setFeedPermit(true);
        shooter.stop();
        intake.stop();
        if (plan == PLAN_FULL && arm.isAvailable()) {
            List<AutoPaths.Leg> toFlower = route.flowerLegs(p);
            if (toFlower != null && now + flowerSeconds(toFlower) <= AutoConfig.PARK_BY_SEC) {
                drive(toFlower);
                enter(State.TO_FLOWER, why + " -> flower");
                return;
            }
            why += toFlower == null ? " (no clear path to the flower)" : " (no time for the flower)";
        }
        beginPark(p, why + " -> parking");
    }

    /** Honest estimate of the whole flower detour including the drive to the parking slot afterwards. */
    private double flowerSeconds(List<AutoPaths.Leg> toFlower) {
        double slide = Math.hypot(route.flower.x() - route.flowerStandoff.x(),
                route.flower.y() - route.flowerStandoff.y()) * .0254 / AutoConfig.FLOWER_PUSH_M_PER_S;
        return SideRoute.seconds(toFlower) + AutoConfig.FLOWER_ARM_DOWN_SEC + 2 * slide + AutoConfig.FLOWER_COLLECT_SEC
                + Math.max(0, AutoConfig.FLOWER_INTAKE_EXTRA_SEC)
                + SideRoute.seconds(route.parkLegs(route.flowerStandoff)) + AutoConfig.PARK_MARGIN_SEC;
    }

    /** How the robot would get to the parking slot from where it is right now. */
    private List<AutoPaths.Leg> parkPlan(Pose p) {
        boolean atFlower = state == State.PUSH_IN || state == State.COLLECT || state == State.BACK_OUT
                || (state == State.ARM_DOWN && !AutoConfig.FLOWER_ARM_DOWN_BEFORE_DRIVE_IN);
        return atFlower ? route.parkFromFlowerLegs(p) : route.parkLegs(p);
    }

    private void beginPark(Pose p, String why) {
        List<AutoPaths.Leg> plan = parkPlan(p);
        shooter.setFeedPermit(true);
        shooter.stop();
        intake.stop();
        arm.up();
        drive(plan);
        enter(State.PARK, why);
    }

    // =====================================================================
    // Driving helpers
    // =====================================================================

    private void drive(List<AutoPaths.Leg> plan) {
        legs = plan.iterator();
        nextLeg();
    }

    private void nextLeg() {
        if (!legs.hasNext()) { target = follower.pose(); return; }
        AutoPaths.Leg leg = legs.next().from(follower.pose());
        target = leg.end;
        legStarted = timer.seconds();
        follower.follow(leg.path());
    }

    /** Follows the current list of legs. @return true once the last one has been reached. */
    private boolean driveDone(double now, Pose p) {
        if (Math.hypot(p.x() - target.x(), p.y() - target.y()) < 2 && !follower.holding()) follower.hold(target);
        boolean last = !legs.hasNext();
        boolean late = now - legStarted > AutoConfig.LEG_LATE_SEC;
        // Points on the way only need to be passed cleanly; the last point is tight unless the robot is slow to settle.
        boolean arrived = last ? (late ? near(p, target, 3.0, 6, 8) : near(p, target, 1.0, 2, 3))
                : (late ? near(p, target, 3.0, 8, 10) : near(p, target, 1.5, 5, 6));
        if (!arrived) return false;
        if (last) return true;
        nextLeg();
        return false;
    }

    /** Moves the hold target steadily from one pose to another (used for the slow push into the flower). */
    private boolean slide(double now, Pose from, Pose to) {
        double inches = Math.hypot(to.x() - from.x(), to.y() - from.y());
        double seconds = inches * .0254 / AutoConfig.FLOWER_PUSH_M_PER_S;
        double u = seconds <= 0 ? 1 : Math.min(1, (now - stateStarted) / seconds);
        follower.hold(new Pose(from.x() + (to.x() - from.x()) * u, from.y() + (to.y() - from.y()) * u, to.heading()));
        // Done when the target has arrived and the robot has had a moment to catch up (it may be pressing on the flower).
        return u >= 1 && (near(follower.pose(), to, 1.5, 5, 4) || now - stateStarted > seconds + 0.6);
    }

    private boolean near(Pose p, Pose goal, double inches, double degrees, double inchesPerSecond) {
        return Math.hypot(p.x() - goal.x(), p.y() - goal.y()) < inches
                && Math.abs(wrap(p.heading() - goal.heading())) < Math.toRadians(degrees)
                && Math.hypot(follower.velocity().vx, follower.velocity().vy) < inchesPerSecond;
    }

    private static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }

    /** true when the camera sees the basket this side is meant to shoot at (not our other basket). */
    private boolean seesOurBasket() {
        return tracker.isVisible() && tracker.getBasketFirstTag()
                == route.basketFirstTag(ShotConfig.BLUE_FIRST_TAG, ShotConfig.RED_FIRST_TAG, ShotConfig.TAGS_PER_BASKET);
    }

    // =====================================================================
    // Bookkeeping
    // =====================================================================

    private void enter(State next, String why) {
        state = next;
        stateStarted = timer.seconds();
        result = why;
        note = "";
    }

    private void finish(String why) {
        result = why + String.format(java.util.Locale.US, " at %.1f s", timer.seconds());
        state = State.DONE;
        arm.up();
        stop();
        report();
    }

    private void report() {
        telemetry.addData("Auto", "%s %s, %s", blue ? "BLUE" : "RED", upSide ? "UP side" : "DOWN side", PLAN_NAMES[plan]);
        telemetry.addData("Step", state);
        telemetry.addData("Status", result);
        if (!note.isEmpty()) telemetry.addData("Detail", note);
        telemetry.addData("Clock (s)", "%.1f", started ? timer.seconds() : 0.0);
        telemetry.addData("Balls counted", shooter.getShotCount());
        if (cameraReady) telemetry.addData("Camera position check", fixText);
        telemetry.addData("Flower arm", arm.getStatus());
        shooter.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) { follower.stop(); follower.drivetrain.stop(); }
        if (mechanismsReady) { shooter.stop(); intake.stop(); }
        if (cameraReady) limelight.stop();
    }
}

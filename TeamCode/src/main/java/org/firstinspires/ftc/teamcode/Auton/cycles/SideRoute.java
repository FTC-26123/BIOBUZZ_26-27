package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * All the field positions for one alliance and one starting side, built from the routes the earlier programs
 * already use (so the start marks, shot marks, curves and parking slots are the same ones):
 * <ul>
 *   <li>Basket 1 wall start = the "Basket 1 Shoot 4 and Park" start, shot mark and parking slot.</li>
 *   <li>Basket 2 wall start = the "Basket 2 Wait 15" start, shot mark and parking slot.</li>
 * </ul>
 * Which of the two is the UP side and which flower each uses is decided in AutoConfig, not here.
 * Each start parks in the half of the parking zone nearer its own starting wall, which leaves the other half
 * for the teammate.
 */
public final class SideRoute {
    /** true = this robot starts at the "Basket 1" wall mark, false = at the "Basket 2" wall mark. */
    public final boolean blue, basketOneStart;
    public final Pose initial, shot, park, flower, flowerStandoff;
    private final CycleRoute upRoute;
    private final DelayedBasketTwoRoute downRoute;

    /**
     * @param blue                 alliance
     * @param basketOneStart       true = start at the Basket 1 wall mark, false = at the Basket 2 wall mark
     * @param flowerRed            flower for this start: {x metres, y metres, facing degrees} for RED
     * @param driveInBackwards     true = the back of the robot leads into the flower
     * @param flowerStandoffInches how far back from the flower the robot stops first
     */
    public SideRoute(boolean blue, boolean basketOneStart, double[] flowerRed, boolean driveInBackwards,
                     double flowerStandoffInches) {
        this.blue = blue;
        this.basketOneStart = basketOneStart;
        if (basketOneStart) {
            upRoute = new CycleRoute(blue ? CycleRoute.Start.BLUE_WEST : CycleRoute.Start.RED_EAST,
                    CycleRoute.Strategy.NO_FLOWERS, CycleRoute.Lineup.FOUR_PARK);
            downRoute = null;
            initial = upRoute.initial;
            shot = upRoute.shot();
            park = upRoute.park;
        } else {
            upRoute = null;
            downRoute = new DelayedBasketTwoRoute(blue);
            initial = downRoute.initial;
            shot = downRoute.shot;
            park = downRoute.park;
        }
        // The stand-off point is straight back from the flower along the direction of the push.
        double back = Math.max(0, flowerStandoffInches) * .0254;
        double push = Math.toRadians(flowerRed[2]);
        double robotHeading = flowerRed[2] + (driveInBackwards ? 180 : 0);
        flower = pose(flowerRed[0], flowerRed[1], robotHeading);
        flowerStandoff = pose(flowerRed[0] - back * Math.cos(push), flowerRed[1] - back * Math.sin(push), robotHeading);
    }

    private Pose pose(double x, double y, double degrees) {
        return new Pose((blue ? -x : x) / .0254, (blue ? -y : y) / .0254, Math.toRadians(degrees + (blue ? 180 : 0)));
    }

    /** First AprilTag id of the basket this side shoots at (four tags in a row). */
    public int basketFirstTag(int blueFirstTag, int redFirstTag, int tagsPerBasket) {
        // The autonomous field model: the Basket 2 end carries the lower four ids, the Basket 1 end the next four.
        return (blue ? blueFirstTag : redFirstTag) + (basketOneStart ? tagsPerBasket : 0);
    }

    /** Start mark to shot mark. */
    public List<AutoPaths.Leg> approachLegs(Pose from) {
        return basketOneStart ? AutoPaths.plan(upRoute.approachLeg(), from, blue) : downRoute.approachLegs(from);
    }

    /**
     * From anywhere to our parking slot, staying outside the metal frame. Uses the designed curve when the robot
     * is on the shot mark, otherwise the clear corridor around the frame corner.
     */
    public List<AutoPaths.Leg> parkLegs(Pose from) {
        try {
            if (basketOneStart) return upRoute.parkingLegs(from);
            if (Math.hypot(from.x() - shot.x(), from.y() - shot.y()) <= AutoPaths.REUSE_INCHES) {
                return downRoute.parkingLegs(from);
            }
            return AutoPaths.lines("Park", from, FrameClearance.path(from, park, blue));
        } catch (IllegalArgumentException noClearPath) {
            // Should not happen from a legal position. Driving straight is better than not parking at all.
            return Collections.singletonList(AutoPaths.Leg.line("Park (direct)", from, park));
        }
    }

    /** Park-only: straight from the start mark to the slot. */
    public List<AutoPaths.Leg> parkOnlyLegs(Pose from) {
        if (basketOneStart) {
            return AutoPaths.plan(AutoPaths.curve("Drive around the frame to park", blue, initial, park,
                    AutoPaths.START_TO_PARK), from, blue);
        }
        return parkLegs(from);
    }

    /**
     * From anywhere to the stand-off point in front of our flower.
     *
     * @return the legs, or null if there is no path that stays clear of the frame
     */
    public List<AutoPaths.Leg> flowerLegs(Pose from) {
        try {
            return AutoPaths.lines("To the flower", from, FrameClearance.path(from, flowerStandoff, blue));
        } catch (IllegalArgumentException noClearPath) {
            return null;
        }
    }

    /** Leaving the flower: straight back to the stand-off point first (no turning next to the flower bed), then park. */
    public List<AutoPaths.Leg> parkFromFlowerLegs(Pose from) {
        List<AutoPaths.Leg> legs = new ArrayList<>();
        Pose back = new Pose(flowerStandoff.x(), flowerStandoff.y(), from.heading());
        if (Math.hypot(from.x() - back.x(), from.y() - back.y()) > 1.0) {
            legs.add(AutoPaths.Leg.line("Back away from the flower", from, back));
        }
        legs.addAll(parkLegs(back));
        return legs;
    }

    /** Driving time for a list of legs at the same cautious speed the other programs budget for. */
    public static double seconds(List<AutoPaths.Leg> legs) {
        return AutoPaths.reserveSeconds(legs);
    }
}

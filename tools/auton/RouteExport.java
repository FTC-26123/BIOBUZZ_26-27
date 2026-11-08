package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.math.Pose;
import java.util.List;
import java.util.Locale;

/**
 * Prints every leg each of the eight autonomous programs drives, calling the same route code the OpModes call.
 * Row: name, phase, endX, endY, endHeadingDeg, controls ("x,y;x,y" or empty), turnDoneAt — Pedro field-centre inches.
 * The first row of each program is its start pose. Each later row is one AutoPaths.Leg: a straight line or one Bezier
 * curve with linear heading (finishing at turnDoneAt of the leg), exactly what follower.follow(leg.path()) receives.
 */
public final class RouteExport {
    private static void start(String name, Pose p) {
        System.out.printf(Locale.US, "%s\tStart\t%.9f\t%.9f\t%.9f\t\t1%n", name, p.x(), p.y(), Math.toDegrees(p.heading()));
    }

    private static void legs(String name, List<AutoPaths.Leg> legs) {
        for (AutoPaths.Leg l : legs) {
            StringBuilder c = new StringBuilder();
            for (Pose q : l.controlPoints()) {
                if (c.length() > 0) c.append(';');
                c.append(String.format(Locale.US, "%.6f,%.6f", q.x(), q.y()));
            }
            System.out.printf(Locale.US, "%s\t%s\t%.9f\t%.9f\t%.9f\t%s\t%.4f%n", name, l.name,
                l.end.x(), l.end.y(), Math.toDegrees(l.end.heading()), c, l.turnDoneAt);
        }
    }

    public static void main(String[] args) {
        for (boolean blue : new boolean[]{false, true}) {
            String side = blue ? "Blue" : "Red";
            CycleRoute.Start start = blue ? CycleRoute.Start.BLUE_WEST : CycleRoute.Start.RED_EAST;

            CycleRoute basket = new CycleRoute(start, CycleRoute.Strategy.NO_FLOWERS, CycleRoute.Lineup.FOUR_PARK);
            start(side + "Basket1Shoot4Park", basket.initial);
            legs(side + "Basket1Shoot4Park", AutoPaths.plan(basket.approachLeg(), basket.initial, blue));
            legs(side + "Basket1Shoot4Park", basket.parkingLegs(basket.shot()));

            start(side + "ParkOnly", start.wallPose());
            legs(side + "ParkOnly", ParkOnlyAuto.legs(start, start.wallPose()));

            GardenParkRoute garden = new GardenParkRoute(blue);
            start(side + "GardenPark", garden.initial);
            Pose at = garden.initial;
            for (int i = 0; i < garden.stages.size(); i++) {
                CycleRoute.Stage s = garden.stages.get(i);
                if (s.action() == CycleRoute.Action.SWEEP) legs(side + "GardenPark", AutoPaths.lines(s.name(), at, java.util.Collections.singletonList(s.pose())));
                else legs(side + "GardenPark", garden.legsInto(i, at));
                at = s.pose();
            }
            legs(side + "GardenPark", garden.parkingLegs(at));

            DelayedBasketTwoRoute late = new DelayedBasketTwoRoute(blue);
            start(side + "Basket2Wait15Shoot4Park", late.initial);
            legs(side + "Basket2Wait15Shoot4Park", late.approachLegs(late.initial));
            legs(side + "Basket2Wait15Shoot4Park", late.parkingLegs(late.shot));
        }
    }
}

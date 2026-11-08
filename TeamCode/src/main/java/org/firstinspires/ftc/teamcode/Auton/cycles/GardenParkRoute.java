package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.math.Pose;
import java.util.ArrayList;
import java.util.List;

/** Camera-free own-garden reload for TeleOp. No opposite garden or second-basket shots. */
public final class GardenParkRoute {
    public static final double SWEEP_METERS_PER_SECOND=.25,CAPTURE_DWELL_SECONDS=.50;
    public static final double PICKUP_POSITION_TOLERANCE_INCHES=.20,PICKUP_HEADING_TOLERANCE_DEGREES=1;
    public final boolean blue;
    public final Pose initial,park;
    public final List<CycleRoute.Stage> stages=new ArrayList<>();
    private final double sign;
    public GardenParkRoute(boolean blue){
        this.blue=blue;sign=blue?-1:1;
        CycleRoute opening=new CycleRoute(blue?CycleRoute.Start.BLUE_WEST:CycleRoute.Start.RED_EAST,
            CycleRoute.Strategy.NO_FLOWERS,CycleRoute.Lineup.FOUR_PARK);
        initial=opening.initial;stages.addAll(opening.stages);
        // Angle the front intake 15 degrees toward the corner. The old parallel sweep caught
        // the last ball only at the outer edge of the intake and needed millimeter-perfect tracking.
        stages.add(new CycleRoute.Stage("Our garden approach",pose(-1.15,-1.48,255),CycleRoute.Action.MOVE,0,8));
        stages.add(new CycleRoute.Stage("Our garden floor sweep",pose(-1.50,-1.48,255),CycleRoute.Action.SWEEP,0,8));
        park=pose(CycleRoute.PARK_X_METERS,CycleRoute.OUR_PARK_Y_METERS,270);
    }
    private Pose pose(double x,double y,double degrees){
        return new Pose(sign*x/.0254,sign*y/.0254,Math.toRadians(degrees+(blue?180:0)));
    }
    public List<Pose> parkingPath(Pose from){
        List<Pose> result=new ArrayList<>();
        // Retreat inboard: hugging the west/east wall would clip the fixed flower pot.
        Pose entry=pose(-1.13,-1.05,270),exit=pose(-1.13,.50,270);
        result.addAll(FrameClearance.path(from,entry,blue));
        result.addAll(FrameClearance.path(entry,exit,blue));
        result.addAll(FrameClearance.path(exit,park,blue));return result;
    }
    public double parkReserve(Pose from){
        double seconds=.8;
        for(Pose next:parkingPath(from)){
            double turn=Math.atan2(Math.sin(next.heading()-from.heading()),Math.cos(next.heading()-from.heading()));
            seconds+=Math.hypot(next.x()-from.x(),next.y()-from.y())*.0254/1.2+Math.abs(turn)/2+.5;
            from=next;
        }
        return seconds;
    }
}

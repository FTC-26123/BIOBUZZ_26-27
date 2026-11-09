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
    /** Designed Pedro leg INTO each stage (null for the slow SWEEP, which uses a moving hold target). */
    public final List<AutoPaths.Leg> legs=new ArrayList<>();
    private final double sign;
    private final CycleRoute opening;
    public GardenParkRoute(boolean blue){
        this.blue=blue;sign=blue?-1:1;
        opening=new CycleRoute(blue?CycleRoute.Start.BLUE_WEST:CycleRoute.Start.RED_EAST,
            CycleRoute.Strategy.NO_FLOWERS,CycleRoute.Lineup.FOUR_PARK);
        initial=opening.initial;stages.addAll(opening.stages);legs.add(opening.approachLeg());
        // Angle the front intake 15 degrees toward the corner. The old parallel sweep caught
        // the last ball only at the outer edge of the intake and needed millimeter-perfect tracking.
        stages.add(new CycleRoute.Stage("Our garden approach",pose(-1.15,-1.48,255),CycleRoute.Action.MOVE,0,8));
        // One straight leg from the shot mark; the 167-degree turn finishes by 60% so the rotating corners clear the wall.
        legs.add(AutoPaths.Leg.line("Our garden approach",opening.shot(),stages.get(stages.size()-1).pose(),AutoPaths.GARDEN_TURN_DONE));
        stages.add(new CycleRoute.Stage("Our garden floor sweep",pose(-1.50,-1.48,255),CycleRoute.Action.SWEEP,0,8));
        legs.add(null);
        park=pose(CycleRoute.PARK_X_METERS,CycleRoute.OUR_PARK_Y_METERS,270);
    }
    private Pose pose(double x,double y,double degrees){
        return new Pose(sign*x/.0254,sign*y/.0254,Math.toRadians(degrees+(blue?180:0)));
    }
    /** Legs to drive into stage {@code index} from the measured pose. */
    public List<AutoPaths.Leg> legsInto(int index,Pose from){
        AutoPaths.Leg designed=legs.get(index);
        return AutoPaths.plan(designed,from,blue);
    }
    /** Sweep end -> park in one continuous curve, inboard of the fixed flower pot. */
    public AutoPaths.Leg parkLeg(){
        return AutoPaths.curve("Park with garden pollen",blue,stages.get(stages.size()-1).pose(),park,AutoPaths.GARDEN_TO_PARK);
    }
    /** Parking from the measured pose: the designed curve after the sweep; the Basket 1 curve (same slot, facing
     * the opening heading) if the garden was skipped at the shot mark; otherwise the inboard corridor. */
    public List<AutoPaths.Leg> parkingLegs(Pose from){
        Pose shot=opening.shot();
        if(Math.hypot(from.x()-shot.x(),from.y()-shot.y())<=AutoPaths.REUSE_INCHES){
            List<AutoPaths.Leg> early=opening.parkingLegs(from);
            if(early.size()==1)return early;
        }
        return AutoPaths.plan(parkLeg(),from,parkingPath(from));
    }
    /** Conservative stop-and-go corridor; used only when the robot is far from the sweep end. */
    public List<Pose> parkingPath(Pose from){
        List<Pose> result=new ArrayList<>();
        // Retreat inboard: hugging the west/east wall would clip the fixed flower pot.
        Pose entry=pose(-1.13,-1.05,270),exit=pose(-1.13,.50,270);
        result.addAll(FrameClearance.path(from,entry,blue));
        result.addAll(FrameClearance.path(entry,exit,blue));
        result.addAll(FrameClearance.path(exit,park,blue));return result;
    }
    public double parkReserve(Pose from){return AutoPaths.reserveSeconds(parkingLegs(from));}
}

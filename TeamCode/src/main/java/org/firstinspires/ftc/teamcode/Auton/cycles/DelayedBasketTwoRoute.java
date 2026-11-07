package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.math.Pose;
import java.util.ArrayList;
import java.util.List;

/** Basket 2 wall start; timed teammate routine, not a sensed basket-state transition. */
public final class DelayedBasketTwoRoute {
    public static final double FEED_NOT_BEFORE_SECONDS=15.0,AIM_SETTLE_SECONDS=.25;
    public final boolean blue;
    public final Pose initial,clearWall,shot,park;
    private final double sign;
    public DelayedBasketTwoRoute(boolean blue){
        this.blue=blue;sign=blue?-1:1;
        // Offset sideways from the usual start: its opposite-wall reflection clips a fixed flower pot.
        initial=pose(-.245,1.564259,270);
        clearWall=pose(-.245,1.43,270);
        // Reflect the current Basket 1 mark across Y=0: exactly the same hive standoff.
        Pose near=new CycleRoute(CycleRoute.Start.RED_EAST,CycleRoute.Strategy.NO_FLOWERS,
            CycleRoute.Lineup.FOUR_PARK).stages.get(1).pose();
        double heading=2*Math.PI-near.heading()+(blue?Math.PI:0);
        shot=new Pose(sign*near.x(),-sign*near.y(),Math.atan2(Math.sin(heading),Math.cos(heading)));
        park=blue ? new Pose(131.44-70.75,19.94-70.75,Math.PI/2)
            : pose(-1.55,CycleRoute.OUR_PARK_Y_METERS,270);
    }
    private Pose pose(double x,double y,double degrees){
        double heading=Math.toRadians(degrees+(blue?180:0));
        return new Pose(sign*x/.0254,sign*y/.0254,(heading%(2*Math.PI)+2*Math.PI)%(2*Math.PI));
    }
    public static boolean readyToFeed(double now,double stableFor){
        return Double.isFinite(now+stableFor)&&now>=FEED_NOT_BEFORE_SECONDS&&stableFor>=AIM_SETTLE_SECONDS;
    }
    public List<Pose> approachPath(){
        List<Pose> result=new ArrayList<>();result.add(clearWall);
        result.addAll(FrameClearance.path(clearWall,shot,blue));return result;
    }
    public List<Pose> parkingPath(Pose from){
        List<Pose> result=new ArrayList<>();
        if(blue){
            // Preserve the clear approach corridor for an early abort; after shooting drive straight to park.
            if(Math.hypot(from.x()-shot.x(),from.y()-shot.y())>1e-6)
                result.addAll(FrameClearance.path(from,shot,true));
            result.add(park);
            return result;
        }
        Pose upper=pose(-1.06,.94,Math.toDegrees(shot.heading())-(blue?180:0));
        Pose lower=new Pose(upper.x(),sign*.50/.0254,shot.heading());
        result.addAll(FrameClearance.path(from,upper,blue));
        result.addAll(FrameClearance.path(upper,lower,blue));
        result.addAll(FrameClearance.path(lower,park,blue));return result;
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

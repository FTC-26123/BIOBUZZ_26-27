package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.math.Pose;

/**
 * Timed sideways intake pass; deliberately makes no claim of sensed inventory.
 * It never throws and never gives up on its own: the hold keeps correcting toward the
 * reference, the capture window widens once the stage is late, and an unsafe measured
 * start is replaced by the designed start so the hold pulls the robot back onto the lane.
 */
public final class GardenStrafe {
    public static final double POSITION_INCHES=1.0,HEADING_DEGREES=3;
    public static final double LATE_POSITION_INCHES=3.0,LATE_HEADING_DEGREES=8;
    /** After the stage limit plus this grace the pass counts as made (the lane was driven with the intake on). */
    public static final double GIVE_UP_GRACE_SECONDS=2.0;
    private Pose from,to;
    private double started=-1,captured=-1,duration;
    private boolean skipped;
    public void reset(){started=captured=-1;skipped=false;}
    public Pose target(Pose measured,Pose destination,double now){return target(measured,null,destination,now,false);}
    /**
     * @param designedFrom planned start of the pass, used if the measured start would cross the frame keep-out; may be null
     * @param late true once the stage limit has passed: widens the capture window so a robot that cannot settle still completes
     */
    public Pose target(Pose measured,Pose designedFrom,Pose destination,double now,boolean late){
        if(started<0){
            from=measured;to=destination;started=now;
            if(!clear(from,to)){
                // Off the lane: reference starts at the designed origin; the hold drags the robot there.
                from=designedFrom;
                if(from==null||!clear(from,to))skipped=true;
            }
            if(!skipped)duration=Math.hypot(to.x()-from.x(),to.y()-from.y())*.0254/GardenParkRoute.SWEEP_METERS_PER_SECOND;
        }
        if(skipped)return measured;
        double u=duration<=0?1:Math.min(1,Math.max(0,(now-started)/duration));
        double angle=Math.abs(Math.atan2(Math.sin(measured.heading()-to.heading()),Math.cos(measured.heading()-to.heading())));
        double position=late?LATE_POSITION_INCHES:POSITION_INCHES,heading=late?LATE_HEADING_DEGREES:HEADING_DEGREES;
        boolean at=u>=1&&Math.hypot(measured.x()-to.x(),measured.y()-to.y())<position&&angle<Math.toRadians(heading);
        if(!at)captured=-1;else if(captured<0)captured=now;
        return new Pose(from.x()+(to.x()-from.x())*u,from.y()+(to.y()-from.y())*u,to.heading());
    }
    private static boolean clear(Pose a,Pose b){return FrameClearance.segment(a.x()*.0254,a.y()*.0254,b.x()*.0254,b.y()*.0254);}
    /** True when no safe pass exists from either start; the stage is then skipped rather than crashing the auto. */
    public boolean skipped(){return skipped;}
    public boolean complete(double now){return skipped||captured>=0&&now-captured>=GardenParkRoute.CAPTURE_DWELL_SECONDS;}
}

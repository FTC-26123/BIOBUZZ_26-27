package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.math.Pose;

/** Timed sideways intake pass; deliberately makes no claim of sensed inventory. */
public final class GardenStrafe {
    private Pose from,to;
    private double started=-1,captured=-1,duration;
    public void reset(){started=captured=-1;}
    public Pose target(Pose measured,Pose destination,double now){
        if(started<0){
            from=measured;to=destination;started=now;
            duration=Math.hypot(to.x()-from.x(),to.y()-from.y())*.0254/GardenParkRoute.SWEEP_METERS_PER_SECOND;
            if(!FrameClearance.segment(from.x()*.0254,from.y()*.0254,to.x()*.0254,to.y()*.0254))
                throw new IllegalArgumentException("Garden strafe enters frame keepout");
        }
        double u=duration<=0?1:Math.min(1,Math.max(0,(now-started)/duration));
        double angle=Math.abs(Math.atan2(Math.sin(measured.heading()-to.heading()),Math.cos(measured.heading()-to.heading())));
        boolean at=u>=1&&Math.hypot(measured.x()-to.x(),measured.y()-to.y())<GardenParkRoute.PICKUP_POSITION_TOLERANCE_INCHES
            &&angle<Math.toRadians(GardenParkRoute.PICKUP_HEADING_TOLERANCE_DEGREES);
        if(!at)captured=-1;else if(captured<0)captured=now;
        return new Pose(from.x()+(to.x()-from.x())*u,from.y()+(to.y()-from.y())*u,to.heading());
    }
    public boolean complete(double now){return captured>=0&&now-captured>=GardenParkRoute.CAPTURE_DWELL_SECONDS;}
}

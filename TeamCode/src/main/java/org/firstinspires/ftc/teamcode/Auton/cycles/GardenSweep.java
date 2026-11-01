package org.firstinspires.ftc.teamcode.Auton.cycles;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.*;

/** Replans the legal garden pickup from fresh image detections and capture-time odometry.
 * The wall-aligned lane is a known field feature; target X is measured from camera pixels. */
public final class GardenSweep {
    private Pose goal,scan;
    private double arrived=-1,lastPlan=-1;
    private boolean paused;
    private boolean inLane;
    public int replans;
    public String status="Waiting for pollen image";
    public void reset(Pose pose){scan=pose;goal=null;arrived=-1;lastPlan=-1;replans=0;paused=false;inLane=false;}
    public boolean update(Follower follower,CycleMechanisms mechanisms,LLResult frame,boolean blue,double now){
        if(!SpillCollector.fresh(frame,0)){
            paused=true;mechanisms.collect(false);follower.stop();follower.drivetrain.stop();status="Stale image: stopped";return false;
        }
        Pose p=follower.pose();com.pedropathing.math.Velocity v=follower.velocity();double sign=blue?-1:1;
        if(paused){if(goal!=null)follower.follow(Paths.line(p,goal).linear(p,goal));arrived=-1;paused=false;}
        double age=(frame.getStaleness()+frame.getCaptureLatency()+frame.getTargetingLatency()+frame.getParseLatency())/1000;
        Pose captured=new Pose(p.x()-v.vx*age,p.y()-v.vy*age,p.heading()-v.omega*age);
        double best=Double.POSITIVE_INFINITY;double[] target=null;
        for(com.qualcomm.hardware.limelightvision.LLResultTypes.ColorResult blob:frame.getColorResults()){
            double[] xy=SpillCollector.project(blob.getTargetXDegrees(),blob.getTargetYDegrees(),captured,.03556);
            if(xy==null)continue;double x=xy[0]*sign,y=xy[1]*sign;
            if(x< -1.80||x> -1.40||y< -1.82||y> -1.65)continue;
            double d=Math.hypot(xy[0]-p.x()*.0254,xy[1]-p.y()*.0254);if(d<best){best=d;target=xy;}
        }
        if(target!=null&&now-lastPlan>.12){
            // Once camera-confirmed, traverse the full known wall lane; the last sphere
            // disappears below the camera before it reaches the intake.
            Pose next=new Pose(sign*(inLane?-1.50:-1.05)/.0254,sign*(-1.575)/.0254,Math.toRadians(180+(blue?180:0)));
            if(goal==null||Math.hypot(next.x()-goal.x(),next.y()-goal.y())>1.2){
                goal=next;replans++;lastPlan=now;arrived=-1;
                follower.follow(Paths.line(p,goal).linear(p,goal));
            }
        }
        if(goal==null){mechanisms.collect(false);follower.hold(scan);status="Looking for garden pollen";return false;}
        mechanisms.collect(true);double distance=Math.hypot(p.x()-goal.x(),p.y()-goal.y());
        if(distance<2&&!follower.holding())follower.hold(goal);
        double headingError=Math.abs(Math.atan2(Math.sin(p.heading()-goal.heading()),Math.cos(p.heading()-goal.heading())));
        if(!inLane&&distance<.30&&headingError<Math.toRadians(1)){
            inLane=true;goal=new Pose(sign*(-1.50)/.0254,sign*(-1.575)/.0254,Math.toRadians(180+(blue?180:0)));
            follower.follow(Paths.line(p,goal).linear(p,goal));arrived=-1;replans++;return false;
        }
        if(distance<.20&&headingError<Math.toRadians(1)){if(arrived<0)arrived=now;if(now-arrived>.22){mechanisms.collect(false);return true;}}
        else arrived=-1;
        status="Following pixel target; replans "+replans;return false;
    }
}

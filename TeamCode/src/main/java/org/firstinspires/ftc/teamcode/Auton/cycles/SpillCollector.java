package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.*;
import java.util.*;

/** Ground-plane projection from neural-detector angles, never simulator ball positions.
 * Calibration: fixed camera 0.305m high, +10deg pitch, zero yaw/roll/offset.
 * Pickup attempts are estimates; only flywheel load confirms subsequent feed events. */
public final class SpillCollector {
    public static final double CAMERA_PITCH=10, CAMERA_HEIGHT=.305, BALL_RADIUS=.03556;
    private final List<double[]> attempted=new ArrayList<>();
    private final Deque<double[]> queued=new ArrayDeque<>();
    private double plannedAt;
    private Pose scan,goal;
    private double arrived=-1;
    private double emptySince=-1;
    private boolean returning;
    public int attempts;
    public boolean clustered;
    public boolean avoidFlowers;
    public boolean avoidFrame;
    // Alliance-normalized Y direction for a team assignment; zero allows both ends.
    public int quadrantEnd;
    private Pose routedGoal;
    private final Deque<Pose> detour=new ArrayDeque<>();
    /** Exclude known pot contact zones; detector class alone cannot identify a ball's source. */
    public static boolean clearOfPots(double x,double y){
        for(double[] pot:new double[][]{{-.5942,1.7283},{1.7283,.5942},{.5942,-1.7283},{-1.7283,-.5942}})
            if(Math.hypot(x-pot[0],y-pot[1])<.42)return false;
        return true;
    }
    public String status="Scanning";
    public boolean hasAttemptedPickup(){return !attempted.isEmpty();}
    public void reset(Pose pose){scan=pose;goal=null;routedGoal=null;detour.clear();queued.clear();attempted.clear();attempts=0;arrived=-1;emptySince=-1;returning=false;}
    public static boolean fresh(LLResult frame,int pipeline){
        if(frame==null||frame.getPipelineIndex()!=pipeline)return false;
        double age=frame.getStaleness()+frame.getCaptureLatency()+frame.getTargetingLatency()+frame.getParseLatency();
        return Double.isFinite(age)&&age>=0&&age<=VisionRules.MAX_AGE_MS;
    }
    /** Empty fresh frames remain usable searches; never fall back to color/nectar results. */
    public static List<LLResultTypes.DetectorResult> pollen(LLResult frame) {
        List<LLResultTypes.DetectorResult> targets = new ArrayList<>();
        if (!fresh(frame, VisionRules.POLLEN_PIPELINE)) return targets;
        for (LLResultTypes.DetectorResult target : frame.getDetectorResults()) {
            if (VisionRules.pollen(target.getClassName(), target.getConfidence(),
                    target.getTargetXDegrees(), target.getTargetYDegrees())) targets.add(target);
        }
        return targets;
    }
    public static double[] project(double tx,double ty,Pose pose,double radius){
        double pitch=Math.toRadians(CAMERA_PITCH),t=Math.tan(Math.toRadians(ty));
        double up=Math.sin(pitch)+t*Math.cos(pitch);
        if(!Double.isFinite(tx+ty)||up>=-.025)return null;
        double scale=(radius-CAMERA_HEIGHT)/up;
        double forward=scale*(Math.cos(pitch)-t*Math.sin(pitch)),right=scale*Math.tan(Math.toRadians(tx));
        if(forward<.1||forward>2)return null;
        double h=pose.heading();
        return new double[]{pose.x()*.0254+forward*Math.cos(h)+right*Math.sin(h),
            pose.y()*.0254+forward*Math.sin(h)-right*Math.cos(h)};
    }
    /** Segment versus frame-foot boxes expanded by the default chassis footprint. */
    public static boolean safeApproach(double x,double y,double gx,double gy){
        for(int sx:new int[]{-1,1})for(int sy:new int[]{-1,1}){
            double cx=.00018832+sx*.602303,cy=sy*.4657255;
            double[] from={x,y},delta={gx-x,gy-y},center={cx,cy},extent={.047290/2+.2286,.057929/2+.2286};
            double lo=0,hi=1;
            for(int axis=0;axis<2;axis++){
                if(Math.abs(delta[axis])<1e-9){if(Math.abs(from[axis]-center[axis])>extent[axis]){lo=2;break;}}
                else {
                    double a=(center[axis]-extent[axis]-from[axis])/delta[axis],b=(center[axis]+extent[axis]-from[axis])/delta[axis];
                    lo=Math.max(lo,Math.min(a,b));hi=Math.min(hi,Math.max(a,b));
                }
            }
            if(lo<=hi)return false;
        }
        return true;
    }
    private Pose approach(double[] xy,Pose scan,double sign){
        double h=scan.heading(),x=xy[0]*sign,y=xy[1]*sign,gx,gy;
        if(x< -1.30){h=Math.toRadians(180+(sign<0?180:0));gx=Math.max(-1.53919,Math.min(-.245,x+.235));gy=Math.max(-1.564259,Math.min(1.564259,y));}
        else if(y< -1.30){h=Math.toRadians(270+(sign<0?180:0));gx=Math.max(-1.50,Math.min(-.245,x));gy=quadrantEnd<0?-1.53:Math.max(-1.53919,y+.235);}
        else if(y>1.30){h=Math.toRadians(90+(sign<0?180:0));gx=Math.max(-1.50,Math.min(-.245,x));gy=quadrantEnd>0?1.53:Math.min(1.53919,y-.235);}
        else if(avoidFrame&&y<-.60){h=Math.toRadians(90+(sign<0?180:0));gx=Math.max(-1.50,Math.min(-.245,x));gy=Math.min(-1.05,y-.26);}
        else if(avoidFrame&&y>.60){h=Math.toRadians(270+(sign<0?180:0));gx=Math.max(-1.50,Math.min(-.245,x));gy=Math.max(1.05,y+.26);}
        else {gx=Math.max(-.95,Math.min(-.245,(xy[0]-.235*Math.cos(h))*sign));if(x>-.42)gx=-.245;
            gy=Math.max(-1.53919,Math.min(1.53919,(xy[1]-.235*Math.sin(h))*sign));}
        return new Pose(gx*sign/.0254,gy*sign/.0254,h);
    }
    private boolean canReach(Pose from,Pose to){
        if(avoidFrame)return FrameClearance.clear(from.x()*.0254,from.y()*.0254)&&FrameClearance.clear(to.x()*.0254,to.y()*.0254);
        return safeApproach(from.x()*.0254,from.y()*.0254,to.x()*.0254,to.y()*.0254);
    }
    private boolean allowed(double[] xy,double sign){
        double x=xy[0]*sign,y=xy[1]*sign;
        return x>=-1.79&&x<=-.05&&Math.abs(y)>=.30&&Math.abs(y)<=1.79
            &&(quadrantEnd==0||y*quadrantEnd>=.30);
    }
    private void driveTo(Follower follower,Pose target,boolean blue){
        if(!avoidFrame){follower.hold(target);return;}
        if(routedGoal!=target){
            detour.clear();
            Pose from=follower.pose();
            int wallSign=quadrantEnd*(blue?-1:1);
            if(!returning&&target.y()*.0254*wallSign>1.36&&Math.sin(target.heading())*wallSign>.9){
                // Line up inside the wall spill before driving into it nose-first.
                // Sweep to the wall because wall occlusion can shorten the projected range.
                // A diagonal approach can knock the row across the centerline.
                Pose staging=new Pose(target.x(),wallSign*1.23/.0254,target.heading());
                detour.addAll(FrameClearance.path(from,staging,blue));
                from=staging;
            }
            detour.addAll(FrameClearance.path(from,target,blue));
            routedGoal=target;
        }
        while(detour.size()>1&&Math.hypot(follower.pose().x()-detour.peekFirst().x(),follower.pose().y()-detour.peekFirst().y())<.65)detour.removeFirst();
        follower.hold(detour.isEmpty()?target:detour.peekFirst());
    }
    public boolean update(Follower follower,CycleMechanisms mechanisms,LLResult frame,int pipeline,boolean blue,int wanted,double now){
        if(!fresh(frame,pipeline)){
            follower.stop();follower.drivetrain.stop();mechanisms.collect(false);emptySince=-1;status="Stale/wrong pipeline: stopped";return false;
        }
        Pose p=follower.pose();
        if(goal!=null){
            driveTo(follower,goal,blue);mechanisms.collect(!returning);
            double d=Math.hypot(p.x()-goal.x(),p.y()-goal.y());
            double e=Math.abs(Math.atan2(Math.sin(p.heading()-goal.heading()),Math.cos(p.heading()-goal.heading())));
            if(d<.55&&e<.04){
                if(arrived<0)arrived=now;
                if(now-arrived>.22){
                    if(!returning){attempts++;if(attempts>=wanted){mechanisms.collect(false);return true;}
                        // Reuse a short-lived pixel map to traverse the observed spill row,
                        // instead of driving back to the scan point after each ball.
                        if(clustered&&wanted>1&&!queued.isEmpty()&&now-plannedAt<3){
                            double[] next=queued.removeFirst();Pose nextGoal=approach(next,scan,blue?-1:1);
                            if(canReach(p,nextGoal)){
                                goal=nextGoal;attempted.add(next);arrived=-1;return false;
                            }
                        }
                        queued.clear();goal=scan;returning=true;}
                    else {goal=null;returning=false;}
                    arrived=-1;
                }
            }else arrived=-1;
            status=returning?"Backing up to rescan":"Approaching detected ball";return false;
        }
        mechanisms.collect(false);follower.hold(scan);
        double sign=blue?-1:1,best=Double.POSITIVE_INFINITY;
        double[] selected=null;
        List<double[]> observed=new ArrayList<>();
        if(clustered)for(LLResultTypes.DetectorResult ball:pollen(frame)){
            double[] xy=project(ball.getTargetXDegrees(),ball.getTargetYDegrees(),p,BALL_RADIUS);
            if(xy!=null&&allowed(xy,sign))observed.add(xy);
        }
        for(LLResultTypes.DetectorResult ball:pollen(frame)){
            double[] xy=project(ball.getTargetXDegrees(),ball.getTargetYDegrees(),p,BALL_RADIUS);
            if(xy==null)continue;
            // Confine pursuit to the alliance spill corridor; stay away from frame feet and center line.
            if(!allowed(xy,sign))continue;
            Pose candidate=approach(xy,scan,sign);if(avoidFlowers&&!clearOfPots(candidate.x()*.0254,candidate.y()*.0254))continue;double gx=candidate.x()*.0254*sign,gy=candidate.y()*.0254*sign;
            // Only the narrow inner corridor may pass the legs; approach outer balls from the end.
            if(!avoidFrame&&gx<-.28&&Math.abs(gy)<.80)continue;
            if(!canReach(p,candidate))continue;
            if(attempted.stream().anyMatch(a->Math.hypot(a[0]-xy[0],a[1]-xy[1])<.065))continue;
            double d=Math.hypot(xy[0]-p.x()*.0254,xy[1]-p.y()*.0254);
            // Prefer a pickup lane that covers multiple visible balls with one pass.
            if(clustered)for(double[] other:observed)if(Math.abs(other[0]-xy[0])<.20&&Math.abs(other[1]-xy[1])<.12)d-=.60;
            if(d<best){best=d;selected=xy;}
        }
        if(selected!=null){
            emptySince=-1;
            goal=approach(selected,scan,sign);attempted.add(selected);driveTo(follower,goal,blue);
            queued.clear();plannedAt=now;
            if(clustered&&wanted>1){
                for(double[] other:observed){
                    if(Math.hypot(other[0]-selected[0],other[1]-selected[1])<.065||Math.abs(other[1]-selected[1])>.15)continue;
                    Pose next=approach(other,scan,sign);
                    if(avoidFlowers&&!clearOfPots(next.x()*.0254,next.y()*.0254))continue;
                    if(!avoidFrame&&next.x()*sign*.0254<-.28&&Math.abs(next.y()*.0254)<.80)continue;
                    if(queued.stream().anyMatch(a->Math.hypot(a[0]-other[0],a[1]-other[1])<.065))continue;
                    if(canReach(goal,next))queued.addLast(other);
                }
            }
        }else {
            if(emptySince<0)emptySince=now;
            // A fresh empty search is not a measured inventory count; continue with whatever was acquired.
            if(now-emptySince>.4){status="No further safe targets in view";return true;}
        }
        status="Scanning neural pipeline "+pipeline;return false;
    }
}

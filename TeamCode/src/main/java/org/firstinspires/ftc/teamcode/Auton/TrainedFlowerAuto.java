package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.drivetrain.DrivePowers;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Auton.cycles.FrameClearance;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.FlowerArm;
import java.util.ArrayDeque;
import java.util.Deque;

/** Deterministic route exported from the user's trained flower policy; not neural inference.
 * No cameras, simulator state, invented inventory feedback or teammate are used by this OpMode. */
public abstract class TrainedFlowerAuto extends OpMode {
    protected abstract boolean blueAlliance();
    public static double FLOWER_COLLECT_SECONDS=2.0;
    public static double FLOWER_PUSH_POWER=.30;
    public static double FLOWER_PUSH_SECONDS=1.25;
    public static double SHOT_BATCH_TIMEOUT_SECONDS=8.0;
    public static double MAX_READY_FEED_SECONDS=2.0;
    public static final double NORMAL_TRAVEL_TOLERANCE_INCHES=3.0;
    public static final String EXPORT_VERSION="RAMP15_V3_TIMED_PUSH_3IN_ARM";
    private enum State { LOWER_RAMP, DRIVE, SHOOT, PUSH, COLLECT, FALLBACK, DONE }
    private State state=State.DONE;
    private final ElapsedTime clock=new ElapsedTime();
    private final Intake intake=new Intake();
    private final Shooter shooter=new Shooter();
    private final FlowerArm flowerArm=new FlowerArm();
    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private int batchStartShots,batchReleases;
    private Pose target;
    private Pose pushStart;
    private int leg,totalReleases,batches,emergencyPot=-1;
    private double began,stable=-1,lastShotLoop,readyFeedSeconds;
    private LimitedDrive limitedDrive;
    private static final class LimitedDrive implements Drivetrain {
        private final Drivetrain base;double cap=1;
        LimitedDrive(Drivetrain base){this.base=base;}
        public void drive(DrivePowers p,boolean manual){
            double demand=Math.abs(p.forward())+Math.abs(p.strafe())+Math.abs(p.turn());
            double scale=demand>cap?cap/demand:1;
            base.drive(new DrivePowers(p.forward()*scale,p.strafe()*scale,p.turn()*scale),manual);
        }
        public double maxScaling(DrivePowers a,DrivePowers b){return base.maxScaling(a,b);}
        public double interpolateVelocity(double a,double b,double t){return base.interpolateVelocity(a,b,t)*cap;}
        public double interpolateAcceleration(double a,double b,double t){return base.interpolateAcceleration(a,b,t)*cap;}
        public void stop(){base.stop();}public void stop(boolean brake){base.stop(brake);}
        public java.util.Map<String,Object> debug(){return base.debug();}
    }
    private String abortDetails="None",lastBatchMode="None";
    private boolean ready;
    private String result="Ready",parkingReason="None";
    private final Deque<Pose> fallback=new ArrayDeque<>();
    private static final double[][] CURVES={
        {-13.606197000,-61.569252000,-13.625674041,-54.803052186,-13.645151083,-48.036852371,-13.664628124,-41.270652557,1.570796327,1.535403000},
        {-13.664628124,-41.270652557,-23.939148416,-41.949464038,-34.213668708,-42.628275519,-44.488189000,-43.307087000,1.535403000,1.535403000},
        {-44.488189000,-43.307087000,-44.094488205,-36.669291596,-43.700787409,-30.031496191,-43.307086614,-23.393700787,1.535403000,3.141592654},
        {-43.307086614,-23.393700787,-47.316391076,-23.393700787,-51.325695538,-23.393700787,-55.335000000,-23.393700787,3.141592654,3.141592654},
        {-55.335000000,-23.393700787,-51.325695538,-23.393700787,-47.316391076,-23.393700787,-43.307086614,-23.393700787,3.141592654,3.141592654},
        {-43.307086614,-23.393700787,-43.700787409,-1.160104858,-44.094488205,21.073491071,-44.488189000,43.307087000,3.141592654,3.141593000},
        {-44.488189000,43.307087000,-34.213668247,42.628275535,-23.939147493,41.949464071,-13.664626740,41.270652606,3.141593000,4.747783000},
        {-13.664626740,41.270652606,-18.952270845,42.474398325,-24.239914950,43.678144045,-29.527559055,44.881889764,4.747783000,0.000000000},
        {-29.527559055,44.881889764,-27.482939632,44.356955381,-25.438320210,43.832020997,-23.393700787,43.307086614,0.000000000,1.570796327},
        {-23.393700787,43.307086614,-23.393700787,47.316391076,-23.393700787,51.325695538,-23.393700787,55.335000000,1.570796327,1.570796327},
        {-23.393700787,55.335000000,-23.393700787,51.325695538,-23.393700787,47.316391076,-23.393700787,43.307086614,1.570796327,1.570796327},
        {-23.393700787,43.307086614,-25.438320210,43.832020997,-27.482939632,44.356955381,-29.527559055,44.881889764,1.570796327,1.570796327},
        {-29.527559055,44.881889764,-24.239914950,43.678144045,-18.952270845,42.474398325,-13.664626740,41.270652606,1.570796327,4.747783000},
        {-13.664626740,41.270652606,-23.939147493,41.949464071,-34.213668247,42.628275535,-44.488189000,43.307087000,4.747783000,4.747783000},
        {-44.488189000,43.307087000,-50.000000000,37.270341333,-54.199475000,31.233595667,-59.055118000,25.196850000,4.747783000,4.712389000}
    };
    private Pose transform(double x,double y,double heading){
        double sign=blueAlliance()?-1:1;
        return new Pose(sign*x,sign*y,heading+(blueAlliance()?Math.PI:0));
    }
    private Pose nominalEnd(int i){double[] c=CURVES[i];return transform(c[6],c[7],c[9]);}
    private Pose parkPose(){return transform(-59.055118,25.196850,Math.PI*1.5);}
    @Override public void init(){
        Follower tuned=org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        limitedDrive=new LimitedDrive(tuned.drivetrain);
        follower=new Follower(tuned.localizer,limitedDrive,tuned.algorithm());
        pinpoint=hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        intake.init(hardwareMap);shooter.init(hardwareMap,intake);
        flowerArm.init(hardwareMap,true);ready=true;
        Pose start=transform(-13.606197,-61.569252,Math.PI/2);
        follower.setPose(start);follower.update();shooter.stop();intake.stop();
        telemetry.addLine("Load FOUR pollen. Both own-side flower pots must contain four pollen.");
        telemetry.addData("Start visualizer inches","%.4f, %.4f",start.x()+70.75,start.y()+70.75);
        telemetry.addData("Heading degrees",Math.toDegrees(start.heading()));
        telemetry.addData("Flower arm",flowerArm.getStatus());
        telemetry.addLine("START lowers the ramp to touchDown before driving.");
        telemetry.addLine(EXPORT_VERSION+": ramp tip 15 inches from center; quarter-inch wall gap.");telemetry.update();
    }
    @Override public void start(){
        clock.reset();leg=totalReleases=batches=0;emergencyPot=-1;parkingReason="None";
        follower.stop();follower.drivetrain.stop();
        if(!flowerArm.isAvailable()){finish("Flower arm unavailable: "+flowerArm.getStatus());return;}
        shooter.resetShotCount();shooter.spinUp();
        state=State.LOWER_RAMP;result="Lowering ramp to touchDown";
        flowerArm.down();flowerArm.update();report();
    }
    private int contactPot(){
        if(emergencyPot>=0)return emergencyPot;
        return leg==3||leg==4?3:leg==9||leg==10?0:-1;
    }
    private boolean flowerContactAllowed(Pose world){
        int pot=contactPot();
        if(pot!=0&&pot!=3)return false;
        if(state!=State.PUSH&&state!=State.COLLECT
            &&!(state==State.DRIVE&&(leg==4||leg==10))
            &&!(state==State.FALLBACK&&emergencyPot>=0))return false;
        double sign=blueAlliance()?-1:1,x=sign*world.x(),y=sign*world.y();
        double h=world.heading()-(blueAlliance()?Math.PI:0);
        double expected=pot==3?Math.PI:Math.PI/2;
        double headingError=Math.atan2(Math.sin(h-expected),Math.cos(h-expected));
        double forward=pot==3?-x:y,side=pot==3?y:x;
        // Only the selected flower's straight approach corridor permits ramp contact.
        return forward>=42.3&&forward<=56.085&&Math.abs(side+23.393700787)<=1.25
            &&Math.abs(headingError)<=Math.toRadians(3);
    }
    private double flowerForward(Pose world){
        double sign=blueAlliance()?-1:1;
        return contactPot()==3?-sign*world.x():sign*world.y();
    }
    private boolean clear(Pose world){
        double sign=blueAlliance()?-1:1,x=sign*world.x()*.0254,y=sign*world.y()*.0254;
        double h=world.heading()-(blueAlliance()?Math.PI:0),c=Math.cos(h),s=Math.sin(h);
        if(!Double.isFinite(x+y+h))return false;
        double minX=10,maxX=-10,minY=10,maxY=-10;
        for(double forward:new double[]{-.2286,.381})for(double right:new double[]{-.2286,.2286}){
            double xx=x+forward*c-right*s,yy=y+forward*s+right*c;
            minX=Math.min(minX,xx);maxX=Math.max(maxX,xx);minY=Math.min(minY,yy);maxY=Math.max(maxY,yy);
        }
        boolean contact=flowerContactAllowed(world);
        // The ramp intentionally hits the flower. Permit one inch of modeled tip
        // overlap at that wall only; other wall and frame limits stay active.
        double minWallX= -1.792859-(contact&&contactPot()==3?.0254:0);
        double maxWallY=1.792859+(contact&&contactPot()==0?.0254:0);
        if(maxX>-.005||minX<minWallX||maxY>maxWallY||minY< -1.792859)return false;
        if(maxX>=-.695&&minX<=.695&&maxY>=-.545&&minY<=.545)return false;
        double[][] pots={{-.5942,1.7283},{1.7283,.5942},{.5942,-1.7283},{-1.7283,-.5942}};
        for(int i=0;i<4;i++){
            double dx=pots[i][0]-x,dy=pots[i][1]-y,f=dx*c+dy*s,r=-dx*s+dy*c;
            double a=Math.max(0,Math.max(-.2286-f,f-.381)),b=Math.max(0,Math.abs(r)-.2286);
            if(Math.hypot(a,b)<.09&&!(contact&&i==contactPot())
                &&(i!=contactPot()||f<.26||f>.55||Math.abs(r)>.025))return false;
        }return true;
    }
    private void driveLeg(){
        if(leg>=CURVES.length){finish("Parked");return;}
        double[] c=CURVES[leg];Pose expected=transform(c[0],c[1],c[8]);
        Pose from=follower.pose();target=nominalEnd(leg);
        boolean retreat=leg==4||leg==10;
        // A normal leg can finish three inches from its nominal endpoint. Join
        // the next path from that measured pose; flower staging stays tighter.
        double startTolerance=leg==3||leg==9?1.1:NORMAL_TRAVEL_TOLERANCE_INCHES+.1;
        if(!retreat&&Math.hypot(from.x()-expected.x(),from.y()-expected.y())>startTolerance){park("Missed curve start");return;}
        if(leg==3||leg==9){
            if(!flowerArm.isAtDown()){finish("Flower push blocked: ramp is not down");return;}
            if(!Double.isFinite(FLOWER_PUSH_POWER)||FLOWER_PUSH_POWER<=0||FLOWER_PUSH_POWER>.45
                ||!Double.isFinite(FLOWER_PUSH_SECONDS)||FLOWER_PUSH_SECONDS<=0||FLOWER_PUSH_SECONDS>2){
                finish("Invalid flower push power/time");return;
            }
            follower.stop();follower.drivetrain.stop();
            state=State.PUSH;began=clock.seconds();pushStart=from;
            follower.manual(new DrivePowers(FLOWER_PUSH_POWER,0,0));
            collect();result="Timed forward flower push";return;
        }
        double sign=blueAlliance()?-1:1;
        Pose c1=transform(c[2],c[3],0),c2=transform(c[4],c[5],0);
        // Join from measured pose without a start-position jump; preserve the learned exit controls.
        c1=new Pose(c1.x()+from.x()-expected.x(),c1.y()+from.y()-expected.y());
        double turn=Math.atan2(Math.sin(target.heading()-from.heading()),Math.cos(target.heading()-from.heading()));
        target=new Pose(target.x(),target.y(),from.heading()+turn);
        state=State.DRIVE;began=clock.seconds();stable=-1;
        if(Math.hypot(target.x()-from.x(),target.y()-from.y())<.08){follower.hold(target);return;}
        // A timed contact may finish short of or beyond the old endpoint. Back
        // straight out from measured pose instead of requiring that endpoint.
        Path path=retreat?Paths.line(from,target).linear(from,target)
            :Paths.curve(from,c1,c2,target).linear(from,target);
        for(int i=0;i<=1000;i++)if(!clear(path.get(i/1000.0))){finish("Curve clearance interlock; inspect placement");return;}
        follower.follow(path);
    }
    private boolean at(double inches,double degrees,double speed){
        Pose p=follower.pose();double h=p.heading()-target.heading();
        return Math.hypot(p.x()-target.x(),p.y()-target.y())<inches
            &&Math.abs(Math.atan2(Math.sin(h),Math.cos(h)))<Math.toRadians(degrees)
            &&Math.hypot(follower.velocity().vx,follower.velocity().vy)<speed;
    }
    private boolean driveArrived(){
        if(leg==2||leg==4||leg==8||leg==10)return at(.75,2.0,2.5);
        return at(NORMAL_TRAVEL_TOLERANCE_INCHES,3.0,4.5);
    }
    private void collect(){intake.start();intake.stopWindmill();}
    private void arrived(){
        follower.hold(target);stable=-1;began=clock.seconds();
        if(leg==0||leg==6||leg==12){state=State.SHOOT;batchStartShots=shooter.getShotCount();batchReleases=0;readyFeedSeconds=0;lastShotLoop=clock.seconds();shooter.setFeedPermit(false);shooter.start();}
        else{leg++;driveLeg();}
    }
    private void finishPush(String why){
        follower.stop();follower.drivetrain.stop();state=State.COLLECT;began=clock.seconds();
        result=why;collect();
    }
    private double reserve(){
        Pose from=follower.pose();double seconds=1.0;
        if(contactPot()>=0){Pose stage=nominalEnd(contactPot()==3?2:8);seconds+=Math.hypot(from.x()-stage.x(),from.y()-stage.y())*.0254/.4+.5;from=stage;}
        for(Pose p:FrameClearance.path(from,parkPose(),blueAlliance())){
            seconds+=Math.hypot(from.x()-p.x(),from.y()-p.y())*.0254/1.0+.5;from=p;
        }return seconds;
    }
    private void park(String reason){
        abortDetails="curve="+(leg+1)+", batches="+batches+", releases="+batchReleases+", feed="+readyFeedSeconds+", speed="+shooter.getCurrentSpeed()+", "+targetErrors();
        follower.stop();follower.drivetrain.stop();shooter.stop();intake.stop();parkingReason=reason;result=reason;
        fallback.clear();Pose from=follower.pose();int pot=contactPot();
        emergencyPot=pot;
        if(pot>=0){Pose stage=nominalEnd(pot==3?2:8);fallback.add(stage);from=stage;}
        fallback.addAll(FrameClearance.path(from,parkPose(),blueAlliance()));
        state=State.FALLBACK;nextFallback();
    }
    private void nextFallback(){
        if(fallback.isEmpty()){finish("Parked");return;}
        target=fallback.remove();Pose from=follower.pose();
        Path path=Paths.line(from,target).linear(from,target);
        for(int i=0;i<=1000;i++)if(!clear(path.get(i/1000.0))){finish("Fallback clearance interlock");return;}
        follower.follow(path);began=clock.seconds();
    }
    @Override public void loop(){
        if(state==State.DONE)return;
        flowerArm.update();
        if(flowerArm.isFaulted()){finish("Flower arm fault: "+flowerArm.getStatus());return;}
        double now=clock.seconds();if(now>=29.7){finish("Autonomous deadline");return;}
        if(state==State.LOWER_RAMP){
            if(flowerArm.isAtDown()){result="Ramp down; starting route";driveLeg();}
            report();return;
        }
        limitedDrive.cap=state==State.PUSH?FLOWER_PUSH_POWER:1;
        follower.update();
        if(pinpoint.getDeviceStatus()!=GoBildaPinpointDriver.DeviceStatus.READY||!clear(follower.pose())){finish("Pose / clearance interlock");return;}
        if(state!=State.FALLBACK&&now+reserve()>=29.2){park("Time reserved for parking");report();return;}
        switch(state){
            case DRIVE:
                intake.stop();
                shooter.update(); // Includes Intake.update(): transfer ramping and jam protection.
                // Let Pedro finish travel/staging before changing states.
                if(driveArrived()){arrived();break;}
                if(now-began>6){park("Drive timeout");break;}
                break;
            case PUSH:
                collect();shooter.update();
                // Time ends the push; odometry supplies maximum-travel guards,
                // never a precision-arrival requirement or confirmation of pickup.
                if(now-began>=FLOWER_PUSH_SECONDS){finishPush("Timed flower push complete");break;}
                if(flowerForward(follower.pose())>=55.835
                    ||Math.hypot(follower.pose().x()-pushStart.x(),follower.pose().y()-pushStart.y())>=13){
                    finishPush("Flower push travel limit");break;
                }
                follower.manual(new DrivePowers(FLOWER_PUSH_POWER,0,0));
                break;
            case COLLECT:
                collect();shooter.update();
                if(now-began>=FLOWER_COLLECT_SECONDS){intake.stop();leg++;driveLeg();}
                break;
            case SHOOT:
                double dt=Math.max(0,Math.min(.1,now-lastShotLoop));lastShotLoop=now;
                boolean aimed=at(1,3,4.5);
                if(!aimed)stable=-1;else if(stable<0)stable=now;
                boolean permit=aimed&&now-stable>=.25;
                // Count only intervals commanded to feed while aim is continuously valid.
                if(permit&&"FEED".equals(shooter.getFeedStage()))readyFeedSeconds+=dt;
                shooter.setFeedPermit(permit);shooter.update();
                batchReleases=shooter.getShotCount()-batchStartShots;
                if(batchReleases>=4||readyFeedSeconds>=MAX_READY_FEED_SECONDS){
                    lastBatchMode=batchReleases>=4?"Shooter release estimate":"Timed feed fallback; count unconfirmed";
                    totalReleases+=batchReleases;batches++;shooter.holdFire();intake.stop();
                    if(leg==12)shooter.stop();
                    leg++;driveLeg();break;
                }
                if(now-began>SHOT_BATCH_TIMEOUT_SECONDS){park("Shooter readiness/feed timeout: "+batchReleases+" estimated releases");break;}
                result=permit?"Shooting; four releases requested":"Reacquiring shooting aim";break;
            case FALLBACK:
                if(at(emergencyPot>=0?.25:NORMAL_TRAVEL_TOLERANCE_INCHES,3,3)){emergencyPot=-1;nextFallback();}break;
            default: break;
        }report();
    }
    private String targetErrors(){
        if(target==null||follower==null)return "No target";
        Pose p=follower.pose();double h=p.heading()-target.heading();
        return "position error (in)="+Math.hypot(p.x()-target.x(),p.y()-target.y())
            +", heading error (deg)="+Math.toDegrees(Math.atan2(Math.sin(h),Math.cos(h)))
            +", drive speed (in/s)="+Math.hypot(follower.velocity().vx,follower.velocity().vy);
    }
    private void report(){
        telemetry.addData("Export version",EXPORT_VERSION);telemetry.addData("Abort details",abortDetails);
        telemetry.addData("Last batch completion",lastBatchMode);telemetry.addData("Ready feed seconds",readyFeedSeconds);telemetry.addData("Auto stage",state);telemetry.addData("Curve",leg+1);
        telemetry.addData("Auto result",result);telemetry.addData("Parking reason",parkingReason);
        telemetry.addData("Target errors",targetErrors());
        telemetry.addData("Flower arm",flowerArm.getStatus());
        telemetry.addData("Ramp switches",flowerArm.switchText());
        telemetry.addData("Flower push power / seconds","%.2f / %.2f",FLOWER_PUSH_POWER,FLOWER_PUSH_SECONDS);
        telemetry.addData("Elapsed (s)",clock.seconds());telemetry.addData("Completed shot batches",batches);
        telemetry.addData("Estimated releases",totalReleases+(state==State.SHOOT?batchReleases:0));
        telemetry.addLine("No basket-tip or ball-count sensor. Flower pickup is timed; tips are not confirmed.");
        shooter.addTelemetry(telemetry);telemetry.update();
    }
    private void finish(String why){if(!"Parked".equals(why)&&follower!=null)abortDetails="curve="+(leg+1)+", pose="+follower.pose()+", feed="+readyFeedSeconds+", velocity="+shooter.getCurrentSpeed();result=why;state=State.DONE;stop();report();}
    @Override public void stop(){
        flowerArm.stop();
        if(follower!=null){follower.stop();follower.drivetrain.stop();}
        if(ready){shooter.stop();intake.stop();}
    }
}

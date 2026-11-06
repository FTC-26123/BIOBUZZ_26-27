package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Shared bounded autonomous with collection, stationary/rolling fire, and a 29.7s cutoff. */
public abstract class ThirtySecondAuto extends OpMode {
    protected abstract CycleRoute.Start startingPoint();
    protected CycleRoute.Strategy strategy(){return CycleRoute.Strategy.FLOWERS;}
    protected CycleRoute.Lineup lineup(){return CycleRoute.Lineup.LEGACY;}
    private final SpillCollector collector=new SpillCollector();
    private final FlowerPickup flowerPickup=new FlowerPickup();
    private final GardenSweep garden=new GardenSweep();
    private MovingShotSolver.Solution movingSolution;
    private double lastSolve=-1;
    private int movingFeeds;
    private int pathReplans;
    private double velocityTime=-1,lastVx,lastVy,accelX,accelY;
    private boolean adaptive(){return route.strategy==CycleRoute.Strategy.ADAPTIVE_ALL||route.rushStyle();}
    private double rollingStart=-1;
    private double plannedReleaseHeading;
    private TransitPath transitPath;
    private boolean haveRollingAim;
    private String shotStatus="Waiting";
    private int emptyBase;
    private double cellSafeAt;
    private double raisedCellSince=-1;
    private boolean partnerTipConfirmed;
    private Follower follower;
    private CycleMechanisms mechanisms;
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A camera;
    private WebcamTags tags;
    private CycleRoute route;
    private CycleRoute.Stage stage;
    private final ElapsedTime timer=new ElapsedTime();
    private int index,shotBase;
    private double stageStart,settled=-1,lastProgress, lastX,lastY,finishedAt;
    private boolean done,parking;
    private java.util.Iterator<Pose> parkSteps;
    private java.util.Iterator<Pose> moveSteps;
    private Pose moveTarget;
    private String result="Ready",vision="Waiting";
    @Override public void init(){
        route=new CycleRoute(startingPoint(),strategy(),lineup()); follower=org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        collector.clustered=route.rushStyle();
        collector.avoidFlowers=route.strategy==CycleRoute.Strategy.NO_FLOWERS||route.lineup==CycleRoute.Lineup.PARTNER;
        collector.avoidFrame=route.avoidFrame();
        pinpoint=hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        follower.setPose(route.initial); follower.update();
        mechanisms=new CycleMechanisms(hardwareMap);
        camera=hardwareMap.get(Limelight3A.class,"limelight");camera.pipelineSwitch(VisionRules.POLLEN_PIPELINE);camera.setPollRateHz(100);camera.start();
        tags=new WebcamTags(hardwareMap);
        telemetry.addData("Place robot (m)",String.format("%.3f, %.3f",route.initial.x()*.0254,route.initial.y()*.0254));
        telemetry.addData("Pedro heading (deg)",Math.toDegrees(route.initial.heading()));
        telemetry.addLine("Verify hardware calibration: 4 pollen preloads, camera pitch +10 degrees, shot elevation 86 degrees.");
        telemetry.update();
    }
    @Override public void start(){timer.reset();index=0;stage=null;cellSafeAt=0;partnerTipConfirmed=false;done=false;parking=false;result="Running";next();}
    private void beginPark(){
        mechanisms.stop();parking=true;result="Parking";
        parkSteps=route.parkingPath(follower.pose()).iterator();nextPark();
    }
    private void nextPark(){
        if(!parkSteps.hasNext()){finish(route.partnerOpener()?"Holding inner lane for TeleOp":"Parked");return;}
        stage=new CycleRoute.Stage(route.partnerOpener()?"Return to inner handoff spot":"Park loading zone",parkSteps.next(),CycleRoute.Action.PARK,0,5);
        stageStart=timer.seconds();settled=-1;lastProgress=stageStart;
        lastX=follower.pose().x();lastY=follower.pose().y();
        followStage();
    }
    private void next(){
        // Load-event timing plus conservative flight/hinge allowance, not simulator tip state.
        if(route.rushStyle()&&stage!=null&&
            (stage.name().equals("Shoot preloads en route to near flower")||stage.name().equals("Shoot preloads toward garden")||stage.name().equals("Shoot while returning to near flower")))cellSafeAt=timer.seconds()+3.15;
        mechanisms.close();
        if(parking){nextPark();return;}
        if(index>=route.stages.size()){
            if(route.partnerOpener())finish("Holding inner lane for TeleOp");
            else beginPark();
            return;
        }
        // Bonus inventory is optional. Keep a measured-pose return budget for a partial pickup.
        if(route.rushStyle()&&index>=route.bonusIndex&&timer.seconds()+route.parkReserve(follower.pose())+.55>=29.5){beginPark();return;}
        stage=route.stages.get(index++);stageStart=timer.seconds();settled=-1;
        raisedCellSince=-1;
        if(route.rushStyle()&&index>route.bonusIndex)mechanisms.stop();
        double preparedSpeed=stage.action()==CycleRoute.Action.BONUS_FIRE?1320:index==1?CycleMechanisms.SPEED:2200;
        if(route.rushStyle()&&index<=route.bonusIndex){
            for(int upcoming=index-1;upcoming<route.bonusIndex;upcoming++){
                CycleRoute.Stage fire=route.stages.get(upcoming);
                if(fire.action()!=CycleRoute.Action.ROLL_NEAR&&fire.action()!=CycleRoute.Action.ROLL_FAR)continue;
                MovingShotSolver.Solution planned=MovingShotSolver.rolling(fire.pose(),new com.pedropathing.math.Velocity(0,0,0),route.blue,fire.action()==CycleRoute.Action.ROLL_NEAR,1100,.2,0,0);
                if(planned!=null)preparedSpeed=planned.ticks();break;
            }
        }
        mechanisms.target(preparedSpeed);
        shotBase=mechanisms.fired;lastProgress=stageStart;lastX=follower.pose().x();lastY=follower.pose().y();
        rollingStart=-1;haveRollingAim=false;emptyBase=mechanisms.emptyPulses;transitPath=new TransitPath(follower.pose(),stage.pose(),route.blue,route.avoidFrame());
        flowerPickup.reset();
        collector.reset(follower.pose());
        pathReplans+=garden.replans;garden.reset(follower.pose());movingSolution=null;lastSolve=-1;
        followStage();
    }
    private void followStage(){
        if(route.avoidFrame()&&(stage.action()==CycleRoute.Action.ROLL_NEAR||stage.action()==CycleRoute.Action.ROLL_FAR)){
            moveSteps=null;moveTarget=null;follower.hold(follower.pose());return;
        }
        if(route.avoidFrame()){
            moveSteps=FrameClearance.path(follower.pose(),stage.pose(),route.blue).iterator();advanceMove();
        }else{moveTarget=null;follower.follow(Paths.line(follower.pose(),stage.pose()).linear(follower.pose(),stage.pose()));}
    }
    private void advanceMove(){
        if(moveSteps!=null&&moveSteps.hasNext()){
            moveTarget=moveSteps.next();follower.follow(Paths.line(follower.pose(),moveTarget).linear(follower.pose(),moveTarget));
        }
    }
    @Override public void loop(){
        if(done){report();return;}
        double now=timer.seconds();
        if(now>=29.7){finish("30-second deadline");report();return;}
        follower.update();Pose p=follower.pose();
        if(route.avoidFrame()){
            if(!FrameClearance.clear(p.x()*.0254,p.y()*.0254)){finish("Frame clearance interlock");report();return;}
            if(moveSteps!=null&&moveSteps.hasNext()&&moveTarget!=null&&Math.hypot(p.x()-moveTarget.x(),p.y()-moveTarget.y())<.65)advanceMove();
        }
        int acknowledged=mechanisms.fired;mechanisms.update(now);
        if(mechanisms.fired>acknowledged&&Math.hypot(follower.velocity().vx,follower.velocity().vy)*.0254>.05)movingFeeds++;
        if(adaptive()){
            com.pedropathing.math.Velocity velocity=follower.velocity();double vx=velocity.vx*.0254,vy=velocity.vy*.0254;
            if(velocityTime>=0&&now-velocityTime>.004){double dt=now-velocityTime,blend=1-Math.exp(-dt/.10);
                accelX+=(Math.max(-4,Math.min(4,(vx-lastVx)/dt))-accelX)*blend;
                accelY+=(Math.max(-4,Math.min(4,(vy-lastVy)/dt))-accelY)*blend;
            }velocityTime=now;lastVx=vx;lastVy=vy;
        }
        if(pinpoint.getDeviceStatus()!=GoBildaPinpointDriver.DeviceStatus.READY||!Double.isFinite(p.x()+p.y()+p.heading())){
            finish("Localization fault");report();return;
        }
        boolean bonusPickup=route.rushStyle()&&index>route.bonusIndex;
        if(!parking&&now>=29.5-route.parkReserve(p)){beginPark();report();return;}
        if(!parking&&!bonusPickup)mechanisms.spin();
        if(stage.action()==CycleRoute.Action.ROLL_NEAR||stage.action()==CycleRoute.Action.ROLL_FAR){rolling(now,p);report();return;}
        if(stage.action()==CycleRoute.Action.SWEEP){
            if(garden.update(follower,mechanisms,camera.getLatestResult(),route.blue,now)||now-stageStart>stage.limit())next();
            vision=garden.status;report();return;
        }
        if(stage.action()==CycleRoute.Action.VISION_COLLECT||stage.action()==CycleRoute.Action.VISION_NECTAR){
            boolean collected=collector.update(follower,mechanisms,camera.getLatestResult(),VisionRules.POLLEN_PIPELINE,route.blue,stage.count(),now);
            vision=collector.status+"; attempts "+collector.attempts;
            if(collected||now-stageStart>stage.limit())next();
            report();return;
        }
        double distance=Math.hypot(p.x()-stage.pose().x(),p.y()-stage.pose().y());
        double error=Math.abs(Math.atan2(Math.sin(p.heading()-stage.pose().heading()),Math.cos(p.heading()-stage.pose().heading())));
        if(stage.action()==CycleRoute.Action.FLOWER_COLLECT){
            if(distance<.65&&error<Math.toRadians(3)){
                follower.hold(stage.pose());mechanisms.collect(true);
                if(flowerPickup.update(now,mechanisms.intakeSpeed(),stage.count())){next();report();return;}
            }else{flowerPickup.reset();mechanisms.collect(distance<6&&error<Math.toRadians(8));}
            vision="Drawing flower pollen; encoder transport and stack settling";
            if(now-stageStart>stage.limit())beginPark();report();return;
        }
        boolean precisionPickup=stage.name().contains("cleanup");
        if(route.rushStyle()&&stage.action()==CycleRoute.Action.COLLECT&&distance<(precisionPickup?.35:2.3)&&error<Math.toRadians(precisionPickup?1:3)){
            mechanisms.collect(true);if(settled<0)settled=now;if(now-settled>(precisionPickup?.18:.12))next();report();return;
        }
        if(Math.hypot(p.x()-lastX,p.y()-lastY)>.25){lastProgress=now;lastX=p.x();lastY=p.y();}
        if(distance<2&&!follower.holding())follower.hold(stage.pose());
        boolean precision=stage.action()==CycleRoute.Action.FIRE||stage.action()==CycleRoute.Action.BONUS_FIRE;
        if(precision){
            double releaseDelay=mechanisms.releaseDelay(now);
            if(lastSolve<0||now-lastSolve>.06){movingSolution=MovingShotSolver.solve(p,follower.velocity(),route.blue,index==1||stage.action()==CycleRoute.Action.BONUS_FIRE,releaseDelay);lastSolve=now;}
            MovingShotSolver.Solution direct=MovingShotSolver.evaluate(p,follower.velocity(),route.blue,index==1||stage.action()==CycleRoute.Action.BONUS_FIRE,mechanisms.speed(),accelX,accelY,releaseDelay);
            if(direct!=null)movingSolution=direct;
            if(movingSolution!=null&&distance<3.2){
                double h=movingSolution.heading();
                boolean fits=chassisFits(p,h);
                if(fits){
                    follower.hold(new Pose(stage.pose().x(),stage.pose().y(),h));
                    // Prefer the already recovered speed when its predicted entry is safe.
                    if(direct==null)mechanisms.aim(movingSolution.ticks());
                    double headingError=Math.abs(Math.atan2(Math.sin(p.heading()-h),Math.cos(p.heading()-h)));
                    int before=mechanisms.fired;double speed=Math.hypot(follower.velocity().vx,follower.velocity().vy)*.0254;
                    mechanisms.feed(now,direct!=null&&now-stageStart>.20&&headingError<Math.toRadians(.5)&&visionReady());
                    if(mechanisms.fired>before&&speed>.05)movingFeeds++;
                    if(mechanisms.fired-shotBase>=stage.count())next();
                    if(now-stageStart>stage.limit())finish("Stage timeout: "+stage.name());report();return;
                }
            }
            mechanisms.feed(now,false);
            if(now-stageStart>stage.limit())finish("Stage timeout: "+stage.name());report();return;
        }
        double moveTolerance=.65;
        if(adaptive()&&stage.action()==CycleRoute.Action.MOVE&&index<route.stages.size()){
            double turn=Math.abs(Math.atan2(Math.sin(route.stages.get(index).pose().heading()-stage.pose().heading()),Math.cos(route.stages.get(index).pose().heading()-stage.pose().heading())));
            if(turn<Math.toRadians(5))moveTolerance=2.0;
        }
        boolean at=distance<(precision?.10:moveTolerance)&&error<Math.toRadians(precision?.25:2);
        if(!at){settled=-1;mechanisms.close();
            if(adaptive()&&stage.action()==CycleRoute.Action.COLLECT&&distance<8&&error<Math.toRadians(8))mechanisms.collect(true);
            if(now-lastProgress>2.5){finish("No path progress");}
        }else{
            if(settled<0){settled=now;follower.hold(stage.pose());}
            if(stage.action()==CycleRoute.Action.MOVE||stage.action()==CycleRoute.Action.PARK){next();}
            else if(stage.action()==CycleRoute.Action.COLLECT){mechanisms.collect(true);if(now-settled>.18)next();}
            else if(stage.action()==CycleRoute.Action.WAIT_TIP){
                mechanisms.close();vision="Waiting for released balls to reach the floor";
                LLResult floor=camera.getLatestResult();
                boolean groundTarget=false;
                if(SpillCollector.fresh(floor,VisionRules.POLLEN_PIPELINE))for(com.qualcomm.hardware.limelightvision.LLResultTypes.DetectorResult blob:SpillCollector.pollen(floor)){
                    double[] xy=SpillCollector.project(blob.getTargetXDegrees(),blob.getTargetYDegrees(),p,SpillCollector.BALL_RADIUS);
                    if(xy!=null&&xy[0]*(route.blue?-1:1)<-.05){groundTarget=true;break;}
                }
                if(now>=cellSafeAt&&now-settled>.8&&groundTarget)next();
            }
            else {
                boolean target=visionReady();
                double fireSettle=route.strategy==CycleRoute.Strategy.TIPPED_POLLEN?.25:.35;
                mechanisms.feed(now,now-settled>fireSettle&&target);
                if(mechanisms.fired-shotBase>=stage.count()){mechanisms.close();if(now-settled>.45)next();}
            }
        }
        if(!done&&now-stageStart>stage.limit()){
            if(stage.action()==CycleRoute.Action.WAIT_TIP||bonusPickup)beginPark();
            else finish("Stage timeout: "+stage.name());
        }
        report();
    }
    private void rolling(double now,Pose p){
        boolean near=stage.action()==CycleRoute.Action.ROLL_NEAR;
        double delay=mechanisms.releaseDelay(now);
        com.pedropathing.math.Velocity measured=follower.velocity();double vx=measured.vx*.0254,vy=measured.vy*.0254;
        double dx=vx*delay,dy=vy*delay,futureVx=vx,futureVy=vy;
        double phase=rollingStart<0?0:now-rollingStart;
        TransitPath.Sample current=transitPath.sample(phase);TransitPath.Sample future=transitPath.sample(phase+delay);
        if(rollingStart>=0){
            // Tracking error decays as the drivetrain catches the reference; it does not
            // persist at constant velocity throughout an accelerating transfer.
            double decay=Math.exp(-delay/.16),lag=.16*(1-decay);
            dx=future.x()-current.x()+(vx-current.vx())*lag;
            dy=future.y()-current.y()+(vy-current.vy())*lag;
            futureVx=future.vx()+(vx-current.vx())*decay;
            futureVy=future.vy()+(vy-current.vy())*decay;
        }
        Pose releasePose=new Pose(p.x()+dx/.0254,p.y()+dy/.0254,p.heading()+measured.omega*delay);
        com.pedropathing.math.Velocity releaseVelocity=new com.pedropathing.math.Velocity(futureVx/.0254,futureVy/.0254,measured.omega);
        boolean atDestination=rollingStart>=0&&phase>=transitPath.duration()&&Math.hypot(p.x()-stage.pose().x(),p.y()-stage.pose().y())*.0254<.06;
        // A stationary fallback aims for zero yaw, rather than chasing each measured yaw
        // correction and feeding it back into the next heading target. The actual gate below
        // still validates measured translation/yaw at release before opening the feeder.
        com.pedropathing.math.Velocity planningVelocity=atDestination&&Math.hypot(vx,vy)<.08
            ?new com.pedropathing.math.Velocity(futureVx/.0254,futureVy/.0254,0):releaseVelocity;
        if(lastSolve<0||now-lastSolve>.06){
            double aimU=route.strategy==CycleRoute.Strategy.NO_FLOWERS&&near?-.16+(mechanisms.fired%3)*.04:(mechanisms.fired%3-1)*.07;
            movingSolution=MovingShotSolver.rolling(releasePose,planningVelocity,route.blue,near,mechanisms.speed(),0,0,0,aimU*(route.blue?-1:1));
            if(movingSolution!=null){plannedReleaseHeading=movingSolution.heading();haveRollingAim=true;}
            lastSolve=now;
        }
        double h=haveRollingAim?plannedReleaseHeading:stage.pose().heading();
        double aimError=Math.abs(Math.atan2(Math.sin(p.heading()-h),Math.cos(p.heading()-h)));
        // At the inner corridor, translate outward before a turn whose swept body
        // would cross the center line, even when its final heading would fit.
        double turn=Math.atan2(Math.sin(h-p.heading()),Math.cos(h-p.heading()));boolean turnFits=true;
        for(int i=0;i<=8;i++)turnFits&=chassisFits(p,p.heading()+turn*i/8);
        if(rollingStart<0&&(!haveRollingAim||!turnFits||(aimError<Math.toRadians(3)&&Math.abs(measured.omega)<.12)))rollingStart=now;
        TransitPath.Sample target=transitPath.sample(rollingStart<0?0:now-rollingStart+.10);
        follower.hold(new Pose(target.x()/.0254,target.y()/.0254,turnFits?h:p.heading()));
        if(movingSolution!=null)mechanisms.aim(movingSolution.ticks());
        MovingShotSolver.Solution direct=MovingShotSolver.evaluate(releasePose,releaseVelocity,route.blue,near,mechanisms.predictedReleaseSpeed(now),0,0,0);
        double speed=Math.hypot(vx,vy),futureSpeed=Math.hypot(futureVx,futureVy);
        boolean arrived=atDestination;
        boolean fast=speed>.40&&futureSpeed>.75;
        // Finish an unsafe-in-transit shot at the actual pickup approach, never by making extra passes.
        boolean settledAtPickup=arrived&&speed<.08&&Math.abs(measured.omega)<.10;
        boolean fresh=rollingVisionReady(now,p,near,arrived),complete=mechanisms.fired-shotBase>=stage.count()||mechanisms.emptyPulses-emptyBase>=1;
        boolean permit=!complete&&now>=cellSafeAt&&rollingStart>=0&&(fast||settledAtPickup)&&movingSolution!=null&&direct!=null&&fresh&&chassisFits(p,h)&&chassisFits(releasePose,h);
        mechanisms.feedPredicted(now,permit,fresh&&!complete);
        shotStatus=String.format(java.util.Locale.US,"v %.2f -> %.2f, remaining %.2f m, aim %.1f deg, RPM ticks %.0f/%.0f, valid %s, feed %s",speed,futureSpeed,current.remaining(),Math.toDegrees(Math.atan2(Math.sin(releasePose.heading()-h),Math.cos(releasePose.heading()-h))),mechanisms.speed(),movingSolution==null?0:movingSolution.ticks(),direct!=null,permit);
        if(complete&&arrived){next();return;}
        if(now-stageStart>stage.limit()){
            if(route.lineup==CycleRoute.Lineup.PARTNER||route.rushStyle()&&!vision.contains("No fresh"))beginPark();
            else finish("Rolling stage timeout: "+stage.name());
        }
    }
    private boolean chassisFits(Pose p,double heading){
        double raw=heading-Math.PI/2,c=Math.cos(raw),s=Math.sin(raw);
        double x=p.x()*.0254+.000349899754*c-.02483252995*s,y=p.y()*.0254+.000349899754*s+.02483252995*c;
        double ex=(.400591312*Math.abs(c)+.4576660022*Math.abs(s))/2,ey=(.400591312*Math.abs(s)+.4576660022*Math.abs(c))/2;
        return Math.abs(x)+ex<1.790&&Math.abs(y)+ey<1.790&&x*(route.blue?-1:1)+ex<-.002;
    }
    private boolean rollingVisionReady(double now,Pose pose,boolean near,boolean arrived){
        if(route.lineup!=CycleRoute.Lineup.PARTNER||near)return visionReady();
        if(!arrived){
            vision="Approaching partner shot position";
            return false;
        }
        boolean raised=tags.readyRaisedFar(route.blue?38:30,pose,route.blue);
        vision=tags.status;
        if(partnerTipConfirmed)return raised;
        if(!raised){
            raisedCellSince=-1;
            return false;
        }
        if(raisedCellSince<0)raisedCellSince=now;
        if(now-raisedCellSince>=.65){
            partnerTipConfirmed=true;
            return true;
        }
        vision="Waiting for raised far cell to settle";
        return false;
    }

    private boolean visionReady(){
        boolean initialCell=index==1||stage.action()==CycleRoute.Action.BONUS_FIRE||stage.action()==CycleRoute.Action.ROLL_NEAR;
        int first=route.blue?(initialCell?42:38):(initialCell?34:30);
        boolean ready=tags.ready(first);vision=tags.status;return ready;
    }
    private void finish(String why){finishedAt=timer.seconds();done=true;result=why;follower.stop();follower.drivetrain.stop();mechanisms.stop();camera.stop();if(tags!=null)tags.close();}
    private void report(){
        telemetry.addData("Auto stage",stage==null?"Init":stage.name());telemetry.addData("Auto result",result);
        telemetry.addData("Auto elapsed",done?finishedAt:timer.seconds());telemetry.addData("Confirmed feed events",mechanisms.fired);
        telemetry.addData("Vision interlock",vision);telemetry.addData("Pedro pose",follower.pose());
        telemetry.addData("Feeder release estimate (s)",mechanisms.releaseDelay(timer.seconds()));telemetry.addData("Last confirmed feed (s)",mechanisms.lastFeedDuration());
        telemetry.addData("Fast shot gate",shotStatus);
        if(adaptive()){telemetry.addData("Moving feed events",movingFeeds);telemetry.addData("Live path replans",pathReplans+garden.replans);}
        telemetry.update();
    }
    @Override public void stop(){if(follower!=null){follower.stop();follower.drivetrain.stop();}if(mechanisms!=null)mechanisms.stop();if(camera!=null)camera.stop();if(tags!=null)tags.close();}
}

package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import java.util.Iterator;

/** Shoot the opening, collect our garden and carry pollen to park; existing Shoot 4 and Park is unchanged. */
public abstract class GardenParkAuto extends OpMode {
    protected abstract boolean blueAlliance();
    private enum State { DRIVE, SWEEP, CAPTURE, SHOOT, PARK, DONE }
    private final ElapsedTime timer=new ElapsedTime();
    private final Intake intake=new Intake();
    private final Shooter shooter=new Shooter();
    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private DcMotorEx intakeEncoder;
    private GardenParkRoute route;
    private PreloadShotCounter counter;
    private Iterator<Pose> waypoints;
    private Pose target,sweepFrom;
    private State state=State.DONE;
    private int stageIndex,releases;
    private double phaseStarted,sweepSeconds;
    private boolean mechanismsReady;
    private String result="Ready";

    @Override public void init(){
        route=new GardenParkRoute(blueAlliance());
        follower=org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        pinpoint=hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        intake.init(hardwareMap);shooter.init(hardwareMap,intake);mechanismsReady=true;
        intakeEncoder=hardwareMap.get(DcMotorEx.class,"intake");
        shooter.stop();intake.stop();follower.setPose(route.initial);follower.update();
        telemetry.addLine("Four pollen preloads; collect ONLY our alliance garden, then park.");
        telemetry.addLine("No flowers or cameras. Keep the working Shoot 4 and Park available.");
        telemetry.addData("Start (field-centered m)","%.3f, %.3f",route.initial.x()*.0254,route.initial.y()*.0254);
        telemetry.addData("Heading (deg)",Math.toDegrees(route.initial.heading()));telemetry.update();
    }
    @Override public void start(){
        timer.reset();stageIndex=releases=0;result="Running";enterStage();
    }
    private CycleRoute.Stage stage(){return route.stages.get(stageIndex);}
    private void enterStage(){
        shooter.stop();intake.stop();phaseStarted=timer.seconds();
        if(stageIndex>=route.stages.size()){park("Garden sweep complete; carrying pollen to TeleOp");return;}
        if(stage().action()==CycleRoute.Action.SWEEP){
            // A moving hold target limits the floor sweep without retuning the team's follower.
            state=State.SWEEP;sweepFrom=follower.pose();target=stage().pose();
            sweepSeconds=Math.hypot(target.x()-sweepFrom.x(),target.y()-sweepFrom.y())*.0254
                /GardenParkRoute.SWEEP_METERS_PER_SECOND;
            if(!FrameClearance.segment(sweepFrom.x()*.0254,sweepFrom.y()*.0254,target.x()*.0254,target.y()*.0254)){
                park("Sweep clearance fault");return;
            }
            intake.start();intake.stopWindmill();
        }else{
            state=State.DRIVE;waypoints=FrameClearance.path(follower.pose(),stage().pose(),route.blue).iterator();nextWaypoint();
        }
    }
    private void nextWaypoint(){target=waypoints.next();Pose from=follower.pose();follower.follow(Paths.line(from,target).linear(from,target));}
    private boolean at(Pose goal){
        Pose p=follower.pose();double angle=p.heading()-goal.heading();
        return Math.hypot(p.x()-goal.x(),p.y()-goal.y())<.65
            &&Math.abs(Math.atan2(Math.sin(angle),Math.cos(angle)))<Math.toRadians(2)
            &&Math.hypot(follower.velocity().vx,follower.velocity().vy)<3;
    }
    private void beginShooting(){
        if(timer.seconds()+1.8+route.parkReserve(follower.pose())>29.5){park("Insufficient shooting and parking time");return;}
        state=State.SHOOT;phaseStarted=timer.seconds();counter=new PreloadShotCounter();
        follower.hold(stage().pose());shooter.start(); // Existing default 1500 ticks/s, existing feeder logic.
    }
    private void park(String why){
        shooter.stop();intake.stop();result=why;state=State.PARK;
        waypoints=route.parkingPath(follower.pose()).iterator();nextWaypoint();
    }
    @Override public void loop(){
        if(state==State.DONE)return;
        double now=timer.seconds();if(now>=29.7){finish("Autonomous deadline");return;}
        follower.update();Pose p=follower.pose();
        if(pinpoint.getDeviceStatus()!=GoBildaPinpointDriver.DeviceStatus.READY||!Double.isFinite(p.x()+p.y()+p.heading())){
            finish("Localization fault");return;
        }
        if(!FrameClearance.clear(p.x()*.0254,p.y()*.0254)){finish("Frame clearance interlock");return;}
        if(state!=State.PARK&&now>=29.5-route.parkReserve(p)){park("Time reserved for parking");report();return;}
        if(state!=State.PARK&&state!=State.SHOOT&&now-phaseStarted>stage().limit()){
            park("Stage timeout; parking");report();return;
        }
        switch(state){
            case SWEEP:
                double u=sweepSeconds<=0?1:Math.min(1,(now-phaseStarted)/sweepSeconds);
                follower.hold(new Pose(sweepFrom.x()+(target.x()-sweepFrom.x())*u,
                    sweepFrom.y()+(target.y()-sweepFrom.y())*u,target.heading()));
                if(u>=1&&at(target)){state=State.CAPTURE;phaseStarted=now;}
                break;
            case CAPTURE:
                if(now-phaseStarted>=GardenParkRoute.CAPTURE_DWELL_SECONDS){stageIndex++;enterStage();}
                break;
            case SHOOT:
                if(!at(stage().pose())){park("Shooting pose lost");break;}
                counter.update(now,shooter.getTargetLaunchPower(),shooter.getCurrentSpeed(),
                    intakeEncoder.getVelocity(),"Shooting".equals(shooter.getShooterStatus()));
                if(counter.shots>=4){
                    releases+=counter.shots;
                    stageIndex++;enterStage();
                }else if(now-phaseStarted>=stage().limit())park("Incomplete batch; parking");
                else shooter.update();
                break;
            case DRIVE: case PARK:
                if(Math.hypot(p.x()-target.x(),p.y()-target.y())<2&&!follower.holding())follower.hold(target);
                if(at(target)){
                    if(waypoints.hasNext())nextWaypoint();
                    else if(state==State.PARK)finish("Parked");
                    else if(stage().action()==CycleRoute.Action.FIRE)beginShooting();
                    else{stageIndex++;enterStage();}
                }
                break;
            default: break;
        }
        report();
    }
    private void finish(String why){result=why;state=State.DONE;stop();report();}
    private void report(){
        telemetry.addData("Auto stage",state);telemetry.addData("Route",stageIndex<route.stages.size()?stage().name():"Park");
        telemetry.addData("Auto result",result);telemetry.addData("Elapsed (s)",timer.seconds());
        telemetry.addData("Estimated releases",releases+(state==State.SHOOT&&counter!=null?counter.shots:0));
        telemetry.addLine("Garden pickup is timed; up to four pollen expected for TeleOp.");
        shooter.addTelemetry(telemetry);telemetry.update();
    }
    @Override public void stop(){
        if(follower!=null){follower.stop();follower.drivetrain.stop();}
        if(mechanismsReady){shooter.stop();intake.stop();}
    }
}

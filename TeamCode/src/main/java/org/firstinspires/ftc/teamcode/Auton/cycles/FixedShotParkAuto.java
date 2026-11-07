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

/** Drive to the fixed shooting mark, use the team's Shooter, then park. No cameras. */
public abstract class FixedShotParkAuto extends OpMode {
    protected abstract CycleRoute.Start startingPoint();
    private enum State { CLEAR, APPROACH, SHOOT, PARK, DONE }
    private final ElapsedTime timer=new ElapsedTime();
    private final Intake intake=new Intake();
    private final Shooter shooter=new Shooter();
    private PreloadShotCounter counter;
    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private DcMotorEx intakeEncoder;
    private CycleRoute route;
    private Iterator<Pose> waypoints;
    private Pose target;
    private State state=State.DONE;
    private double shotStarted;
    private boolean mechanismsReady;
    private String result="Ready";

    @Override public void init(){
        route=new CycleRoute(startingPoint(),CycleRoute.Strategy.NO_FLOWERS,CycleRoute.Lineup.FOUR_PARK);
        follower=org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        pinpoint=hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        intake.init(hardwareMap);shooter.init(hardwareMap,intake);
        mechanismsReady=true;
        intakeEncoder=hardwareMap.get(DcMotorEx.class,"intake");
        shooter.stop();intake.stop();
        follower.setPose(route.initial);follower.update();
        telemetry.addData("Place robot (m)","%.3f, %.3f",route.initial.x()*.0254,route.initial.y()*.0254);
        telemetry.addData("Heading (deg)",Math.toDegrees(route.initial.heading()));
        telemetry.addData("Shot visualizer (in)","%.4f, %.4f",route.stages.get(1).pose().x()+70.75,
            route.stages.get(1).pose().y()+70.75);
        telemetry.addData("Shooter default (ticks/s)",shooter.getTargetLaunchPower());
        telemetry.addLine("Load four pollen. No Limelight or webcam required.");telemetry.update();
    }
    @Override public void start(){
        timer.reset();counter=new PreloadShotCounter();state=State.CLEAR;result="Running";
        drive(route.stages.get(0).pose());
    }
    private void drive(Pose destination){
        waypoints=FrameClearance.path(follower.pose(),destination,route.blue).iterator();nextWaypoint();
    }
    private void nextWaypoint(){
        target=waypoints.next();Pose from=follower.pose();
        follower.follow(Paths.line(from,target).linear(from,target));
    }
    private boolean at(Pose p,Pose goal){
        double angle=p.heading()-goal.heading();
        return Math.hypot(p.x()-goal.x(),p.y()-goal.y())<.65
            &&Math.abs(Math.atan2(Math.sin(angle),Math.cos(angle)))<Math.toRadians(2)
            &&Math.hypot(follower.velocity().vx,follower.velocity().vy)<3;
    }
    private void park(String reason){
        shooter.stop();intake.stop();state=State.PARK;result=reason;
        waypoints=route.parkingPath(follower.pose()).iterator();nextWaypoint();
    }
    @Override public void loop(){
        if(state==State.DONE)return;
        double now=timer.seconds();
        if(now>=29.7){finish("Autonomous deadline");return;}
        follower.update();Pose p=follower.pose();
        if(pinpoint.getDeviceStatus()!=GoBildaPinpointDriver.DeviceStatus.READY
            ||!Double.isFinite(p.x()+p.y()+p.heading())){finish("Localization fault");return;}
        if(!FrameClearance.clear(p.x()*.0254,p.y()*.0254)){finish("Frame clearance interlock");return;}
        if(state!=State.PARK&&now>=29.5-route.parkReserve(p)){park("Time reserved for parking");report();return;}
        if(state==State.SHOOT){
            if(!at(p,route.stages.get(1).pose())){park("Shooting pose lost");report();return;}
            // Count the preceding feed interval before Shooter may pause on a speed dip.
            counter.update(now,shooter.getTargetLaunchPower(),shooter.getCurrentSpeed(),
                intakeEncoder.getVelocity(),"Shooting".equals(shooter.getShooterStatus()));
            if(counter.shots>=4)park("Four estimated releases; parking");
            else if(now-shotStarted>=route.stages.get(1).limit())park("Shooting timeout; parking");
            else shooter.update();
        }else{
            double distance=Math.hypot(p.x()-target.x(),p.y()-target.y());
            if(distance<2&&!follower.holding())follower.hold(target);
            if(at(p,target)){
                if(waypoints.hasNext())nextWaypoint();
                else if(state==State.CLEAR){state=State.APPROACH;drive(route.stages.get(1).pose());}
                else if(state==State.APPROACH){
                    state=State.SHOOT;shotStarted=now;follower.hold(route.stages.get(1).pose());
                    shooter.start(); // Default 1500 ticks/s; Shooter owns PIDF and feeder readiness.
                }else finish("Parked");
            }
        }
        report();
    }
    private void finish(String reason){result=reason;state=State.DONE;stop();report();}
    private void report(){
        telemetry.addData("Auto stage",state);telemetry.addData("Auto result",result);
        telemetry.addData("Elapsed (s)",timer.seconds());
        telemetry.addData("Estimated releases",counter==null?0:counter.shots);
        shooter.addTelemetry(telemetry);telemetry.update();
    }
    @Override public void stop(){
        if(follower!=null){follower.stop();follower.drivetrain.stop();}
        if(mechanismsReady){shooter.stop();intake.stop();}
    }
}

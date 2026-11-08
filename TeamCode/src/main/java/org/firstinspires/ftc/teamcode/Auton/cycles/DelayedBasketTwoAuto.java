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

/** Aim/spin up during first 15 s, then release four preloads into an already raised Basket 2. */
public abstract class DelayedBasketTwoAuto extends OpMode {
    protected abstract boolean blueAlliance();
    private enum State { DRIVE, WAIT, SHOOT, PARK, DONE }
    private final ElapsedTime timer=new ElapsedTime();
    private final Intake intake=new Intake();
    private final Shooter shooter=new Shooter();
    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private DcMotorEx intakeEncoder;
    private DelayedBasketTwoRoute route;
    private PreloadShotCounter counter;
    private Iterator<Pose> waypoints;
    private Pose target;
    private State state=State.DONE;
    private double stableSince=-1,firstFeedAt=-1;
    private boolean mechanismsReady;
    private String result="Ready",parkingReason="None";

    @Override public void init(){
        route=new DelayedBasketTwoRoute(blueAlliance());
        follower=org.firstinspires.ftc.teamcode.pedro.Constants.create(hardwareMap);
        pinpoint=hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        intake.init(hardwareMap);shooter.init(hardwareMap,intake);mechanismsReady=true;
        intakeEncoder=hardwareMap.get(DcMotorEx.class,"intake");
        shooter.stop();intake.stop();follower.setPose(route.initial);follower.update();
        telemetry.addLine("BASKET 2 WALL START: load four pollen. Teammate must raise Basket 2 before 15 s.");
        telemetry.addData("Start visualizer (in)","%.4f, %.4f",route.initial.x()+70.75,route.initial.y()+70.75);
        telemetry.addData("Start heading (deg)",Math.toDegrees(route.initial.heading()));
        telemetry.addData("Shot visualizer (in)","%.4f, %.4f",route.shot.x()+70.75,route.shot.y()+70.75);
        telemetry.addLine("Drive/aim and spin up during wait. Intake/transfer/windmill stay OFF until 15 s.");telemetry.update();
    }
    @Override public void start(){
        timer.reset();counter=new PreloadShotCounter();stableSince=firstFeedAt=-1;
        state=State.DRIVE;result="Aiming for Basket 2";parkingReason="None";
        shooter.start();waypoints=route.approachPath().iterator();nextWaypoint();
    }
    private void nextWaypoint(){
        target=waypoints.next();Pose from=follower.pose();follower.follow(Paths.line(from,target).linear(from,target));
    }
    private boolean at(Pose p,Pose goal,double inches,double degrees,double speed){
        double turn=Math.atan2(Math.sin(p.heading()-goal.heading()),Math.cos(p.heading()-goal.heading()));
        return Math.hypot(p.x()-goal.x(),p.y()-goal.y())<inches&&Math.abs(turn)<Math.toRadians(degrees)
            &&Math.hypot(follower.velocity().vx,follower.velocity().vy)<speed;
    }
    private void waitForAim(){
        state=State.WAIT;stableSince=-1;follower.hold(route.shot);shooter.updateSpinupOnly();
        result="Waiting for 15 s and settled aim";
    }
    private void park(String why){
        shooter.stop();intake.stop();state=State.PARK;result=parkingReason=why;
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
        counter.update(now,shooter.getTargetLaunchPower(),shooter.getCurrentSpeed(),intakeEncoder.getVelocity(),
            state==State.SHOOT&&"Shooting".equals(shooter.getShooterStatus()));
        if(state!=State.PARK&&now>=29.5-route.parkReserve(p)){park("Time reserved for parking");report();return;}
        if(state!=State.PARK&&counter.shots>=4){park("Four estimated releases; mechanisms off");report();return;}
        // Two seconds after feeding may begin, stop being picky about the exact mark.
        boolean late=now>=DelayedBasketTwoRoute.FEED_NOT_BEFORE_SECONDS+2;
        switch(state){
            case DRIVE: case PARK:
                if(state==State.DRIVE){
                    shooter.updateSpinupOnly();
                    // Slow to settle: once close enough, hold the mark and wait there instead of giving up.
                    if(now>8&&at(p,route.shot,3.0,5,6)){waitForAim();break;}
                }
                if(Math.hypot(p.x()-target.x(),p.y()-target.y())<2&&!follower.holding())follower.hold(target);
                // Corridor points only need to be passed cleanly once slow or parking; the final point stays tight.
                boolean relaxed=waypoints.hasNext()&&(state==State.PARK||now>8);
                if(relaxed?at(p,target,1.5,5,6):at(p,target,1.0,2,3)){
                    if(waypoints.hasNext())nextWaypoint();
                    else if(state==State.PARK)finish("Parked");
                    else waitForAim();
                }
                break;
            case WAIT:
                shooter.updateSpinupOnly();
                if(late?at(p,route.shot,3.0,5,6):at(p,route.shot,1.0,2,3)){if(stableSince<0)stableSince=now;}
                else stableSince=-1;
                if(stableSince>=0&&DelayedBasketTwoRoute.readyToFeed(now,now-stableSince)){
                    state=State.SHOOT;result="Shooting Basket 2";feed(now);
                }
                break;
            case SHOOT:
                // Brief settling corrections pause feeding and reacquire aim instead of instantly parking.
                if(!(late?at(p,route.shot,3.5,6,8):at(p,route.shot,1.0,3,4.5))){waitForAim();break;}
                feed(now);
                break;
            default: break;
        }
        report();
    }
    private void feed(double now){
        if(now<DelayedBasketTwoRoute.FEED_NOT_BEFORE_SECONDS){waitForAim();return;}
        shooter.update();
        if(firstFeedAt<0&&"Shooting".equals(shooter.getShooterStatus()))firstFeedAt=now;
    }
    private void finish(String why){result=why;state=State.DONE;stop();report();}
    private void report(){
        telemetry.addData("Auto stage",state);telemetry.addData("Auto result",result);
        telemetry.addData("Parking reason",parkingReason);telemetry.addData("Elapsed (s)",timer.seconds());
        telemetry.addData("First feed command (s)",firstFeedAt);
        telemetry.addData("Estimated releases",counter==null?0:counter.shots);
        telemetry.addLine("Teammate must raise Basket 2. Basket state is not sensed.");
        shooter.addTelemetry(telemetry);telemetry.update();
    }
    @Override public void stop(){
        if(follower!=null){follower.stop();follower.drivetrain.stop();}
        if(mechanismsReady){shooter.stop();intake.stop();}
    }
}

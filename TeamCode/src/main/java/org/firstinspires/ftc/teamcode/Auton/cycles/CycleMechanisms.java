package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.qualcomm.robotcore.hardware.*;

/** Nonblocking flywheel recovery and single-shot windmill pulses. No simulator state access. */
public final class CycleMechanisms {
    private final DcMotorEx flywheel;
    private final DcMotorEx intake;
    private final DcMotor windmill;
    private final FeederModel feeder=new FeederModel();
    private final CRServo transferServo;
    private double readySince=-1, opened=-1, lastShot=-10;
    private boolean armed;
    private boolean awaitingLoad;
    private boolean cancelled;
    private double waitUntil;
    private double targetSpeed=SPEED;
    private double armedSpeed;
    private double retryAt;
    private double commanded,lastObserved,lastUpdate=-1;
    public int fired;
    public int emptyPulses;
    public static final double SPEED=2100;
    public CycleMechanisms(HardwareMap hw) {
        flywheel=hw.get(DcMotorEx.class,"shooter"); intake=hw.get(DcMotorEx.class,"intake");
        windmill=hw.get(DcMotor.class,"windmill"); transferServo=hw.get(CRServo.class,"transferServo");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        windmill.setDirection(DcMotorSimple.Direction.FORWARD);
        windmill.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        windmill.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stop();
    }
    public double intakeSpeed(){return intake.getVelocity();}
    public double speed(){return flywheel.getVelocity();}
    public double releaseDelay(double now){return feeder.remaining(now,intake.getVelocity(),lastShot);}
    public double lastFeedDuration(){return feeder.lastObserved;}
    private void requestVelocity(double ticks){commanded=ticks;flywheel.setVelocity(ticks);}
    private double feedCommand(double now){return FlywheelForecast.command(speed(),targetSpeed,releaseDelay(now));}
    public double predictedReleaseSpeed(double now){return FlywheelForecast.after(speed(),feedCommand(now),releaseDelay(now));}
    /** Observe before planning so confirmed shots and remaining travel use current encoders. */
    public void update(double now){
        feeder.observe(now,intake.getVelocity());
        double measured=speed(),expected=lastUpdate<0?measured:FlywheelForecast.after(lastObserved,commanded,Math.max(0,now-lastUpdate));
        lastObserved=measured;lastUpdate=now;
        // A commanded transient before substantial feeder travel cannot be a released ball.
        double nominalTicks=FeederModel.PATH/(Math.PI*FeederModel.DIAMETER*FeederModel.TRANSFER)*FeederModel.TICKS_PER_REV;
        if(opened>=0&&armed&&feeder.progressTicks()>nominalTicks*.70&&expected-measured>Math.max(35,expected*.035)){fired++;lastShot=now;feeder.confirm(now);close();}
    }
    public void target(double ticks){targetSpeed=ticks;close();}
    public void aim(double ticks){if(Math.abs(targetSpeed-ticks)>3){if(Math.abs(targetSpeed-ticks)>20)readySince=-1;targetSpeed=ticks;}}
    public void spin(){if(opened<0)requestVelocity(targetSpeed);}
    public void collect(boolean on){
        intake.setPower(on?-FeederModel.POWER:0);
        transferServo.setPower(on?org.firstinspires.ftc.teamcode.subsystems.Intake.TRANSFER_SERVO_POWER:0);
        windmill.setPower(0);
    }
    public void close(){collect(false);opened=-1;armed=awaitingLoad=false;readySince=-1;feeder.cancel();}
    private void continuePulse(double now,boolean permit){
        // Close after one predicted ball, independently of load-detection latency. Waiting
        // with the windmill running could otherwise feed a second ball before the first is acknowledged.
        if(!awaitingLoad&&(!permit||now-opened>feeder.lastPrediction+.04)){
            collect(false);awaitingLoad=true;cancelled=!permit;waitUntil=now+.12;feeder.pause();
        }
        if(awaitingLoad&&now>=waitUntil){if(!cancelled)emptyPulses++;close();retryAt=now+.04;}
    }
    /** Counts flywheel load events, not elapsed-time guesses. Missing load aborts at stage timeout. */
    public void feed(double now,boolean permit) {
        update(now);
        spin();
        double velocity=speed();
        // Account for a load that already occurred even if the next camera/trajectory
        // interlock closes. Permission only controls future feeding.
        if(opened>=0){
            requestVelocity(targetSpeed);
            continuePulse(now,permit);
            return;
        }
        if(!permit){collect(false);opened=-1;armed=false;
            if(Math.abs(velocity-targetSpeed)<25){if(readySince<0)readySince=now;}else readySince=-1;
            return;
        }
        if(Math.abs(velocity-targetSpeed)<25){if(readySince<0)readySince=now;}
        else readySince=-1;
        if(readySince>=0&&now-readySince>.12&&now-lastShot>.4&&now>=retryAt){
            opened=now;feeder.begin(now,lastShot);armed=true;armedSpeed=targetSpeed;collect(true);windmill.setPower(org.firstinspires.ftc.teamcode.subsystems.Intake.WINDMILL_IN);
        }
    }
    /** Predictive feeding: trajectory validity replaces the fixed speed/settle window.
     * Encoder load still acknowledges each shot; a closed interlock cancels the next feed. */
    public void feedPredicted(double now,boolean permit){
        feedPredicted(now,permit,false);
    }
    public void feedPredicted(double now,boolean permit,boolean prepare){
        update(now);
        double velocity=speed();
        if(opened>=0){
            requestVelocity(feedCommand(now));
            continuePulse(now,permit);
            return;
        }
        // Adjust for motor response throughout transport, targeting RPM at release.
        // The trajectory gate verifies the forecast rather than freezing current RPM.
        requestVelocity(feedCommand(now));
        if(!permit){collect(prepare);return;}
        // Start transport during recovery/spacing, rather than waiting and then paying transit again.
        if(now-lastShot>.07&&now>=retryAt){opened=now;feeder.begin(now,lastShot);armed=true;armedSpeed=velocity;collect(true);windmill.setPower(org.firstinspires.ftc.teamcode.subsystems.Intake.WINDMILL_IN);}
    }
    public void stop(){requestVelocity(0);close();}
}

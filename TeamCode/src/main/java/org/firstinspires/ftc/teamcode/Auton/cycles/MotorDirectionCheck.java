package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;

/** Held-control, low-power hardware check independent of cameras and Pedro tuning. */
@TeleOp(name="26123 Motor Direction Check",group="26123 Hardware")
public final class MotorDirectionCheck extends OpMode {
    private CycleMechanisms mechanisms;
    private DcMotorEx intake,shooter;
    private DcMotor windmill;
    private DcMotor[] drive;
    @Override public void init(){
        mechanisms=new CycleMechanisms(hardwareMap);
        intake=hardwareMap.get(DcMotorEx.class,"intake");
        windmill=hardwareMap.get(DcMotor.class,"windmill");
        shooter=hardwareMap.get(DcMotorEx.class,"shooter");
        drive=new DcMotor[]{hardwareMap.get(DcMotor.class,"frontLeftMotor"),
                hardwareMap.get(DcMotor.class,"frontRightMotor"),
                hardwareMap.get(DcMotor.class,"backLeftMotor"),
                hardwareMap.get(DcMotor.class,"backRightMotor")};
        for(int i=0;i<drive.length;i++){
            drive[i].setDirection(i%2==0?DcMotorSimple.Direction.REVERSE:DcMotorSimple.Direction.FORWARD);
            drive[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            drive[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);drive[i].setPower(0);
        }
    }
    @Override public void init_loop(){telemetry.addLine("Support robot before wheel checks. Hold RB and one control after Start.");telemetry.update();}
    @Override public void loop(){
        boolean enabled=gamepad1.right_bumper;
        boolean roller=enabled&&gamepad1.a;
        boolean transfer=enabled&&!gamepad1.a&&gamepad1.b;
        boolean launch=enabled&&!gamepad1.a&&!gamepad1.b&&gamepad1.x;
        boolean wheels=enabled&&!gamepad1.a&&!gamepad1.b&&!gamepad1.x&&gamepad1.dpad_up;
        intake.setPower(roller?-.25:0);windmill.setPower(transfer?.25:0);shooter.setVelocity(launch?600:0);
        for(DcMotor motor:drive)motor.setPower(wheels?.15:0);
        telemetry.addLine("Hold RB + A: intake; B: windmill; X: shooter; D-pad up: drive forward.");
        telemetry.addLine("One check at a time. Gate stays closed. Release RB to stop.");
        telemetry.addData("intake power / ticks",intake.getPower()+" / "+intake.getVelocity());
        telemetry.addData("windmill power",windmill.getPower());
        telemetry.addData("shooter ticks/sec",shooter.getVelocity());
        telemetry.addData("Drive directions","left REVERSE; right FORWARD");telemetry.update();
    }
    @Override public void stop(){if(mechanisms!=null)mechanisms.stop();if(drive!=null)for(DcMotor motor:drive)if(motor!=null)motor.setPower(0);}
}

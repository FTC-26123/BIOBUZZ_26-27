package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Camera-only diagnostic. No drivetrain or mechanism commands. */
@TeleOp(name="26123 Limelight AprilTag Check",group="26123 Vision")
public final class LimelightTagCheck extends OpMode {
    private Limelight3A camera;
    private LimelightTags tags;
    @Override public void init(){
        camera=hardwareMap.get(Limelight3A.class,"limelight");
        camera.pipelineSwitch(VisionRules.SHOT_PIPELINE);camera.setPollRateHz(100);camera.start();
        tags=new LimelightTags(camera);
    }
    @Override public void init_loop(){loop();}
    @Override public void loop(){
        LLResult frame=camera.getLatestResult();
        telemetry.addData("AprilTag pipeline",frame==null?"Waiting":frame.getPipelineIndex());
        telemetry.addData("Fresh tag frame",SpillCollector.fresh(frame,VisionRules.SHOT_PIPELINE));
        for(int first:new int[]{30,34,38,42})telemetry.addData("Basket tags "+first,tags.ready(first));
        telemetry.addData("Lens height (in)",LimelightMount.HEIGHT_METERS/.0254);
        telemetry.addData("Lens forward offset (in)",LimelightMount.FORWARD_METERS/.0254);
        telemetry.addData("Camera upward pitch (deg)",LimelightMount.PITCH_DEGREES);
        telemetry.addLine("AprilTags only. No webcam or pollen detector required. No motors commanded.");
        telemetry.update();
    }
    @Override public void stop(){if(camera!=null)camera.stop();}
}

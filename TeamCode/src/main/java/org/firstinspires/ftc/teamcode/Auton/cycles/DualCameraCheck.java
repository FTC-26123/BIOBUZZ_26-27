package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Camera-only check: no drivetrain, shooter or follower tuning needed. */
@TeleOp(name="26123 Dual Camera Check",group="26123 Vision")
public final class DualCameraCheck extends OpMode {
    private Limelight3A pollen;
    private WebcamTags tags;
    @Override public void init(){
        pollen=hardwareMap.get(Limelight3A.class,"limelight");
        pollen.pipelineSwitch(VisionRules.POLLEN_PIPELINE);pollen.setPollRateHz(100);pollen.start();
        tags=new WebcamTags(hardwareMap);
    }
    @Override public void init_loop(){loop();}
    @Override public void loop(){
        LLResult result=pollen.getLatestResult();
        telemetry.addData("Pollen pipeline",result==null?"Waiting":result.getPipelineIndex());
        telemetry.addData("Fresh pollen frame",SpillCollector.fresh(result,VisionRules.POLLEN_PIPELINE));
        telemetry.addData("yellow_pollen detections",SpillCollector.pollen(result).size());
        for(int first:new int[]{30,34,38,42}){
            telemetry.addData("Webcam cell "+first,tags.ready(first));
        }
        telemetry.addData("Webcam status",tags.status);
        telemetry.addLine("Camera check only. No motors are commanded.");telemetry.update();
    }
    @Override public void stop(){if(pollen!=null)pollen.stop();if(tags!=null)tags.close();}
}

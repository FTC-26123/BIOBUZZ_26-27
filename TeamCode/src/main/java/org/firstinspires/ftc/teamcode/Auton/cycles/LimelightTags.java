package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.limelightvision.*;

/** Basket checks share the existing Limelight. No USB webcam or VisionPortal. */
public final class LimelightTags {
    private final Limelight3A camera;
    private final LimelightShotPose geometry=new LimelightShotPose();
    public String status="Waiting for Limelight AprilTags";
    public LimelightTags(Limelight3A camera){this.camera=camera;}
    public boolean ready(int first){
        LLResult frame=camera.getLatestResult();
        if(!SpillCollector.fresh(frame,VisionRules.SHOT_PIPELINE)||!frame.isValid()||frame.getFiducialResults()==null){
            status="No fresh Limelight AprilTag frame";return false;
        }
        for(LLResultTypes.FiducialResult tag:frame.getFiducialResults())if(VisionRules.expectedTag(tag.getFiducialId(),first)){
            status="Limelight verified basket tags "+first;return true;
        }
        status="Wrong basket tags - holding fire";return false;
    }
    public boolean readyRaisedFar(int first,Pose robot,boolean blue){return readyRaisedCell(first,robot,blue,false);}
    public boolean readyRaisedCell(int first,Pose robot,boolean blue,boolean near){return height(first,robot,blue,near,true);}
    public boolean readyLoweredCell(int first,Pose robot,boolean blue,boolean near){return height(first,robot,blue,near,false);}
    private boolean height(int first,Pose robot,boolean blue,boolean near,boolean raised){
        int expected=blue?(near?42:38):(near?34:30);
        if(first!=expected){status="Wrong basket assignment";return false;}
        boolean valid=geometry.observed(camera.getLatestResult(),robot,new Velocity(0,0,0),blue,near,0,0,raised)!=null;
        status=valid?(raised?"Limelight basket raised":"Limelight basket lowered"):geometry.status;
        return valid;
    }
}

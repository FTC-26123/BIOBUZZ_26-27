package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.math.*;
import com.qualcomm.hardware.limelightvision.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import java.util.*;

public class LimelightMountChecks {
    static int checks;
    static final double[] OFFSETS = {-6.5,-2.75,2.75,6.5};
    static void check(boolean pass,String message){checks++;if(!pass)throw new AssertionError(message);}
    static LLResult frame(Pose actual,Velocity velocity,boolean blue,boolean near,double age,int pipeline,double height){
        double sign=blue?-1:1,end=near?-1:1,pitch=Math.toRadians(43);
        int first=blue?(near?42:38):(near?34:30);
        double h=actual.heading()-velocity.omega*age,c=Math.cos(h),s=Math.sin(h);
        double rx=actual.x()*.0254-velocity.vx*.0254*age+.1778*c,ry=actual.y()*.0254-velocity.vy*.0254*age+.1778*s;
        List<LLResultTypes.FiducialResult> tags=new ArrayList<>();
        for(int k=0;k<4;k++){
            double dx=sign*(-.32385-end*OFFSETS[k]*.0254)-rx,dy=sign*end*(height<1?.289414654:.327341117)-ry;
            double forward=dx*c+dy*s,right=dx*s-dy*c,up=height-.254;
            double camForward=forward*Math.cos(pitch)+up*Math.sin(pitch);
            double down=forward*Math.sin(pitch)-up*Math.cos(pitch);
            Pose3D camera=new Pose3D(new Position(DistanceUnit.METER,right,down,camForward,0),
                new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0,0));
            tags.add(new LLResultTypes.FiducialResult(first+k,camera,null,0,0));
        }
        return new LLResult(pipeline,0,tags,List.of(),null,null,age,()->0);
    }
    public static void main(String[] args){
        LimelightShotPose vision=new LimelightShotPose();
        for(boolean blue:new boolean[]{false,true})for(boolean near:new boolean[]{false,true}){
            double sign=blue?-1:1,end=near?-1:1,x=-1.13*sign,y=.95*sign*end;
            double h=Math.atan2(.392176*sign*end-y,-.32385*sign-x);
            Pose actual=new Pose(x/.0254,y/.0254,h);
            Velocity v=new Velocity(12*sign,-8*sign,.35);
            Pose drifted=new Pose(actual.x()+.12/.0254,actual.y()-.07/.0254,h);
            Pose measured=vision.measured(frame(actual,v,blue,near,.06,1,1.261519),drifted,v,blue,near,0,0);
            check(measured!=null,"expected raised tags accepted");
            check(Math.hypot(measured.x()-actual.x(),measured.y()-actual.y())*.0254<.03,"camera pitch, tag offset, alliance and latency recover actual position");
            LLResult exact=frame(actual,v,blue,near,.06,1,1.261519);
            Position t=exact.getFiducialResults().get(0).getTargetPoseCameraSpace().getPosition();
            Pose deterministic=LimelightShotPose.fromCamera(exact.getFiducialResults().get(0).getFiducialId(),t.x,t.y,t.z,.06,drifted,v,blue,near,0,0);
            check(deterministic!=null && Math.hypot(deterministic.x()-actual.x(),deterministic.y()-actual.y())<1e-6,"exact capture projection and motion compensation");
            Pose accelerationOrigin=new Pose(actual.x()+.5*2*.06*.06/.0254,actual.y()-.5*1.5*.06*.06/.0254,h);
            LLResult acceleratedFrame=frame(accelerationOrigin,v,blue,near,.06,1,1.261519);
            Position acceleratedTag=acceleratedFrame.getFiducialResults().get(0).getTargetPoseCameraSpace().getPosition();
            Pose accelerated=LimelightShotPose.fromCamera(acceleratedFrame.getFiducialResults().get(0).getFiducialId(),acceleratedTag.x,acceleratedTag.y,acceleratedTag.z,.06,drifted,v,blue,near,2,-1.5);
            check(accelerated!=null && Math.hypot(accelerated.x()-actual.x(),accelerated.y()-actual.y())<1e-6,"acceleration during capture delay");
            check(vision.rangeMeters>0,"measured range available");
            check(vision.measured(frame(actual,v,blue,near,0,7,1.261519),drifted,v,blue,near,0,0)==null,"pollen pipeline cannot authorize a shot");
            check(vision.measured(frame(actual,v,blue,near,.25,1,1.261519),drifted,v,blue,near,0,0)==null,"stale measurement rejected");
            check(vision.measured(frame(actual,v,blue,!near,0,1,1.261519),drifted,v,blue,near,0,0)==null,"other basket rejected");
            check(vision.measured(frame(actual,v,blue,near,0,1,.91),drifted,v,blue,near,0,0)==null,"lowered basket rejected");
            check(vision.observed(frame(actual,v,blue,near,0,1,.91),drifted,v,blue,near,0,0,false)!=null,"lower stop observed by Limelight");
            check(vision.observed(frame(actual,v,blue,near,0,1,1.261519),drifted,v,blue,near,0,0,false)==null,"raised basket cannot confirm lower stop");
            Pose badOdo=new Pose(drifted.x()+2/.0254,drifted.y(),h);
            check(vision.measured(frame(actual,v,blue,near,0,1,1.261519),badOdo,v,blue,near,0,0)==null,"inconsistent solve rejected");
        }
        double[] ticks=new double[2];
        for(int i=0;i<2;i++){
            double x=i==0?-1.13:-1.20,y=-.85;
            Pose actual=new Pose(x/.0254,y/.0254,Math.atan2(-.392176-y,-.32385-x));
            Pose measured=vision.measured(frame(actual,new Velocity(0,0,0),false,true,0,1,1.261519),actual,new Velocity(0,0,0),false,true,0,0);
            MovingShotSolver.Solution shot=MovingShotSolver.rolling(measured,new Velocity(0,0,0),false,true,1700,0,0,0);
            check(shot!=null,"86-degree solution reachable");
            ticks[i]=shot.ticks();
        }
        check(Math.abs(ticks[0]-ticks[1])>10,"measured Limelight range changes commanded shooting speed");
        check(MovingShotSolver.LAUNCH_ANGLE_DEGREES==86,"measured elevation preserved");
        double saved=LimelightMount.FORWARD_METERS;
        LimelightMount.FORWARD_METERS=Double.NaN;
        check(vision.measured(null,new Pose(0,0,0),new Velocity(0,0,0),false,true,0,0)==null,"unconfigured mount holds shots");
        LimelightMount.FORWARD_METERS=saved;
        System.out.println("PASS "+checks+" Limelight geometry, freshness, basket, motion and range-to-power checks; speed ticks "+Arrays.toString(ticks));
    }
}

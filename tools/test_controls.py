"""Run behavior checks on production control classes without robot hardware.
Requires JDK 17+ on PATH, or JAVA_HOME. SDK fakes only implement used hardware calls.
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode'
STUBS = {
'com/qualcomm/robotcore/hardware/DcMotorSimple.java': '''package com.qualcomm.robotcore.hardware; public class DcMotorSimple { public enum Direction {FORWARD,REVERSE} public void setDirection(Direction d) {} }''',
'com/qualcomm/robotcore/hardware/DcMotor.java': '''package com.qualcomm.robotcore.hardware; public class DcMotor extends DcMotorSimple { public enum ZeroPowerBehavior {BRAKE} public double power; public void setPower(double p){power=p;} public void setZeroPowerBehavior(ZeroPowerBehavior b){} }''',
'com/qualcomm/robotcore/hardware/DcMotorEx.java': '''package com.qualcomm.robotcore.hardware; public class DcMotorEx extends DcMotor { public double velocity, commanded; public void setVelocity(double v){commanded=v;} public double getVelocity(){return velocity;} }''',
'com/qualcomm/robotcore/hardware/Servo.java': '''package com.qualcomm.robotcore.hardware; public class Servo { public double position; public void setPosition(double p){position=p;} }''',
'com/qualcomm/robotcore/hardware/HardwareMap.java': '''package com.qualcomm.robotcore.hardware; import java.util.*; public class HardwareMap { public Map<String,Object> devices=new HashMap<>(); public <T>T get(Class<T> c,String n){return c.cast(devices.get(n));} public <T>T tryGet(Class<T> c,String n){return get(c,n);} }''',
'com/qualcomm/robotcore/hardware/Gamepad.java': '''package com.qualcomm.robotcore.hardware; public class Gamepad { public boolean right_bumper; public double right_trigger; public boolean aWasPressed(){return false;} public boolean bWasPressed(){return false;} public boolean dpadUpWasPressed(){return false;} public boolean dpadDownWasPressed(){return false;} }''',
'android/util/Log.java': '''package android.util; public class Log { public static int d(String t,String s){return 0;} }''',
'org/firstinspires/ftc/robotcore/external/navigation/DistanceUnit.java': '''package org.firstinspires.ftc.robotcore.external.navigation; public enum DistanceUnit {INCH}''',
'org/firstinspires/ftc/robotcore/external/navigation/AngleUnit.java': '''package org.firstinspires.ftc.robotcore.external.navigation; public enum AngleUnit {DEGREES}''',
'org/firstinspires/ftc/robotcore/external/navigation/Pose2D.java': '''package org.firstinspires.ftc.robotcore.external.navigation; public class Pose2D { double x,y,h; public Pose2D(DistanceUnit u,double x,double y,AngleUnit a,double h){this.x=x;this.y=y;this.h=h;} public double getX(DistanceUnit u){return x;} public double getY(DistanceUnit u){return y;} public double getHeading(AngleUnit u){return h;} }''',
'org/firstinspires/ftc/teamcode/odometry/GoBildaPinpointDriver.java': '''package org.firstinspires.ftc.teamcode.odometry; import org.firstinspires.ftc.robotcore.external.navigation.*; public class GoBildaPinpointDriver { public int reads; public enum GoBildaOdometryPods {goBILDA_4_BAR_POD} public enum EncoderDirection {FORWARD,REVERSED} public void update(){reads++;} public Pose2D getPosition(){return new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.DEGREES,0);} public void setOffsets(double x,double y){} public void setEncoderResolution(GoBildaOdometryPods p){} public void setEncoderDirections(EncoderDirection x,EncoderDirection y){} public void resetPosAndIMU(){} }''',
}
STUBS.update({
'com/qualcomm/robotcore/util/ElapsedTime.java': 'package com.qualcomm.robotcore.util; public class ElapsedTime { public static double now; private double start; public void reset(){start=now;} public double seconds(){return now-start;} public double time(){return seconds();} }',
'com/qualcomm/hardware/limelightvision/LLResult.java': 'package com.qualcomm.hardware.limelightvision; import java.util.*; import org.firstinspires.ftc.robotcore.external.navigation.*; public class LLResult { public boolean valid=true; public long stale; public double timestamp,tx,ty,ta; public int pipeline=1; public Pose3D pose=new Pose3D(); public List<LLResultTypes.FiducialResult> tags=new ArrayList<>(); public boolean isValid(){return valid;} public long getStaleness(){return stale;} public double getTimestamp(){return timestamp;} public int getPipelineIndex(){return pipeline;} public double getTx(){return tx;} public double getTy(){return ty;} public double getTa(){return ta;} public Pose3D getBotpose(){return pose;} public Pose3D getBotpose_MT2(){return pose;} public List<LLResultTypes.FiducialResult> getFiducialResults(){return tags;} }',
'com/qualcomm/hardware/limelightvision/LLResultTypes.java': 'package com.qualcomm.hardware.limelightvision; import org.firstinspires.ftc.robotcore.external.navigation.*; public class LLResultTypes { public static class FiducialResult { public int id; public FiducialResult(int id){this.id=id;} public int getFiducialId(){return id;} public Pose3D pose=new Pose3D(); public Pose3D getTargetPoseCameraSpace(){return pose;} } }',
'com/qualcomm/hardware/limelightvision/Limelight3A.java': 'package com.qualcomm.hardware.limelightvision; public class Limelight3A { public LLResult result; public int reads; public LLResult getLatestResult(){reads++;return result;} public void pipelineSwitch(int p){} public void start(){} public void stop(){} public void updateRobotOrientation(double d){} }',
'com/qualcomm/hardware/rev/RevHubOrientationOnRobot.java': 'package com.qualcomm.hardware.rev; public class RevHubOrientationOnRobot { public enum LogoFacingDirection {RIGHT} public enum UsbFacingDirection {FORWARD} public RevHubOrientationOnRobot(LogoFacingDirection l,UsbFacingDirection u){} }',
'com/qualcomm/robotcore/hardware/IMU.java': 'package com.qualcomm.robotcore.hardware; import com.qualcomm.hardware.rev.*; import org.firstinspires.ftc.robotcore.external.navigation.*; public class IMU { public static class Parameters { public Parameters(RevHubOrientationOnRobot o){} } public void initialize(Parameters p){} public YawPitchRollAngles getRobotYawPitchRollAngles(){return new YawPitchRollAngles();} }',
'org/firstinspires/ftc/robotcore/external/navigation/YawPitchRollAngles.java': 'package org.firstinspires.ftc.robotcore.external.navigation; public class YawPitchRollAngles {public double getYaw(){return 0;} }',
'org/firstinspires/ftc/robotcore/external/navigation/Position.java': 'package org.firstinspires.ftc.robotcore.external.navigation; public class Position { public double x,y,z=1; public Position toUnit(DistanceUnit u){return this;} }',
'org/firstinspires/ftc/robotcore/external/navigation/Pose3D.java': 'package org.firstinspires.ftc.robotcore.external.navigation; public class Pose3D { public Position position=new Position(); public Position getPosition(){return position;} public Orientation getOrientation(){return new Orientation();} public static class Orientation { public double getYaw(AngleUnit u){return 0;} public double getPitch(AngleUnit u){return 0;} public double getRoll(AngleUnit u){return 0;} } }',
'org/firstinspires/ftc/robotcore/external/Telemetry.java': 'package org.firstinspires.ftc.robotcore.external; public class Telemetry {public void addData(String s,Object v){} }',
'com/pedropathing/follower/Follower.java': 'package com.pedropathing.follower; public class Follower {public double turn; public void manual(double x,double y,double t){turn=t;} }',
'com/pedropathing/ivy/Command.java': 'package com.pedropathing.ivy; import java.util.function.*; public class Command {public Runnable start,execute; public BooleanSupplier done; public Consumer<Object> end; public static Command build(){return new Command();} public Command setStart(Runnable r){start=r;return this;} public Command setExecute(Runnable r){execute=r;return this;} public Command setDone(BooleanSupplier r){done=r;return this;} public Command setEnd(Consumer<Object> r){end=r;return this;} }',
})
STUBS['org/firstinspires/ftc/robotcore/external/navigation/DistanceUnit.java'] = 'package org.firstinspires.ftc.robotcore.external.navigation; public enum DistanceUnit {INCH,METER}'
STUBS['org/firstinspires/ftc/robotcore/external/navigation/AngleUnit.java'] = 'package org.firstinspires.ftc.robotcore.external.navigation; public enum AngleUnit {DEGREES,RADIANS}'
TEST = '''package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.teamcode.odometry.GoBildaPinpointDriver;
public class ControlRegression {
 static int checks;
 static void check(boolean ok,String why){checks++;if(!ok)throw new AssertionError(why);}
 static void near(double a,double b,String why){check(Math.abs(a-b)<1e-9,why+": "+a+" != "+b);}
 public static void main(String[] args) {
  HardwareMap map=new HardwareMap(); DcMotorEx fly=new DcMotorEx(); DcMotor roller=new DcMotor(), wind=new DcMotor(); Servo gate=new Servo();
  map.devices.put("shooter",fly);map.devices.put("intake",roller);map.devices.put("windmill",wind);map.devices.put("gate",gate);
  Intake intake=new Intake();intake.init(map);Shooter shooter=new Shooter();shooter.init(map,intake);
  near(gate.position,0.5,"initial gate closed");
  intake.start();check(roller.power>0&&wind.power<0,"default inward direction");
  intake.switchSpeed(0.3);near(wind.power,-0.3,"custom inward direction");
  intake.switchSpeed(-0.3);near(wind.power,0.3,"custom reverse direction");
  intake.switchSpeed(Double.NaN);near(roller.power,0,"invalid intake power stopped");
  shooter.start();fly.velocity=2000;shooter.update(0);shooter.update(200_000_000L);
  check(shooter.isReady(),"stable speed ready");near(roller.power,0,"spinup alone does not feed");
  shooter.setFireRequested(true);shooter.update(210_000_000L);check(roller.power>0,"ready requested feed");
  fly.velocity=1800;shooter.update(220_000_000L);near(roller.power,0,"droop pauses feed");near(gate.position,0.5,"droop closes gate");
  fly.velocity=2000;shooter.update(230_000_000L);check(!shooter.isReady(),"recovery requires dwell");
  shooter.update(390_000_000L);check(roller.power>0,"recovery resumes requested feed");
  intake.out();shooter.update(400_000_000L);check(roller.power<0,"manual reverse beats automatic feed");near(gate.position,0.5,"reverse closes gate");
  shooter.setFireRequested(true);intake.stop();shooter.update(410_000_000L);near(roller.power,0,"manual stop wins same loop");
  shooter.setFireRequested(true);shooter.update(420_000_000L);shooter.closeGate();shooter.update(430_000_000L);near(roller.power,0,"gate close cancels feeding");
  shooter.stop();fly.velocity=2000;shooter.update(900_000_000L);check(!shooter.isReady(),"coasting disabled");near(roller.power,0,"coasting cannot feed");
  shooter.start();shooter.setFireRequested(true);shooter.update(1_000_000_000L);shooter.update(1_200_000_000L);shooter.switchSpeed();near(fly.commanded,1500,"active preset immediately changes commanded velocity");check(!shooter.isReady(),"preset change resets readiness");shooter.update(1_300_000_000L);near(roller.power,0,"overspeed pauses feed");
  map.devices.remove("gate");Shooter noGate=new Shooter();noGate.init(map,intake);noGate.openGate();noGate.closeGate();noGate.stop();check(noGate.getGateStatus().equals("Not installed"),"missing gate safe and truthful");
  DcMotor fl=new DcMotor(),fr=new DcMotor(),bl=new DcMotor(),br=new DcMotor();map.devices.put("frontLeftMotor",fl);map.devices.put("frontRightMotor",fr);map.devices.put("backLeftMotor",bl);map.devices.put("backRightMotor",br);
  MecanumDrive drive=new MecanumDrive();drive.init(map);drive.drive(-1,1,1);near(fl.power,0.5,"combined command respects half speed");near(fr.power,-1.0/6,"normalization preserves wheel ratio");near(bl.power,1.0/6,"normalized back left");near(br.power,1.0/6,"normalized back right");
  drive.setSpeedMultiplier(1);drive.drive(-0.2,0.1,0);near(fl.power,0.3,"unsaturated command unchanged");
  GoBildaPinpointDriver pinpoint=new GoBildaPinpointDriver();map.devices.put("pinpoint",pinpoint);OdometryMovement odom=new OdometryMovement();odom.init(map,drive);drive.init(map,odom);odom.update(new Gamepad());odom.getHorizantalValue();odom.getVerticalValue();odom.getHeadingDegrees();check(pinpoint.reads==1,"one odometry read per loop including getters");
  odom.startDistanceMovement("forward",12);odom.update(new Gamepad());drive.drive(0,0,0);check(fl.power!=0,"wired odometry moves drive");odom.stopMoving();near(fl.power,0,"stop clears offsets before zero drive");near(br.power,0,"all wheels stopped");
  AprilTagAlignment alignment=new AprilTagAlignment();Limelight vision=new Limelight(); com.qualcomm.hardware.limelightvision.Limelight3A camera=new com.qualcomm.hardware.limelightvision.Limelight3A();map.devices.put("limelight",camera);vision.init(map);com.qualcomm.hardware.limelightvision.LLResult result=new com.qualcomm.hardware.limelightvision.LLResult();result.tx=15;camera.result=result;vision.update();alignment.init(vision,new com.qualcomm.robotcore.util.ElapsedTime());alignment.update();alignment.autoRotate();drive.init(map,alignment);drive.drive(0,0,0);check(fl.power!=0,"vision offset active before stop");drive.stop();near(fl.power,0,"stop bypasses vision offset");near(br.power,0,"stop bypasses all injected motion");
  shooter.stop();shooter.start();fly.velocity=1500;shooter.update(2_000_000_000L);shooter.update(2_200_000_000L);shooter.setFireRequested(true);shooter.update(2_300_000_000L);shooter.setFireRequested(false);shooter.update(2_400_000_000L);near(roller.power,0,"trigger release cancels feed");
  check(vision.isTargetVisible()&&vision.getBotPose()!=null,"fresh pose visible");result.stale=201;vision.update();check(!vision.isTargetVisible()&&vision.getBotPose()==null,"stale pose cleared");result.stale=0;result.valid=false;vision.update();check(!vision.isTargetVisible()&&vision.getBotPose()==null,"invalid pose cleared");result.valid=true;result.tx=Double.NaN;vision.update();check(!vision.isTargetVisible(),"NaN target rejected");
  alignment.init(vision,new com.qualcomm.robotcore.util.ElapsedTime(),0.123,0.456);alignment.addIndexValue();near(alignment.getkP_rotation(),0.133,"custom P retained when editing");near(alignment.getkD_rotation(),0.456,"custom D retained when editing P");
  result.tx=0;result.timestamp=1;for(int id=34;id<=37;id++)result.tags.add(new com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult(id));
  com.pedropathing.follower.Follower follower=new com.pedropathing.follower.Follower();org.firstinspires.ftc.teamcode.Auton.HiveAim aim=new org.firstinspires.ftc.teamcode.Auton.HiveAim(follower,camera);com.pedropathing.ivy.Command command=aim.alignAndCalculate("test",0);com.qualcomm.robotcore.util.ElapsedTime.now=0;command.start.run();com.qualcomm.robotcore.util.ElapsedTime.now=0.3;command.execute.run();com.qualcomm.robotcore.util.ElapsedTime.now=0.6;command.execute.run();check(!command.done.getAsBoolean(),"frozen frame cannot lock");result.timestamp=2;command.execute.run();check(command.done.getAsBoolean(),"distinct fresh frame completes stable lock");
  shooter.setFireRequested(true);shooter.closeGate();shooter.update(2_500_000_000L);near(roller.power,0,"held fire plus gate close wins");
  for(com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult tag:result.tags)tag.pose.position.x=0.3;
  command=aim.alignAndCalculate("turn test",0);com.qualcomm.robotcore.util.ElapsedTime.now=0;command.start.run();com.qualcomm.robotcore.util.ElapsedTime.now=0.3;result.timestamp=3;int reads=camera.reads;command.execute.run();check(camera.reads==reads+1,"one camera snapshot per execute");check(follower.turn!=0,"offcenter target turns");double turn=follower.turn;com.qualcomm.robotcore.util.ElapsedTime.now=0.35;command.execute.run();near(follower.turn,turn,"fresh duplicate preserves turn");result.stale=201;command.execute.run();near(follower.turn,0,"stale frame stops turn");result.stale=0;result.timestamp=4;command.execute.run();check(follower.turn!=0,"fresh target reacquires");result.pipeline=0;command.execute.run();near(follower.turn,0,"wrong pipeline same timestamp stops turn");
  System.out.println("PASS: "+checks+" control behavior checks");
 }
}
'''

def main():
    java_home = os.environ.get('JAVA_HOME')
    suffix = '.exe' if os.name == 'nt' else ''
    javac = str(Path(java_home)/('bin/javac'+suffix)) if java_home else shutil.which('javac')
    java = str(Path(java_home)/('bin/java'+suffix)) if java_home else shutil.which('java')
    if not javac or not java:
        raise SystemExit('Set JAVA_HOME to a JDK, or add javac/java to PATH.')
    with tempfile.TemporaryDirectory(prefix='robot-controls-') as temp:
        root=Path(temp)
        for name, source in STUBS.items():
            path=root/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_text(source)
        dest=root/'org/firstinspires/ftc/teamcode/subsystems'
        dest.mkdir(parents=True,exist_ok=True)
        for name in ('Shooter','Intake','MecanumDrive','OdometryMovement','AprilTagAlignment','Limelight'):
            source=(MAIN/'subsystems'/f'{name}.java').read_text()
            (dest/f'{name}.java').write_text(source)
        aim_dest=root/'org/firstinspires/ftc/teamcode/Auton/HiveAim.java'
        aim_dest.parent.mkdir(parents=True,exist_ok=True)
        aim_dest.write_text((MAIN/'Auton/HiveAim.java').read_text())
        (dest/'ControlRegression.java').write_text(TEST)
        subprocess.run([javac,'-d',str(root/'classes')]+[str(p) for p in root.rglob('*.java')],check=True)
        subprocess.run([java,'-cp',str(root/'classes'),'org.firstinspires.ftc.teamcode.subsystems.ControlRegression'],check=True)
if __name__=='__main__':main()

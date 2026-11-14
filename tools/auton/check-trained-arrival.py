from pathlib import Path
import subprocess, sys, os, shutil

repo = Path(__file__).resolve().parents[2]
root = repo / 'build/trained-arrival-check'
root.mkdir(parents=True, exist_ok=True)
src = root / 'src'
classes = root / 'classes'
classes.mkdir(exist_ok=True)
def write(package, name, body):
    path = src / Path(package.replace('.', '/')) / (name + '.java')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('package ' + package + ';\n' + body)

write('com.pedropathing.math', 'Pose', '''public class Pose {
 private final double x,y,h; public Pose(double x,double y){this(x,y,0);} public Pose(double x,double y,double h){this.x=x;this.y=y;this.h=h;}
 public double x(){return x;} public double y(){return y;} public double heading(){return h;}
}''')
write('com.pedropathing.drivetrain', 'DrivePowers', 'public class DrivePowers {double a,b,c;public DrivePowers(double a,double b,double c){this.a=a;this.b=b;this.c=c;} public double forward(){return a;}public double strafe(){return b;}public double turn(){return c;}}')
write('com.pedropathing.drivetrain', 'Drivetrain', '''public interface Drivetrain {
 void drive(DrivePowers p,boolean m); double maxScaling(DrivePowers a,DrivePowers b);
 double interpolateVelocity(double a,double b,double t); double interpolateAcceleration(double a,double b,double t);
 void stop();void stop(boolean b);java.util.Map<String,Object> debug();}''')
write('com.pedropathing.paths', 'Path', '''import com.pedropathing.math.Pose;
public class Path {public Pose a,b;public Path(Pose a,Pose b){this.a=a;this.b=b;}public Path linear(Pose a,Pose b){return this;}
public Pose get(double t){return new Pose(a.x()+(b.x()-a.x())*t,a.y()+(b.y()-a.y())*t,a.heading()+(b.heading()-a.heading())*t);}}''')
write('com.pedropathing.api', 'Paths', '''import com.pedropathing.math.Pose;import com.pedropathing.paths.Path;
public class Paths {public static Path line(Pose a,Pose b){return new Path(a,b);}public static Path curve(Pose a,Pose b,Pose c,Pose d){return new Path(a,d);}}''')
write('com.pedropathing.follower', 'Follower', '''import com.pedropathing.math.Pose;import com.pedropathing.paths.Path;import com.pedropathing.drivetrain.*;
public class Follower {public Object localizer;public Drivetrain drivetrain;public Pose p;public boolean held;public int holds,follows;public DrivePowers raw;
public static class Velocity{public double vx,vy;}public Velocity v=new Velocity();
public Follower(Object l,Drivetrain d,Object a){drivetrain=d;}public Object algorithm(){return null;}public Pose pose(){return p;}
public void setPose(Pose p){this.p=p;}public Velocity velocity(){return v;}public void update(){if(drivetrain!=null){if(raw!=null)drivetrain.drive(raw,true);else if(!held)drivetrain.stop();}} public void stop(){raw=null;held=false;}
public void manual(DrivePowers p){raw=p;held=false;}
public void follow(Path p){follows++;held=false;}public void hold(Pose p){holds++;held=true;}public boolean holding(){return held;}}''')
write('com.qualcomm.robotcore.util', 'ElapsedTime', 'public class ElapsedTime {public static double now;private double origin;public ElapsedTime(){reset();}public void reset(){origin=now;} public double seconds(){return now-origin;}}')
write('com.qualcomm.hardware.gobilda', 'GoBildaPinpointDriver', 'public class GoBildaPinpointDriver {public enum DeviceStatus{READY}public DeviceStatus getDeviceStatus(){return DeviceStatus.READY;}}')
write('com.qualcomm.robotcore.hardware', 'CRServo', 'public interface CRServo {void setPower(double p);}')
write('com.qualcomm.robotcore.hardware', 'TouchSensor', 'public interface TouchSensor {boolean isPressed();}')
write('com.qualcomm.robotcore.hardware', 'HardwareMap', '''public class HardwareMap {
public java.util.Map<String,Object> devices=new java.util.HashMap<>();
public <T>T get(Class<T> c,String n){T t=tryGet(c,n);if(t==null)throw new RuntimeException("Missing "+n);return t;}
public <T>T tryGet(Class<T> c,String n){return c.cast(devices.get(n));}}''')
write('org.firstinspires.ftc.robotcore.external', 'Telemetry', 'public class Telemetry {public void addLine(String s){}public void addData(String k,Object v){}public void addData(String k,String f,Object... a){}public void update(){}}')
write('com.qualcomm.robotcore.eventloop.opmode', 'OpMode', '''public abstract class OpMode {
public com.qualcomm.robotcore.hardware.HardwareMap hardwareMap=new com.qualcomm.robotcore.hardware.HardwareMap();
public org.firstinspires.ftc.robotcore.external.Telemetry telemetry=new org.firstinspires.ftc.robotcore.external.Telemetry();
public abstract void init();public void start(){}public abstract void loop();public void stop(){} }''')
write('org.firstinspires.ftc.teamcode.subsystems', 'RobotSettings', 'public class RobotSettings {public static void ensureLoaded(){}}')
write('org.firstinspires.ftc.teamcode.pedro', 'Constants', 'public class Constants {public static com.pedropathing.follower.Follower create(Object h){return null;}}')
write('org.firstinspires.ftc.teamcode.subsystems', 'Intake', 'public class Intake {public void init(Object h){}public void stop(){}public void start(){}public void stopWindmill(){}}')
write('org.firstinspires.ftc.teamcode.subsystems', 'Shooter', '''public class Shooter {
private double target=1900,trim=25;public double spinTarget;
public void setAutoTarget(double t){target=t;}public void resetTrim(){trim=0;}public double getTargetLaunchPower(){return target+trim;}
public void init(Object h,Intake i){}public void stop(){}public void start(){}public void spinUp(){spinTarget=getTargetLaunchPower();}public void update(){}public void holdFire(){}
public void resetShotCount(){}public int getShotCount(){return 0;}public double getCurrentSpeed(){return 0;}public void setFeedPermit(boolean b){}
public String getFeedStage(){return "WAIT";}public void addTelemetry(Object t){} }''')
pkg='org.firstinspires.ftc.teamcode.Auton'
write(pkg, 'ArrivalChecks', '''import java.lang.reflect.*;import com.pedropathing.follower.Follower;import com.pedropathing.math.Pose;import com.pedropathing.drivetrain.*;
import com.qualcomm.robotcore.util.ElapsedTime;import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.*;import org.firstinspires.ftc.teamcode.subsystems.FlowerArm;
public class ArrivalChecks {
static class Auto extends TrainedFlowerAuto {boolean blue;Auto(boolean b){blue=b;}protected boolean blueAlliance(){return blue;}}
static class Servo implements CRServo {double power;public void setPower(double p){power=p;}}
static class Touch implements TouchSensor {boolean pressed;Touch(boolean p){pressed=p;}public boolean isPressed(){return pressed;}}
static class Drive implements Drivetrain {
 DrivePowers last=new DrivePowers(0,0,0);public void drive(DrivePowers p,boolean m){last=p;}
 public double maxScaling(DrivePowers a,DrivePowers b){return 1;}public double interpolateVelocity(double a,double b,double t){return 0;}
 public double interpolateAcceleration(double a,double b,double t){return 0;}public void stop(){last=new DrivePowers(0,0,0);}public void stop(boolean b){stop();}
 public java.util.Map<String,Object> debug(){return java.util.Collections.emptyMap();}
}
static Field field(String n)throws Exception{Field f=TrainedFlowerAuto.class.getDeclaredField(n);f.setAccessible(true);return f;}
static void set(Auto a,String n,Object v)throws Exception{field(n).set(a,v);}static Object get(Auto a,String n)throws Exception{return field(n).get(a);}
static Object call(Auto a,String n,Class<?>[] t,Object... v)throws Exception{Method m=TrainedFlowerAuto.class.getDeclaredMethod(n,t);m.setAccessible(true);return m.invoke(a,v);}
static int checks;static void check(boolean b,String s){checks++;if(!b)throw new AssertionError(s);}
static Pose shift(Pose p,boolean blue,double error,double degrees){return new Pose(p.x()-error*Math.cos(p.heading()),p.y()-error*Math.sin(p.heading()),p.heading()+Math.toRadians(degrees));}
static Auto setup(boolean blue,int leg,double error,double degrees,double speed)throws Exception{
 ElapsedTime.now=0;
 Auto a=new Auto(blue);Follower f=new Follower(null,null,null);set(a,"follower",f);set(a,"leg",leg);
 a.hardwareMap.devices.put("flowerArm",new Servo());a.hardwareMap.devices.put("touchUp",new Touch(false));a.hardwareMap.devices.put("touchDown",new Touch(true));
 ((FlowerArm)get(a,"flowerArm")).init(a.hardwareMap,true);
 Pose target=(Pose)call(a,"nominalEnd",new Class[]{int.class},leg);set(a,"target",target);f.p=shift(target,blue,error,degrees);f.v.vx=speed;
 Field state=field("state");state.set(a,Enum.valueOf((Class)state.getType(),"DRIVE"));
 set(a,"pinpoint",new GoBildaPinpointDriver());Class<?> drive=Class.forName(TrainedFlowerAuto.class.getName()+"$LimitedDrive");
 Constructor<?> ctor=drive.getDeclaredConstructors()[0];ctor.setAccessible(true);Object wrapper=ctor.newInstance(new Drive());set(a,"limitedDrive",wrapper);f.drivetrain=(Drivetrain)wrapper;
 ElapsedTime.now=1;return a;
}
public static void main(String[] args)throws Exception{
for(boolean blue:new boolean[]{false,true}){
 Auto geometry=setup(blue,0,0,0,0);
 double[][] curves=(double[][])get(geometry,"CURVES");
 for(int i:new int[]{2,3,4,8,9,10}){
  Pose original=(Pose)call(geometry,"transform",new Class[]{double.class,double.class,double.class},curves[i][6],curves[i][7],curves[i][9]);
  Pose shifted=(Pose)call(geometry,"nominalEnd",new Class[]{int.class},i);
  double h=(i<8?Math.PI:Math.PI/2)+(blue?Math.PI:0),dx=shifted.x()-original.x(),dy=shifted.y()-original.y();
  check(Math.abs(-Math.sin(h)*dx+Math.cos(h)*dy-3)<1e-6,"lineup must move three inches toward robot-left on each alliance");
  check(Math.abs(Math.cos(h)*dx+Math.sin(h)*dy)<1e-6,"lineup offset must not move closer to wall");
 }
 for(int i=1;i<curves.length;i++){
  Pose previous=(Pose)call(geometry,"nominalEnd",new Class[]{int.class},i-1);
  Pose next=(Pose)call(geometry,"curvePoint",new Class[]{int.class,int.class,double.class},i,0,curves[i][8]);
  check(Math.hypot(previous.x()-next.x(),previous.y()-next.y())<1e-5,"shifted route endpoints must stay connected");
 }
 for(int i:new int[]{0,6,12}){
  Pose p=(Pose)call(geometry,"nominalEnd",new Class[]{int.class},i);
  Pose original=(Pose)call(geometry,"transform",new Class[]{double.class,double.class,double.class},curves[i][6],curves[i][7],curves[i][9]);
  check(Math.hypot(p.x()-original.x(),p.y()-original.y())<1e-6,"flower offset must not shift shooting marks");
 }
 // Exercise the real CR-servo arm and limit-switch code during startup.
 Auto startup=setup(blue,0,0,0,0);Follower sf=(Follower)get(startup,"follower");
 sf.p=(Pose)call(startup,"transform",new Class[]{double.class,double.class,double.class},-13.606197,-61.569252,Math.PI/2);
 FlowerArm arm=(FlowerArm)get(startup,"flowerArm");Servo servo=(Servo)startup.hardwareMap.devices.get("flowerArm");
 Touch up=(Touch)startup.hardwareMap.devices.get("touchUp"),down=(Touch)startup.hardwareMap.devices.get("touchDown");
 up.pressed=true;down.pressed=false;arm.init(startup.hardwareMap,true);
 check(servo.power==0,"initialization must not lower the ramp before START");
 check(!FlowerArm.ENABLED&&arm.isAvailable(),"required instance must work without changing saved/global enable");
 FlowerArm.DIRECTION=-1; // Model a previously saved value loaded during INIT.
 startup.start();check(get(startup,"state").toString().equals("LOWER_RAMP")&&arm.isDown(),"START must command ramp down first");
 check(FlowerArm.DIRECTION==1,"auton must use requested direction even after old settings load");
 check(servo.power<0&&sf.follows==0,"direction 1 must drive down with negative servo power before travel");
 check(((org.firstinspires.ftc.teamcode.subsystems.Shooter)get(startup,"shooter")).spinTarget==1500,"spin-up must use exactly 1500 with old trim removed");
 up.pressed=false;ElapsedTime.now=1.5;startup.loop();check(sf.follows==0,"robot must wait for touchDown");
 down.pressed=true;ElapsedTime.now=1.6;startup.loop();
 check(servo.power==0&&arm.isAtDown(),"touchDown must stop the actual arm servo");
 check(sf.follows==1&&get(startup,"state").toString().equals("DRIVE"),"confirmed down must start the route");
 startup=setup(blue,0,0,0,0);arm=(FlowerArm)get(startup,"flowerArm");down=(Touch)startup.hardwareMap.devices.get("touchDown");
 down.pressed=false;startup.start();ElapsedTime.now=1.4;startup.loop();startup.stop();
 check(((Servo)startup.hardwareMap.devices.get("flowerArm")).power==0,"STOP must turn off a moving ramp servo");
 startup=setup(blue,0,0,0,0);startup.hardwareMap.devices.remove("touchDown");arm=(FlowerArm)get(startup,"flowerArm");arm.init(startup.hardwareMap,true);startup.start();
 check(get(startup,"state").toString().equals("DONE")&&((Follower)get(startup,"follower")).follows==0,"missing switch must block startup");
 startup=setup(blue,0,0,0,0);down=(Touch)startup.hardwareMap.devices.get("touchDown");down.pressed=false;startup.start();ElapsedTime.now=6.1;startup.loop();
 check(get(startup,"state").toString().equals("DONE")&&((Servo)startup.hardwareMap.devices.get("flowerArm")).power==0,"arm timeout must stop servo and auton");
 startup=setup(blue,0,0,0,0);((Touch)startup.hardwareMap.devices.get("touchUp")).pressed=true;startup.start();startup.loop();
 check(get(startup,"state").toString().equals("DONE"),"contradictory switches must stop startup");
 // Normal travel accepts a three-inch radius and rejoins from measured pose.
 Auto normal=setup(blue,1,2.9,0,0);
 check((Boolean)call(normal,"driveArrived",new Class[]{}),"normal travel must accept 2.9 inches");
 normal.loop();check((Integer)get(normal,"leg")==2,"normal arrival must advance to next path");
 check(get(normal,"parkingReason").equals("None"),"old 1.1-inch join gate must not reject accepted travel error");
 check(((Follower)get(normal,"follower")).follows==1&&get(normal,"state").toString().equals("DRIVE"),"next path must start from measured position");
 normal=setup(blue,1,3.01,0,0);
 check(!(Boolean)call(normal,"driveArrived",new Class[]{}),"normal travel must reject errors above three inches");
 normal=setup(blue,2,2.9,0,0);
 check(!(Boolean)call(normal,"driveArrived",new Class[]{}),"flower alignment must retain tighter tolerance");
 normal=setup(blue,1,2.9,3.1,0);
 check(!(Boolean)call(normal,"driveArrived",new Class[]{}),"normal heading limit must remain active");
 normal=setup(blue,1,2.9,0,4.6);
 check(!(Boolean)call(normal,"driveArrived",new Class[]{}),"normal speed limit must remain active");
 normal=setup(blue,0,2.9,0,0);normal.loop();
 check(get(normal,"state").toString().equals("SHOOT"),"travel can settle into shooting hold within three inches");
 ElapsedTime.now=1.4;normal.loop();
 check(get(normal,"result").equals("Reacquiring shooting aim"),"shooting must still require precise aim");
 for(int leg:new int[]{2,8}){
  Auto a=setup(blue,leg,.5,1.5,1);
  a.loop();check((Integer)get(a,"leg")==leg+1,"must advance into approach");check(get(a,"state").toString().equals("PUSH"),"staging must start timed push");
  check(((Follower)get(a,"follower")).follows==0,"pickup must not follow a Pedro path");
 }
 for(int leg:new int[]{3,9}){
  Auto a=setup(blue,leg,0,0,0);Follower f=(Follower)get(a,"follower");Pose t=(Pose)get(a,"target");
  f.p=(Pose)call(a,"nominalEnd",new Class[]{int.class},leg-1);call(a,"driveLeg",new Class[]{});
  check(get(a,"state").toString().equals("PUSH"),"must enter push from measured staging pose");
  ElapsedTime.now=1.1;a.loop();check(f.raw.forward()==.30&&f.raw.strafe()==0&&f.raw.turn()==0,"both alliances must command robot-forward power");
  check(f.follows==0,"manual push must not use an endpoint path");
  ElapsedTime.now=2.3;a.loop();check(get(a,"state").toString().equals("PUSH")&&f.raw!=null,"push must continue beyond the old 1.25-second duration");
  // The timed push ends even if localization is still three inches short.
  f.p=shift(t,blue,3,0);ElapsedTime.now=1+TrainedFlowerAuto.FLOWER_PUSH_SECONDS;a.loop();
  check(get(a,"state").toString().equals("COLLECT"),"elapsed time must end push without precision arrival");
  check(f.raw==null,"collection must cancel manual drive");
  Field base=f.drivetrain.getClass().getDeclaredField("base");base.setAccessible(true);Drive d=(Drive)base.get(f.drivetrain);
  check(d.last.forward()==0,"drive must stop immediately when push ends");
  ElapsedTime.now=1+TrainedFlowerAuto.FLOWER_PUSH_SECONDS+TrainedFlowerAuto.FLOWER_COLLECT_SECONDS+.01;a.loop();check((Integer)get(a,"leg")==leg+1,"collection must advance to retreat");
  check(f.follows==1&&get(a,"state").toString().equals("DRIVE"),"retreat must start from measured pose despite missing old endpoint");
  f.p=shift((Pose)get(a,"target"),blue,.5,1.5);a.loop();check((Integer)get(a,"leg")==leg+2,"retreat must accept staging tolerance");
  // Intentional contact must not trip the old quarter-inch wall/pot checks.
  a=setup(blue,leg,0,0,0);f=(Follower)get(a,"follower");f.p=(Pose)call(a,"nominalEnd",new Class[]{int.class},leg-1);call(a,"driveLeg",new Class[]{});
  f.p=new Pose(t.x()+.1*Math.cos(t.heading()),t.y()+.1*Math.sin(t.heading()),t.heading()+Math.toRadians(.99));
  ElapsedTime.now=1.2;a.loop();check(get(a,"state").toString().equals("PUSH"),"selected-flower contact must remain allowed");
  f.p=new Pose(t.x()+.6*Math.cos(t.heading()),t.y()+.6*Math.sin(t.heading()),t.heading());a.loop();
  check(get(a,"state").toString().equals("COLLECT")&&f.raw==null,"overtravel guard must stop power and collect");
  a=setup(blue,leg,0,0,0);f=(Follower)get(a,"follower");f.p=(Pose)call(a,"nominalEnd",new Class[]{int.class},leg-1);call(a,"driveLeg",new Class[]{});
  f.p=new Pose(t.x(),t.y(),t.heading()+Math.toRadians(4));ElapsedTime.now=1.1;a.loop();
  check(get(a,"state").toString().equals("DONE")&&f.raw==null,"bad alignment outside contact corridor must stop");
  a=setup(blue,leg,0,0,0);f=(Follower)get(a,"follower");f.p=(Pose)call(a,"nominalEnd",new Class[]{int.class},leg-1);call(a,"driveLeg",new Class[]{});
  ElapsedTime.now=29.7;a.loop();check(get(a,"state").toString().equals("DONE")&&f.raw==null,"deadline must cancel push");
  a=setup(blue,leg,0,0,0);f=(Follower)get(a,"follower");f.p=(Pose)call(a,"nominalEnd",new Class[]{int.class},leg-1);call(a,"driveLeg",new Class[]{});
  a.stop();check(f.raw==null,"external stop must cancel manual power");
  a=setup(blue,leg,0,0,0);f=(Follower)get(a,"follower");f.p=(Pose)call(a,"nominalEnd",new Class[]{int.class},leg-1);call(a,"driveLeg",new Class[]{});
  ElapsedTime.now=28;a.loop();check(get(a,"parkingReason").equals("Time reserved for parking"),"parking reserve must interrupt push");
  base=f.drivetrain.getClass().getDeclaredField("base");base.setAccessible(true);d=(Drive)base.get(f.drivetrain);
  check(d.last.forward()==0,"parking must stop manual motor power immediately");
  TrainedFlowerAuto.FLOWER_PUSH_SECONDS=2.1;
  a=setup(blue,leg,0,0,0);f=(Follower)get(a,"follower");f.p=(Pose)call(a,"nominalEnd",new Class[]{int.class},leg-1);call(a,"driveLeg",new Class[]{});
  check(get(a,"state").toString().equals("DONE")&&f.raw==null,"invalid duration must not start push");
  TrainedFlowerAuto.FLOWER_PUSH_SECONDS=2.0;
 }
 Auto a=setup(blue,2,1,0,0);ElapsedTime.now=6.1;a.loop();check(get(a,"parkingReason").equals("Drive timeout"),"travel timeout must remain enabled");
 check(get(a,"abortDetails").toString().contains("position error (in)="),"abort must retain original target error");
}
System.out.println("PASS: "+checks+" red/blue staging, timed push, motor stop, collection, measured retreat, contact corridor and deadline checks. Scripted hardware; no physical pickup claim.");
}}
''')
java=Path(os.environ['JAVA_HOME'])/'bin' if os.environ.get('JAVA_HOME') else Path(shutil.which('javac') or sys.exit('Set JAVA_HOME to a JDK or add javac to PATH')).parent
exe='.exe' if os.name=='nt' else ''
actual=repo/'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auton'
files=list(src.rglob('*.java'))+[Path(sys.argv[1]) if len(sys.argv)>1 else actual/'TrainedFlowerAuto.java',actual/'cycles/FrameClearance.java',actual.parent/'subsystems/FlowerArm.java',actual.parent/'subsystems/InSimulator.java']
subprocess.run([str(java/('javac'+exe)),'-d',str(classes),*map(str,files)],check=True)
subprocess.run([str(java/('java'+exe)),'-cp',str(classes),pkg+'.ArrivalChecks'],check=True)

# Optional nominal-route geometry check against the installed Pedro implementation.
pedro_jar=os.environ.get('PEDRO_CORE_JAR')
if pedro_jar:
    real_classes=root/'real-pedro-classes'
    real_classes.mkdir(exist_ok=True)
    real_files=[p for p in files if not str(p.relative_to(src) if p.is_relative_to(src) else p).replace('\\','/').startswith('com/pedropathing/') and p.name!='ArrivalChecks.java']
    real_files.append(repo/'tools/auton/ShiftedRouteChecks.java')
    subprocess.run([str(java/('javac'+exe)),'-cp',pedro_jar,'-d',str(real_classes),*map(str,real_files)],check=True)
    subprocess.run([str(java/('java'+exe)),'-cp',str(real_classes)+os.pathsep+pedro_jar,pkg+'.ShiftedRouteChecks'],check=True)

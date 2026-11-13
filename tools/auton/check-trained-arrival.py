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
write('com.pedropathing.drivetrain', 'DrivePowers', 'public class DrivePowers {public DrivePowers(double a,double b,double c){} public double forward(){return 0;}public double strafe(){return 0;}public double turn(){return 0;}}')
write('com.pedropathing.drivetrain', 'Drivetrain', '''public interface Drivetrain {
 void drive(DrivePowers p,boolean m); double maxScaling(DrivePowers a,DrivePowers b);
 double interpolateVelocity(double a,double b,double t); double interpolateAcceleration(double a,double b,double t);
 void stop();void stop(boolean b);java.util.Map<String,Object> debug();}''')
write('com.pedropathing.paths', 'Path', '''import com.pedropathing.math.Pose;
public class Path {public Pose a,b;public Path(Pose a,Pose b){this.a=a;this.b=b;}public Path linear(Pose a,Pose b){return this;}
public Pose get(double t){return new Pose(a.x()+(b.x()-a.x())*t,a.y()+(b.y()-a.y())*t,a.heading()+(b.heading()-a.heading())*t);}}''')
write('com.pedropathing.api', 'Paths', '''import com.pedropathing.math.Pose;import com.pedropathing.paths.Path;
public class Paths {public static Path line(Pose a,Pose b){return new Path(a,b);}public static Path curve(Pose a,Pose b,Pose c,Pose d){return new Path(a,d);}}''')
write('com.pedropathing.follower', 'Follower', '''import com.pedropathing.math.Pose;import com.pedropathing.paths.Path;import com.pedropathing.drivetrain.Drivetrain;
public class Follower {public Object localizer;public Drivetrain drivetrain;public Pose p;public boolean held;public int holds,follows;
public static class Velocity{public double vx,vy;}public Velocity v=new Velocity();
public Follower(Object l,Drivetrain d,Object a){drivetrain=d;}public Object algorithm(){return null;}public Pose pose(){return p;}
public void setPose(Pose p){this.p=p;}public Velocity velocity(){return v;}public void update(){} public void stop(){}
public void follow(Path p){follows++;held=false;}public void hold(Pose p){holds++;held=true;}public boolean holding(){return held;}}''')
write('com.qualcomm.robotcore.util', 'ElapsedTime', 'public class ElapsedTime {public static double now;public void reset(){now=0;} public double seconds(){return now;}}')
write('com.qualcomm.hardware.gobilda', 'GoBildaPinpointDriver', 'public class GoBildaPinpointDriver {public enum DeviceStatus{READY}public DeviceStatus getDeviceStatus(){return DeviceStatus.READY;}}')
write('com.qualcomm.robotcore.eventloop.opmode', 'OpMode', '''public abstract class OpMode {
public Object hardwareMap;public Telemetry telemetry=new Telemetry();public static class Telemetry{public void addLine(String s){}public void addData(String k,Object v){}public void addData(String k,String f,Object... a){}public void update(){}}
public abstract void init();public void start(){}public abstract void loop();public void stop(){} }'''.replace('public Object hardwareMap;', 'public HardwareMap hardwareMap;public static class HardwareMap{public <T>T get(Class<T> c,String n){return null;}}'))
write('org.firstinspires.ftc.teamcode.pedro', 'Constants', 'public class Constants {public static com.pedropathing.follower.Follower create(Object h){return null;}}')
write('org.firstinspires.ftc.teamcode.subsystems', 'Intake', 'public class Intake {public void init(Object h){}public void stop(){}public void start(){}public void stopWindmill(){}}')
write('org.firstinspires.ftc.teamcode.subsystems', 'Shooter', '''public class Shooter {
public void init(Object h,Intake i){}public void stop(){}public void start(){}public void spinUp(){}public void update(){}public void holdFire(){}
public void resetShotCount(){}public int getShotCount(){return 0;}public double getCurrentSpeed(){return 0;}public void setFeedPermit(boolean b){}
public String getFeedStage(){return "WAIT";}public void addTelemetry(Object t){} }''')
pkg='org.firstinspires.ftc.teamcode.Auton'
write(pkg, 'ArrivalChecks', '''import java.lang.reflect.*;import com.pedropathing.follower.Follower;import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
public class ArrivalChecks {
static class Auto extends TrainedFlowerAuto {boolean blue;Auto(boolean b){blue=b;}protected boolean blueAlliance(){return blue;}}
static Field field(String n)throws Exception{Field f=TrainedFlowerAuto.class.getDeclaredField(n);f.setAccessible(true);return f;}
static void set(Auto a,String n,Object v)throws Exception{field(n).set(a,v);}static Object get(Auto a,String n)throws Exception{return field(n).get(a);}
static Object call(Auto a,String n,Class<?>[] t,Object... v)throws Exception{Method m=TrainedFlowerAuto.class.getDeclaredMethod(n,t);m.setAccessible(true);return m.invoke(a,v);}
static int checks;static void check(boolean b,String s){checks++;if(!b)throw new AssertionError(s);}
static Pose shift(Pose p,boolean blue,double error,double degrees){return new Pose(p.x()-error*Math.cos(p.heading()),p.y()-error*Math.sin(p.heading()),p.heading()+Math.toRadians(degrees));}
static Auto setup(boolean blue,int leg,double error,double degrees,double speed)throws Exception{
 Auto a=new Auto(blue);Follower f=new Follower(null,null,null);set(a,"follower",f);set(a,"leg",leg);
 Pose target=(Pose)call(a,"nominalEnd",new Class[]{int.class},leg);set(a,"target",target);f.p=shift(target,blue,error,degrees);f.v.vx=speed;
 Field state=field("state");state.set(a,Enum.valueOf((Class)state.getType(),"DRIVE"));
 set(a,"pinpoint",new GoBildaPinpointDriver());Class<?> drive=Class.forName(TrainedFlowerAuto.class.getName()+"$LimitedDrive");
 Constructor<?> ctor=drive.getDeclaredConstructors()[0];ctor.setAccessible(true);set(a,"limitedDrive",ctor.newInstance(new Object[]{null}));
 ElapsedTime.now=1;return a;
}
public static void main(String[] args)throws Exception{
for(boolean blue:new boolean[]{false,true}){
 for(int leg:new int[]{2,8}){
  Auto a=setup(blue,leg,.5,1.5,1);
  a.loop();check((Integer)get(a,"leg")==leg+1,"must advance into approach");check(get(a,"state").toString().equals("DRIVE"),"approach must remain active");
  check(((Follower)get(a,"follower")).follows==1,"next approach must follow a path");
 }
 for(int leg:new int[]{3,9}){
  Auto a=setup(blue,leg,1,0,0);a.loop();Follower f=(Follower)get(a,"follower");
  check(f.holds==0,"must not switch to hold one inch short");check(get(a,"state").toString().equals("DRIVE"),"must continue approaching");
  Pose t=(Pose)get(a,"target");f.p=shift(t,blue,.3,0);a.loop();check(get(a,"state").toString().equals("DRIVE"),"contact tolerance must stay precise");
  f.p=shift(t,blue,.2,0);a.loop();check(get(a,"state").toString().equals("COLLECT"),"contact arrival must start collection");
  a=setup(blue,leg,1,0,0);ElapsedTime.now=6.1;a.loop();check(get(a,"parkingReason").equals("Drive timeout"),"timeout must remain enabled");
  check(get(a,"abortDetails").toString().contains("position error (in)="),"abort must retain original target error");
  a=setup(blue,leg,.2,0,0);ElapsedTime.now=6.1;a.loop();check(get(a,"state").toString().equals("COLLECT"),"arrival on timeout loop must win");
 }
}
System.out.println("PASS: "+checks+" red/blue staging, approach, contact, timeout, and diagnostic checks. Scripted hardware; no physical pickup claim.");
}}
''')
java=Path(os.environ['JAVA_HOME'])/'bin' if os.environ.get('JAVA_HOME') else Path(shutil.which('javac') or sys.exit('Set JAVA_HOME to a JDK or add javac to PATH')).parent
exe='.exe' if os.name=='nt' else ''
actual=repo/'TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Auton'
files=list(src.rglob('*.java'))+[Path(sys.argv[1]) if len(sys.argv)>1 else actual/'TrainedFlowerAuto.java',actual/'cycles/FrameClearance.java']
subprocess.run([str(java/('javac'+exe)),'-d',str(classes),*map(str,files)],check=True)
subprocess.run([str(java/('java'+exe)),'-cp',str(classes),pkg+'.ArrivalChecks'],check=True)

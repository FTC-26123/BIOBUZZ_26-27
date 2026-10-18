"""Exercise autonomous decisions and deadlines without robot hardware (Python 3 / JDK 17+)."""
from pathlib import Path
import os
import shutil
import subprocess
import sys
import tempfile

sys.dont_write_bytecode = True
import test_controls as support

STUBS = dict(support.STUBS)
STUBS.update({
    'com/pedropathing/ivy/behaviors/EndCondition.java': '''package com.pedropathing.ivy.behaviors; public enum EndCondition { NATURALLY, INTERRUPTED, SUSPENDED }''',
    'com/pedropathing/ivy/Command.java': '''package com.pedropathing.ivy; import java.util.function.*; import com.pedropathing.ivy.behaviors.EndCondition; public class Command {
      public Runnable start=()->{},execute=()->{}; public BooleanSupplier done=()->false; public Consumer<EndCondition> end=c->{};
      public static Command build(){return new Command();} public Command setStart(Runnable r){start=r;return this;}
      public Command setExecute(Runnable r){execute=r;return this;} public Command setDone(BooleanSupplier r){done=r;return this;}
      public Command setEnd(Consumer<EndCondition> r){end=r;return this;} public void start(){start.run();}
      public void execute(){execute.run();} public boolean done(){return done.getAsBoolean();} public void end(EndCondition c){end.accept(c);} }''',
    'com/pedropathing/ivy/Scheduler.java': '''package com.pedropathing.ivy; public class Scheduler {public static void reset(){} public static void execute(){} public static void schedule(Command c){} }''',
    'com/pedropathing/ivy/commands/Commands.java': '''package com.pedropathing.ivy.commands; import com.pedropathing.ivy.*; import java.util.function.*; public class Commands {public static Command lazy(Supplier<Command> s){ Command[] c={null};return Command.build().setStart(()->{c[0]=s.get();c[0].start();}).setExecute(()->c[0].execute()).setDone(()->c[0].done()).setEnd(e->c[0].end(e));}}''',
    'com/pedropathing/math/Pose.java': '''package com.pedropathing.math; public class Pose {private final double x,y,h; public Pose(double x,double y,double h){this.x=x;this.y=y;this.h=h;}public double x(){return x;}public double y(){return y;}public double heading(){return h;} }''',
    'com/pedropathing/api/PoseFactory.java': '''package com.pedropathing.api; import com.pedropathing.math.Pose; public class PoseFactory {public static PoseFactory degrees(){return new PoseFactory();} public Pose of(double x,double y,double h){return new Pose(x,y,Math.toRadians(h));}}''',
    'com/pedropathing/paths/Path.java': '''package com.pedropathing.paths; import com.pedropathing.math.Pose; public class Path {public Path linear(Pose a,Pose b){return this;}public Path tangent(){return this;}}''',
    'com/pedropathing/api/Paths.java': '''package com.pedropathing.api; import com.pedropathing.paths.Path; import com.pedropathing.math.Pose; public class Paths {public static Path line(Pose a,Pose b){return new Path();}public static Path curve(Pose a,Pose b,Pose c){return new Path();}}''',
    'com/pedropathing/follower/Follower.java': '''package com.pedropathing.follower; import com.pedropathing.math.Pose;import com.pedropathing.paths.Path;public class Follower {
      public double forward,strafe,turn;public boolean busy,parametric;public Pose position=new Pose(0,0,0);public int stops;public final Drivetrain drivetrain=new Drivetrain();
      public class Drivetrain {public void stop(){stops++;forward=strafe=turn=0;}}public void manual(double x,double y,double t){forward=x;strafe=y;turn=t;}
      public void stop(){forward=strafe=turn=0;busy=false;}public void follow(Path p){busy=true;}public boolean isBusy(){return busy;}public boolean atParametricEnd(){return parametric;}
      public Pose pose(){return position;}public void setPose(Pose p){position=p;}public void update(){}public Path currentPath(){return null;}public int pathIndex(){return 0;}public double distanceToEndpoint(){return 0;}}
    ''',
    'com/qualcomm/hardware/gobilda/GoBildaPinpointDriver.java': '''package com.qualcomm.hardware.gobilda;public class GoBildaPinpointDriver {public enum DeviceStatus {READY,FAULT} public DeviceStatus getDeviceStatus(){return DeviceStatus.READY;}}''',
    'org/firstinspires/ftc/teamcode/pedro/Constants.java': '''package org.firstinspires.ftc.teamcode.pedro;import com.pedropathing.follower.Follower;import com.qualcomm.robotcore.hardware.HardwareMap;public class Constants {public static Follower create(HardwareMap h){return new Follower();}}''',
    'com/qualcomm/robotcore/eventloop/opmode/Autonomous.java': '''package com.qualcomm.robotcore.eventloop.opmode; public @interface Autonomous {String name();String group();}''',
    'com/qualcomm/robotcore/eventloop/opmode/LinearOpMode.java': '''package com.qualcomm.robotcore.eventloop.opmode;import com.qualcomm.robotcore.hardware.HardwareMap;import org.firstinspires.ftc.robotcore.external.Telemetry;public abstract class LinearOpMode {public HardwareMap hardwareMap=new HardwareMap();public Telemetry telemetry=new Telemetry();public abstract void runOpMode();public void waitForStart(){}public boolean isStopRequested(){return false;}public boolean opModeIsActive(){return false;}public void idle(){}}''',
    'org/firstinspires/ftc/robotcore/external/Telemetry.java': '''package org.firstinspires.ftc.robotcore.external;public class Telemetry {public void addData(String s,Object v){}public void addLine(String s){}public void update(){}}''',
})
STUBS['com/qualcomm/hardware/limelightvision/LLResult.java'] = STUBS['com/qualcomm/hardware/limelightvision/LLResult.java'].replace(
    'public boolean isValid()', 'public int parses;public List<LLResultTypes.ColorResult> colors=new ArrayList<>();public List<LLResultTypes.ColorResult> getColorResults(){parses++;return colors;}public boolean isValid()')
STUBS['com/qualcomm/hardware/limelightvision/LLResultTypes.java'] = STUBS['com/qualcomm/hardware/limelightvision/LLResultTypes.java'].replace(
    'public static class FiducialResult', '''public static class ColorResult {public double tx,area;public java.util.List<java.util.List<Double>> corners;public ColorResult(double x,double a,double top,double bottom){tx=x;area=a;corners=java.util.Arrays.asList(java.util.Arrays.asList(0.0,top),java.util.Arrays.asList(1.0,bottom));}public double getTargetXDegrees(){return tx;}public double getTargetArea(){return area;}public java.util.List<java.util.List<Double>> getTargetCorners(){return corners;}}public static class FiducialResult''')

TEST = '''package org.firstinspires.ftc.teamcode.Auton;
import com.pedropathing.follower.Follower;import com.pedropathing.ivy.Command;import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;import com.qualcomm.hardware.limelightvision.*;import com.qualcomm.robotcore.util.ElapsedTime;
public class AutonomousRegression {
 static int checks;static void check(boolean ok,String why){checks++;if(!ok)throw new AssertionError(why);}static void near(double x,double y,String why){check(Math.abs(x-y)<1e-9,why);}
 static void now(double t){ElapsedTime.now=t;}static Command waiting(){return Command.build().setDone(()->false);}
 public static void main(String[] args){
  Follower f=new Follower();Limelight3A camera=new Limelight3A();LLResult r=new LLResult();r.pipeline=0;r.timestamp=1;camera.result=r;
  r.colors.add(new LLResultTypes.ColorResult(20,1,347.3333333333,380));
  YellowBallSeeker seeker=new YellowBallSeeker(f,camera);Command seek=seeker.command();now(0);seek.start();now(0.1);seeker.sample();seek.execute();
  check(!seek.done(),"off-axis close ball is not acquired");near(f.forward,0,"close off-axis ball does not drive forward");check(f.turn>0,"close off-axis ball continues turning");
  int parses=r.parses;seeker.sample();seek.execute();seeker.addTelemetry(new org.firstinspires.ftc.robotcore.external.Telemetry());seek.done();
  check(r.parses==parses,"duplicate frame is parsed once; done/telemetry do not resample");check(camera.reads==2,"one camera fetch per explicit sample");
  r.colors.get(0).tx=0;r.timestamp=2;now(0.2);seeker.sample();seek.execute();check(!seek.done(),"arrival requires dwell");
  now(0.4);seeker.sample();seek.execute();check(!seek.done(),"frozen frame cannot confirm arrival");
  r.timestamp=3;seeker.sample();seek.execute();check(seek.done()&&seeker.acquired(),"centered close distinct frames confirm arrival");seek.end(EndCondition.NATURALLY);near(f.turn,0,"seeker end stops turn");
  seek=seeker.command();now(1);seek.start();r.timestamp=4;r.colors.get(0).tx=Double.NaN;seeker.sample();seek.execute();
  check(!seek.done()&&!seeker.acquired(),"NaN angle cannot acquire");near(f.forward,0,"NaN angle cannot command fast forward");check(Double.isFinite(f.turn),"NaN angle cannot command NaN turn");
  r.colors.get(0).tx=0;r.colors.get(0).area=Double.POSITIVE_INFINITY;r.timestamp=5;seeker.sample();seek.execute();near(f.forward,0,"nonfinite area rejected");
  r.colors.get(0).area=1;r.timestamp=6;r.stale=201;seeker.sample();seek.execute();near(f.turn,0,"stale camera stops search/steering");near(f.forward,0,"stale camera stops translation");
  r.stale=0;r.pipeline=1;seeker.sample();seek.execute();near(f.turn,0,"wrong pipeline cannot steer search");
  r.pipeline=0;r.timestamp=7;r.colors.clear();seeker.sample();seek.execute();check(f.turn>0&&f.forward==0,"fresh no-target frame permits bounded search");
  now(7.1);check(seek.done()&&!seeker.acquired(),"search timeout distinguished from arrival");seek.end(EndCondition.NATURALLY);near(f.turn,0,"timed-out search stops immediately");
  // Production wrapper waits for settled completion, not merely parametric end.
  Command follow=AutoPlan.follow(f,com.pedropathing.paths.Path::new);follow.start();f.parametric=true;check(!follow.done(),"parametric end alone does not finish path");f.busy=false;check(follow.done(),"completed follower finishes path");
  // Frozen progress must not launch an untrusted parking attempt or later stage.
  now(0);f.position=new Pose(0,0,0);int[] later={0},parks={0},ends={0};
  AutoPlan stalled=new AutoPlan(f,()->true).add("blocked",()->Command.build().setEnd(e->ends[0]++),8,true,()->true)
   .add("must skip",()->Command.build().setStart(()->later[0]++),8,true,()->true).park(()->Command.build().setStart(()->parks[0]++));
  Command run=stalled.command();run.start();f.manual(.5,0,0);now(2.1);run.execute();check(run.done(),"frozen path ends on progress deadline");near(f.forward,0,"watchdog zeros motors");check(parks[0]==0&&later[0]==0,"stalled localization does not blindly park or run later stages");check(ends[0]==1,"failed child ended exactly once");run.end(EndCondition.NATURALLY);check(ends[0]==1,"outer completion does not end child twice");
  // A timed-out non-motion stage goes straight to a bounded fallback park.
  now(0);parks[0]=0;later[0]=0;boolean[] parkDone={false};
  AutoPlan timed=new AutoPlan(f,()->true).add("aim",AutonomousRegression::waiting,1,false,()->true)
   .add("skip",()->Command.build().setStart(()->later[0]++),1,false,()->true)
   .park(()->Command.build().setStart(()->parks[0]++).setDone(()->parkDone[0]));
  run=timed.command();run.start();now(1.1);run.execute();check(parks[0]==1&&later[0]==0&&!run.done(),"stage deadline enters fallback once");parkDone[0]=true;run.execute();check(run.done(),"successful fallback completes");
  // Faults and invalid poses stop without attempting fallback.
  now(0);boolean[] ready={true};parks[0]=0;AutoPlan faulty=new AutoPlan(f,()->ready[0]).add("path",AutonomousRegression::waiting,8,true,()->true).park(()->Command.build().setStart(()->parks[0]++));
  run=faulty.command();run.start();ready[0]=false;run.execute();check(run.done()&&parks[0]==0,"device fault inhibits parking");
  ready[0]=true;f.position=new Pose(Double.NaN,0,0);run=faulty.command();run.start();check(run.done()&&parks[0]==0,"invalid initial pose cannot launch motion");f.position=new Pose(0,0,0);
  now(0);parks[0]=0;AutoPlan budget=new AutoPlan(f,()->true).add("work",AutonomousRegression::waiting,100,false,()->true).park(()->Command.build().setStart(()->parks[0]++));
  run=budget.command();run.start();now(22.1);run.execute();check(parks[0]==1&&!run.done(),"reserve time starts park");now(27.1);run.execute();check(run.done(),"global deadline bounds parking too");
  now(0);parks[0]=0;AutoPlan unsuccessful=new AutoPlan(f,()->true).add("failed visual acquisition",()->Command.build().setDone(()->true),2,false,()->false).park(()->Command.build().setStart(()->parks[0]++));
  run=unsuccessful.command();run.start();run.execute();check(parks[0]==1,"unsuccessful stage result routes to fallback");run.end(EndCondition.INTERRUPTED);near(f.forward,0,"cancel stops motors");
  // Both complete production OpModes are compiled alongside these helpers.
  check(new red_cycle()!=null&&new red_forward()!=null,"both autonomous OpModes construct");
  System.out.println("PASS: "+checks+" autonomous behavior checks");
 }
}'''


def main():
    java_home = os.environ.get('JAVA_HOME')
    suffix = '.exe' if os.name == 'nt' else ''
    javac = str(Path(java_home) / ('bin/javac' + suffix)) if java_home else shutil.which('javac')
    java = str(Path(java_home) / ('bin/java' + suffix)) if java_home else shutil.which('java')
    if not javac or not java:
        raise SystemExit('Set JAVA_HOME to a JDK, or add javac/java to PATH.')
    with tempfile.TemporaryDirectory(prefix='robot-autonomous-') as temp:
        root = Path(temp)
        for name, source in STUBS.items():
            path = root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(source)
        for name in ('AutoPlan', 'YellowBallSeeker', 'HiveAim', 'red_cycle', 'red_forward'):
            path = root / 'org/firstinspires/ftc/teamcode/Auton' / (name + '.java')
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text((support.MAIN / 'Auton' / (name + '.java')).read_text())
        path = root / 'org/firstinspires/ftc/teamcode/subsystems/Limelight.java'
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text((support.MAIN / 'subsystems/Limelight.java').read_text())
        (root / 'org/firstinspires/ftc/teamcode/Auton/AutonomousRegression.java').write_text(TEST)
        subprocess.run([javac, '-d', str(root / 'classes')] + [str(p) for p in root.rglob('*.java')], check=True)
        subprocess.run([java, '-cp', str(root / 'classes'), 'org.firstinspires.ftc.teamcode.Auton.AutonomousRegression'], check=True)


if __name__ == '__main__':
    main()

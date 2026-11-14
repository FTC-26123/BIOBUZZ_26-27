package org.firstinspires.ftc.teamcode.Auton;
import java.lang.reflect.*;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
public class ShiftedRouteChecks {
 static class Auto extends TrainedFlowerAuto {
  final boolean blue;Auto(boolean blue){this.blue=blue;}protected boolean blueAlliance(){return blue;}
 }
 static Field field(String n)throws Exception{Field f=TrainedFlowerAuto.class.getDeclaredField(n);f.setAccessible(true);return f;}
 static Object call(Auto a,String n,Class<?>[] types,Object... args)throws Exception{
  Method m=TrainedFlowerAuto.class.getDeclaredMethod(n,types);m.setAccessible(true);return m.invoke(a,args);
 }
 static Pose point(Auto a,int leg,int xy,double h)throws Exception{return (Pose)call(a,"curvePoint",new Class[]{int.class,int.class,double.class},leg,xy,h);}
 public static void main(String[] args)throws Exception{
  int checks=0;
  for(boolean blue:new boolean[]{false,true}){
   Auto a=new Auto(blue);double[][] curves=(double[][])field("CURVES").get(a);
   Field state=field("state");
   for(int leg=0;leg<curves.length;leg++){
    field("leg").set(a,leg);boolean push=leg==3||leg==9;
    state.set(a,Enum.valueOf((Class)state.getType(),push?"PUSH":"DRIVE"));
    Pose from=point(a,leg,0,curves[leg][8]),to=point(a,leg,6,curves[leg][9]);
    if(push)to=new Pose(to.x()+.5*Math.cos(to.heading()),to.y()+.5*Math.sin(to.heading()),to.heading());
    Path path=(push||leg==4||leg==10)?Paths.line(from,to).linear(from,to)
      :Paths.curve(from,point(a,leg,2,0),point(a,leg,4,0),to).linear(from,to);
    for(int i=0;i<=2000;i++){
     Pose p=path.get(i/2000.0);
     if(!(Boolean)call(a,"clear",new Class[]{Pose.class},p))throw new AssertionError("Not clear: blue="+blue+", leg="+(leg+1)+", t="+(i/2000.0)+", pose="+p);
     checks++;
    }
   }
  }
  System.out.println("PASS: "+checks+" nominal-route clearance samples using actual Pedro curves, both alliances and both shifted approaches. Geometry only; no physical pickup claim.");
 }
}

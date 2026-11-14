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
    double depth=2.5;
    if(push)to=new Pose(to.x()+depth*Math.cos(to.heading()),to.y()+depth*Math.sin(to.heading()),to.heading());
    if(leg==4||leg==10)from=new Pose(from.x()+depth*Math.cos(from.heading()),from.y()+depth*Math.sin(from.heading()),from.heading());
    Path path=(push||leg==4||leg==10)?Paths.line(from,to).linear(from,to)
      :Paths.curve(from,point(a,leg,2,0),point(a,leg,4,0),to).linear(from,to);
    for(int i=0;i<=2000;i++){
     Pose p=path.get(i/2000.0);
     if(!(Boolean)call(a,"clear",new Class[]{Pose.class},p))throw new AssertionError("Not clear: blue="+blue+", leg="+(leg+1)+", t="+(i/2000.0)+", pose="+p);
     checks++;
    }
   }
   for(int leg:new int[]{3,9}){
    field("leg").set(a,leg);state.set(a,Enum.valueOf((Class)state.getType(),"COLLECT"));
    Pose target=point(a,leg,6,curves[leg][9]);
    for(double depth:new double[]{2.5,3.065})for(double side:new double[]{-5,5})for(double degrees:new double[]{-15,15}){
     double h=target.heading();Pose contact=new Pose(target.x()+depth*Math.cos(h)-side*Math.sin(h),target.y()+depth*Math.sin(h)+side*Math.cos(h),h+Math.toRadians(degrees));
     if(!(Boolean)call(a,"clear",new Class[]{Pose.class},contact))throw new AssertionError("Deep-contact recovery envelope is not clear: "+contact);
     checks++;
    }
    for(double degrees:new double[]{-30,30}){
     double h=target.heading();
     Pose inboard=new Pose(target.x()+.165*Math.cos(h),target.y()+.165*Math.sin(h),h+Math.toRadians(degrees));
     if(!(Boolean)call(a,"clear",new Class[]{Pose.class},inboard))throw new AssertionError("Inboard contact turn should permit recovery");
     Pose nearWall=new Pose(target.x()+3.065*Math.cos(h),target.y()+3.065*Math.sin(h),h+Math.toRadians(degrees));
     if((Boolean)call(a,"clear",new Class[]{Pose.class},nearWall))throw new AssertionError("Actual chassis wall envelope must still reject deeply rotated contact");
     checks+=2;
    }
   }
   field("leg").set(a,0);state.set(a,Enum.valueOf((Class)state.getType(),"DRIVE"));
   Pose start=point(a,0,0,curves[0][8]),end=point(a,0,6,curves[0][9]);double sign=blue?-1:1;
   for(double dx:new double[]{-.25,.25})for(double angle:new double[]{-2,2}){
    Pose measured=new Pose(start.x()+sign*dx,start.y()-sign*.25,start.heading()+Math.toRadians(angle));
    Pose first=point(a,0,2,0);first=new Pose(first.x()+measured.x()-start.x(),first.y()+measured.y()-start.y());
    Path departure=Paths.curve(measured,first,point(a,0,4,0),end).linear(measured,end);
    for(int i=0;i<=2000;i++){
     if(!(Boolean)call(a,"clear",new Class[]{Pose.class},departure.get(i/2000.0)))throw new AssertionError("Noisy startup departure is not clear");
     checks++;
    }
   }
   field("leg").set(a,5);state.set(a,Enum.valueOf((Class)state.getType(),"FALLBACK"));
   Pose stage=point(a,4,6,curves[4][9]);Pose actual=new Pose(stage.x()+sign*2.9,stage.y(),stage.heading());
   java.util.List<Pose> escape=(java.util.List<Pose>)call(a,"parkingWaypoints",new Class[]{Pose.class},actual);
   for(Pose waypoint:escape){
    Path path=Paths.line(actual,waypoint).linear(actual,waypoint);
    for(int i=0;i<=1000;i++){
     if(!(Boolean)call(a,"clear",new Class[]{Pose.class},path.get(i/1000.0)))throw new AssertionError("Measured-pose parking escape is not clear");
     checks++;
    }
    actual=waypoint;
   }
  }
  System.out.println("PASS: "+checks+" route, deep-contact, noisy-startup and parking clearance samples using actual Pedro curves on both alliances. Geometry only; no physical pickup claim.");
 }
}

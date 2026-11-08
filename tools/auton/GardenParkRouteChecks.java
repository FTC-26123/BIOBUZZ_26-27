package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.math.Pose;
import java.util.List;
import java.util.Locale;

public final class GardenParkRouteChecks {
    private static int checks;
    private static void check(boolean ok,String why){checks++;if(!ok)throw new AssertionError(why);}
    private static void footprint(Pose p,boolean blue){
        double half=.2286*(Math.abs(Math.cos(p.heading()))+Math.abs(Math.sin(p.heading())));
        double x=p.x()*.0254,y=p.y()*.0254,s=blue?-1:1;
        check(Math.abs(x)+half<=1.792859+1e-6&&Math.abs(y)+half<=1.792859+1e-6,"wall clearance");
        check(FrameClearance.clear(x,y),"frame clearance");
        for(double[] pot:new double[][]{{-.5942,1.7283},{1.7283,.5942},{.5942,-1.7283},{-1.7283,-.5942}}){
            double dx=pot[0]-x,dy=pot[1]-y,c=Math.cos(p.heading()),sn=Math.sin(p.heading());
            double outsideX=Math.max(0,Math.abs(dx*c+dy*sn)-.2286);
            double outsideY=Math.max(0,Math.abs(-dx*sn+dy*c)-.2286);
            check(Math.hypot(outsideX,outsideY)>.08,"fixed flower pot clearance");
        }
        check(Math.abs(x+1.55*s)>half+.2286||Math.abs(y-1.19*s)>half+.2286,"teammate footprint");
    }
    private static Pose interpolate(Pose from,Pose to,double t){
        double turn=Math.atan2(Math.sin(to.heading()-from.heading()),Math.cos(to.heading()-from.heading()));
        return new Pose(from.x()+(to.x()-from.x())*t,from.y()+(to.y()-from.y())*t,from.heading()+turn*t);
    }
    private static void verify(Pose from,List<Pose> path,boolean blue){
        for(Pose next:path){for(int i=0;i<=100;i++)footprint(interpolate(from,next,i/100.0),blue);from=next;}
    }
    private static void same(Pose a,Pose b){check(Math.hypot(a.x()-b.x(),a.y()-b.y())<1e-8&&Math.abs(a.heading()-b.heading())<1e-8,"working opening unchanged");}
    private static void row(String name,String phase,Pose pose){
        System.out.printf(Locale.US,"%s\t%s\t%.9f\t%.9f\t%.9f%n",name,phase,pose.x(),pose.y(),Math.toDegrees(pose.heading()));
    }
    public static void main(String[] args){
        boolean export=args.length>0&&args[0].equals("--export");
        GardenParkRoute red=new GardenParkRoute(false),blue=new GardenParkRoute(true);
        for(int i=0;i<red.stages.size();i++){
            Pose a=red.stages.get(i).pose(),b=blue.stages.get(i).pose();
            check(Math.hypot(a.x()+b.x(),a.y()+b.y())<1e-8,"all stages mirror by 180 degrees");
        }
        for(GardenParkRoute route:new GardenParkRoute[]{red,blue}){
            CycleRoute original=new CycleRoute(route.blue?CycleRoute.Start.BLUE_WEST:CycleRoute.Start.RED_EAST,
                CycleRoute.Strategy.NO_FLOWERS,CycleRoute.Lineup.FOUR_PARK);
            same(route.initial,original.initial);
            same(route.stages.get(0).pose(),original.stages.get(0).pose());
            String name=route.blue?"BlueGardenPark":"RedGardenPark";
            Pose previous=route.initial;int fire=0,sweeps=0;
            if(export)row(name,"Start",previous);
            for(CycleRoute.Stage stage:route.stages){
                if(stage.action()==CycleRoute.Action.FIRE){fire++;check(stage.count()==4,"four per batch");}
                if(stage.action()==CycleRoute.Action.SWEEP)sweeps++;
                check(stage.action()!=CycleRoute.Action.FLOWER_COLLECT&&stage.action()!=CycleRoute.Action.VISION_COLLECT,"no flowers or vision");
                check(stage.pose().x()*.0254*(route.blue?-1:1)<0&&stage.pose().y()*.0254*(route.blue?-1:1)<-.9,"no opponent garden or far-basket target");
                List<Pose> path=FrameClearance.path(previous,stage.pose(),route.blue);
                verify(previous,path,route.blue);
                Pose from=previous;
                for(Pose next:path){
                    for(int i=0;i<=10;i++){
                        Pose sample=interpolate(from,next,i/10.0);
                        verify(sample,route.parkingPath(sample),route.blue);
                        check(Double.isFinite(route.parkReserve(sample))&&route.parkReserve(sample)>0,"finite parking reserve");
                    }
                    if(export)row(name,stage.name(),next);from=next;
                }
                previous=stage.pose();
            }
            check(fire==1&&sweeps==1,"one four-preload batch and only our garden");
            List<Pose> park=route.parkingPath(previous);verify(previous,park,route.blue);
            List<Pose> curve=new java.util.ArrayList<>();for(AutoPaths.Leg l:route.parkingLegs(previous))curve.addAll(l.sample(200));verify(previous,curve,route.blue);
            if(export)for(Pose next:park)row(name,"Park",next);
            check(route.parkReserve(previous)>0,"positive parking reserve");
        }
        if(!export)System.out.println("PASS "+checks+" garden-park geometry, in-flight fallback paths, mirroring, opening and timing checks");
    }
}

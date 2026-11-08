package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.math.Pose;
import java.util.List;

public final class DelayedBasketTwoChecks {
    private static int checks;
    private static void check(boolean ok,String why){checks++;if(!ok)throw new AssertionError(why);}
    private static void verify(Pose from,List<Pose> path,boolean blue){
        double sign=blue?-1:1;
        for(Pose to:path){
            double turn=Math.atan2(Math.sin(to.heading()-from.heading()),Math.cos(to.heading()-from.heading()));
            for(int i=0;i<=200;i++){
                double u=i/200.0,x=(from.x()+(to.x()-from.x())*u)*.0254,y=(from.y()+(to.y()-from.y())*u)*.0254;
                double h=from.heading()+turn*u,half=.2286*(Math.abs(Math.cos(h))+Math.abs(Math.sin(h)));
                check(Math.abs(x)+half<=1.792859+1e-6&&Math.abs(y)+half<=1.792859+1e-6,"wall footprint");
                check(FrameClearance.clear(x,y),"metal frame keepout");
                check(Math.abs(x+1.55*sign)>half+.2286||Math.abs(y-.64*sign)>half+.2286,"reserved teammate footprint");
                for(double[] pot:new double[][]{{-.5942,1.7283},{1.7283,.5942},{.5942,-1.7283},{-1.7283,-.5942}}){
                    double dx=pot[0]-x,dy=pot[1]-y,c=Math.cos(h),s=Math.sin(h);
                    double a=Math.max(0,Math.abs(dx*c+dy*s)-.2286),b=Math.max(0,Math.abs(-dx*s+dy*c)-.2286);
                    check(Math.hypot(a,b)>.08,"fixed flower pot clearance");
                }
            }
            from=to;
        }
    }
    public static void main(String[] args){
        check(!DelayedBasketTwoRoute.readyToFeed(14.999,10),"never feed early");
        check(!DelayedBasketTwoRoute.readyToFeed(15,.249),"must settle before feeding");
        check(DelayedBasketTwoRoute.readyToFeed(15,.25),"feed opens at fifteen seconds with settled aim");
        check(!DelayedBasketTwoRoute.readyToFeed(Double.NaN,1),"invalid time never feeds");
        DelayedBasketTwoRoute red=new DelayedBasketTwoRoute(false),blue=new DelayedBasketTwoRoute(true);
        for(Pose[] pair:new Pose[][]{{red.initial,blue.initial},{red.shot,blue.shot}})
            check(Math.hypot(pair[0].x()+pair[1].x(),pair[0].y()+pair[1].y())<1e-8,"180 degree coordinate mirror");
        check(Math.abs(blue.park.x()+70.75-127.88)<1e-9&&Math.abs(blue.park.y()+70.75-21.27)<1e-9,"requested Blue park coordinates");
        check(blue.parkingPath(blue.shot).size()==1,"Blue drives straight from shooting to park");

        check(Math.abs(red.park.x()+70.75-13.62)<1e-9&&Math.abs(red.park.y()+70.75-120.23)<1e-9,"Red park mirrors Blue (start-side half of the zone)");
        check(red.parkingPath(red.shot).size()==1,"Red drives straight from shooting to park");
        for(DelayedBasketTwoRoute route:new DelayedBasketTwoRoute[]{red,blue}){
            double sign=route.blue?-1:1;
            Pose near=new CycleRoute(route.blue?CycleRoute.Start.BLUE_WEST:CycleRoute.Start.RED_EAST,
                CycleRoute.Strategy.NO_FLOWERS,CycleRoute.Lineup.FOUR_PARK).stages.get(1).pose();
            double nearRange=Math.hypot(near.x()*.0254+.32385*sign,near.y()*.0254+.392176*sign);
            double farRange=Math.hypot(route.shot.x()*.0254+.32385*sign,route.shot.y()*.0254-.392176*sign);
            check(Math.abs(nearRange-farRange)<1e-10,"same standoff as current Basket 1 mark");
            check(Math.abs(route.initial.y()*.0254-1.564259*sign)<1e-9,"opposite wall start");
            Pose previous=route.initial;
            for(Pose next:route.approachPath()){
                verify(previous,java.util.Collections.singletonList(next),route.blue);
                for(int k=0;k<=10;k++){
                    double u=k/10.0,turn=Math.atan2(Math.sin(next.heading()-previous.heading()),Math.cos(next.heading()-previous.heading()));
                    Pose sample=new Pose(previous.x()+(next.x()-previous.x())*u,previous.y()+(next.y()-previous.y())*u,previous.heading()+turn*u);
                    verify(sample,route.parkingPath(sample),route.blue);
                    check(Double.isFinite(route.parkReserve(sample))&&route.parkReserve(sample)>0,"finite fallback reserve");
                }
                previous=next;
            }
            verify(route.shot,route.parkingPath(route.shot),route.blue);
            check(15+route.parkReserve(route.shot)<29.5,"time remains after wait for shooting");
        }
        if(args.length>0&&args[0].equals("--export")){
            for(DelayedBasketTwoRoute route:new DelayedBasketTwoRoute[]{red,blue}){
                String name=(route.blue?"Blue":"Red")+"Basket2Wait15Shoot4Park";
                row(name,"Start",route.initial);
                for(Pose p:route.approachPath())row(name,"Aim for Basket 2; feed after 15 seconds",p);
                for(Pose p:route.parkingPath(route.shot))row(name,"Park",p);
            }
        }else System.out.println("PASS "+checks+" delayed Basket 2 timing, distance, mirror and swept/fallback clearance checks");
    }
    private static void row(String name,String stage,Pose p){
        System.out.printf(java.util.Locale.US,"%s\t%s\t%.9f\t%.9f\t%.9f%n",name,stage,p.x(),p.y(),Math.toDegrees(p.heading()));
    }
}

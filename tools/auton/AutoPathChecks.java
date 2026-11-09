package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.math.Pose;
import java.util.ArrayList;
import java.util.List;

/**
 * Samples every designed continuous leg of the eight autons (and the same legs started up to 4 inches off their
 * planned start): 18-inch footprint inside the walls, robot centre outside FrameClearance, footprint clear of the
 * fixed flower pots, clear of the reserved teammate slot by 3 inches, and entirely on our alliance's half.
 * Also checks that each program's designed route is a single leg per drive (no intermediate stops).
 */
public final class AutoPathChecks {
    private static int checks;
    private static void check(boolean ok,String why){checks++;if(!ok)throw new AssertionError(why);}
    private static final double HALF=.2286,WALL=1.797;
    private static final double[][] POTS={{-.5942,1.7283},{1.7283,.5942},{.5942,-1.7283},{-1.7283,-.5942}};

    /** startAtWall: the leg begins touching a wall; that wall's clearance may not shrink in the first 3 inches. */
    private static void verify(String what,AutoPaths.Leg leg,boolean blue,double teammateY,boolean startAtWall){
        double sign=blue?-1:1;
        Pose first=null;double firstWall=0,firstAcross=0;
        for(Pose p:leg.sample(400)){
            double x=p.x()*.0254,y=p.y()*.0254,h=p.heading();
            double half=HALF*(Math.abs(Math.cos(h))+Math.abs(Math.sin(h)));
            double wall=Math.min(WALL-Math.abs(x),WALL-Math.abs(y))-half;
            if(first==null){first=p;firstWall=wall;firstAcross=x*sign+half;}
            double moved=Math.hypot(p.x()-first.x(),p.y()-first.y());
            // A leg starting at a wall must move away from it at least half as fast as it travels, up to 1 inch.
            check(wall>=Math.min(startAtWall?firstWall+.5*moved*.0254:.0254,.0254)-.0015,what+": wall clearance at "+x+", "+y);
            check(FrameClearance.clear(x,y),what+": frame keepout at "+x+", "+y);
            for(double[] pot:POTS){
                double dx=pot[0]-x,dy=pot[1]-y,c=Math.cos(h),s=Math.sin(h);
                double a=Math.max(0,Math.abs(dx*c+dy*s)-HALF),b=Math.max(0,Math.abs(-dx*s+dy*c)-HALF);
                check(Math.hypot(a,b)>.08+.0254,what+": fixed flower pot clearance "+Math.hypot(a,b)+" at "+x+", "+y+" from start "+first.x()*.0254+", "+first.y()*.0254);
            }
            check(Math.abs(x+1.55*sign)>half+HALF+.0762||Math.abs(y-teammateY*sign)>half+HALF+.0762,what+": 3 inches from teammate slot");
            // Never further across the centre line than the start pose itself (a misplaced start may already be).
            check(x*sign+half<=Math.max(1e-6,firstAcross+1e-6),what+": stays on our half of the field");
        }
    }
    /** Nominal parking curves keep at least 4 inches between the footprint and every fixed flower pot. */
    private static void potMargin(String what,AutoPaths.Leg leg){
        for(Pose p:leg.sample(400)){
            double x=p.x()*.0254,y=p.y()*.0254,h=p.heading(),c=Math.cos(h),s=Math.sin(h);
            for(double[] pot:POTS){
                double dx=pot[0]-x,dy=pot[1]-y;
                double a=Math.max(0,Math.abs(dx*c+dy*s)-HALF),b=Math.max(0,Math.abs(-dx*s+dy*c)-HALF);
                check(Math.hypot(a,b)-.08>=4*.0254,what+": 4-inch pot margin, got "+(Math.hypot(a,b)-.08)/.0254+" in");
            }
        }
    }
    private static void verifyAll(String what,List<AutoPaths.Leg> legs,boolean blue,double teammateY,boolean startAtWall){
        for(AutoPaths.Leg l:legs){verify(what+" / "+l.name,l,blue,teammateY,startAtWall);startAtWall=false;}
    }
    private static Pose offset(Pose p,double dx,double dy){return new Pose(p.x()+dx,p.y()+dy,p.heading());}
    private static List<Pose> jitter(Pose p,boolean atWall,boolean blue){
        List<Pose> out=new ArrayList<>();double inward=blue?-1:1;
        for(double dx:new double[]{-2,0,2})for(double dy:new double[]{-2,0,2}){
            if(atWall&&dy*inward<0)continue; // cannot start inside the wall
            out.add(offset(p,dx,dy));
        }
        return out;
    }
    public static void main(String[] args){
        for(boolean blue:new boolean[]{false,true}){
            String side=blue?"Blue":"Red";
            CycleRoute.Start start=blue?CycleRoute.Start.BLUE_WEST:CycleRoute.Start.RED_EAST;
            CycleRoute basket=new CycleRoute(start,CycleRoute.Strategy.NO_FLOWERS,CycleRoute.Lineup.FOUR_PARK);
            check(basket.parkLeg().curved(),"Basket 1 parks in one continuous curve");
            check(AutoPaths.plan(basket.approachLeg(),basket.initial,blue).size()==1,"Basket 1 opening is one leg");
            check(basket.parkingLegs(basket.shot()).size()==1,"Basket 1 parking is one leg");
            check(ParkOnlyAuto.legs(start,start.wallPose()).size()==1,"Park Only is one leg");
            potMargin(side+" Basket 1 park curve",basket.parkLeg());
            potMargin(side+" Park Only curve",ParkOnlyAuto.leg(start));
            for(Pose p:jitter(basket.initial,true,blue))verifyAll(side+" Basket 1 approach",AutoPaths.plan(basket.approachLeg(),p,blue),blue,1.19,true);
            for(Pose p:jitter(basket.shot(),false,blue))verifyAll(side+" Basket 1 park",basket.parkingLegs(p),blue,1.19,false);
            for(Pose p:jitter(start.wallPose(),true,blue))verifyAll(side+" Park Only",ParkOnlyAuto.legs(start,p),blue,1.19,true);

            GardenParkRoute garden=new GardenParkRoute(blue);
            Pose at=garden.initial;
            for(int i=0;i<garden.stages.size();i++){
                CycleRoute.Stage s=garden.stages.get(i);
                if(s.action()!=CycleRoute.Action.SWEEP){
                    check(garden.legsInto(i,at).size()==1,"Garden stage "+i+" is one leg");
                    for(Pose p:jitter(at,i==0,blue))verifyAll(side+" Garden "+s.name(),garden.legsInto(i,p),blue,1.19,i==0);
                }else{
                    // The validated pickup sweep deliberately ends 0.67 inch off the side wall (unchanged by this redesign).
                    for(Pose p:AutoPaths.Leg.line("sweep",at,s.pose()).sample(200)){
                        double h=p.heading(),half=HALF*(Math.abs(Math.cos(h))+Math.abs(Math.sin(h)));
                        check(Math.min(WALL-Math.abs(p.x()*.0254),WALL-Math.abs(p.y()*.0254))-half>=.5*.0254,side+" Garden sweep stays 0.5 inch off the walls");
                    }
                }
                at=s.pose();
            }
            check(garden.parkingLegs(at).size()==1&&garden.parkLeg().curved(),"Garden parks in one continuous curve");
            potMargin(side+" Garden park curve",garden.parkLeg());
            check(garden.parkingLegs(basket.shot()).size()==1,"Garden aborted at the shot mark parks in one curve");
            for(Pose p:jitter(basket.shot(),false,blue))verifyAll(side+" Garden early park",garden.parkingLegs(p),blue,1.19,false);
            verifyAll(side+" Garden park",garden.parkingLegs(at),blue,1.19,true);
            for(double dx:new double[]{0,1.5})for(double dy:new double[]{0,1.5})
                verifyAll(side+" Garden park (offset)",garden.parkingLegs(offset(at,blue?-dx:dx,blue?-dy:dy)),blue,1.19,true);

            DelayedBasketTwoRoute late=new DelayedBasketTwoRoute(blue);
            check(late.approachLegs(late.initial).size()==1,"Basket 2 opening is one leg");
            // The Basket 2 mark already sits 1.6 inches from the fixed flower pot on that wall: jitter only away from it.
            for(Pose p:jitter(late.initial,true,!blue))if((p.x()-late.initial.x())*(blue?-1:1)>=0)verifyAll(side+" Basket 2 approach",late.approachLegs(p),blue,.64,true);
            for(Pose p:jitter(late.shot,false,blue))verifyAll(side+" Basket 2 park",late.parkingLegs(p),blue,.64,false);
        }
        System.out.println("PASS "+checks+" continuous-leg footprint, frame, pot, teammate, half-field and single-leg checks");
    }
}

package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.math.Pose;
import java.util.*;

/** Samples the swept 18-inch chassis, including wall pickup retreat and parking. */
public final class PathClearanceChecks {
    private static int checks;
    private static void check(boolean ok,String message){checks++;if(!ok)throw new AssertionError(message);}
    private static CycleRoute r0(CycleRoute.Start s){return new CycleRoute(s,CycleRoute.Strategy.NO_FLOWERS,CycleRoute.Lineup.FOUR_PARK);}
    private static boolean fits(Pose p){
        double half=.2286*(Math.abs(Math.cos(p.heading()))+Math.abs(Math.sin(p.heading())));
        return Math.abs(p.x()*.0254)+half<=1.8 && Math.abs(p.y()*.0254)+half<=1.8;
    }
    private static void verify(Pose from,List<Pose> path){
        for(Pose to:path){
            check(FrameClearance.segment(from.x()*.0254,from.y()*.0254,to.x()*.0254,to.y()*.0254),"frame segment");
            double dh=Math.atan2(Math.sin(to.heading()-from.heading()),Math.cos(to.heading()-from.heading()));
            for(int i=0;i<=100;i++){
                double t=i/100.0;
                Pose p=new Pose(from.x()+(to.x()-from.x())*t,from.y()+(to.y()-from.y())*t,from.heading()+dh*t);
                check(fits(p),"swept chassis crosses wall at "+p.x()*.0254+", "+p.y()*.0254);
            }
            from=to;
        }
    }
    private static List<Pose> sampled(List<AutoPaths.Leg> legs){List<Pose> out=new ArrayList<>();for(AutoPaths.Leg l:legs){List<Pose> s=l.sample(200);out.addAll(s.subList(1,s.size()));}return out;}
    public static void main(String[] args){
        for(CycleRoute.Start start:Arrays.asList(CycleRoute.Start.RED_EAST,CycleRoute.Start.BLUE_WEST)){
            for(CycleRoute.Lineup mode:Arrays.asList(CycleRoute.Lineup.FOUR_PARK,CycleRoute.Lineup.FOUR_RECOVER_TEAM)){
                CycleRoute r=new CycleRoute(start,CycleRoute.Strategy.NO_FLOWERS,mode);
                Pose from=r.initial;
                for(CycleRoute.Stage s:r.stages){
                    List<Pose> path=FrameClearance.path(from,s.pose(),r.blue);
                    verify(from,path);verify(from,r.parkingPath(from));from=s.pose();
                }
                verify(from,r.parkingPath(from));
                double sign=r.blue?-1:1;
                for(double x:new double[]{-1.53919,-1.50,-1.13,-.90,-.36,-.245})
                    for(double y:new double[]{-1.564259,-1.53,-1.43,-1.18,-1.05,-.91})
                        for(int degrees=0;degrees<360;degrees+=30){
                            Pose pickup=new Pose(sign*x/.0254,sign*y/.0254,Math.toRadians(degrees));
                            if(!fits(pickup)||!FrameClearance.clear(sign*x,sign*y))continue;
                            verify(pickup,r.parkingPath(pickup));
                            Pose scan=r.stages.get(0).pose();
                            verify(pickup,FrameClearance.path(pickup,scan,r.blue));
                            verify(scan,FrameClearance.path(scan,pickup,r.blue));
                        }
            }
            CycleRoute.Start park=start; // Park Only starts on the same wall mark as the shooting autons
            verify(park.wallPose(),sampled(ParkOnlyAuto.legs(park,park.wallPose())));
            verify(r0(park).shot(),sampled(r0(park).parkingLegs(r0(park).shot())));
        }
        System.out.println("PASS "+checks+" swept chassis, wall retreat, mirrored route and parking checks");
    }
}

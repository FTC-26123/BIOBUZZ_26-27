package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.math.Pose;
import java.util.*;

/** Conservative axis bounds of our rotated chassis against a parked 18-inch teammate. */
public final class TeammateParkingChecks {
    private static int checks;
    private static void check(boolean ok,String why){checks++;if(!ok)throw new AssertionError(why);}
    private static void verify(Pose from,List<Pose> path,Pose teammate){
        for(Pose to:path){
            double turn=Math.atan2(Math.sin(to.heading()-from.heading()),Math.cos(to.heading()-from.heading()));
            for(int i=0;i<=200;i++){
                double t=i/200.0,h=from.heading()+turn*t;
                double x=(from.x()+(to.x()-from.x())*t)*.0254;
                double y=(from.y()+(to.y()-from.y())*t)*.0254;
                double half=.2286*(Math.abs(Math.cos(h))+Math.abs(Math.sin(h)));
                check(Math.abs(x-teammate.x()*.0254)>half+.2286
                    ||Math.abs(y-teammate.y()*.0254)>half+.2286,"our approach overlaps parked teammate");
            }
            from=to;
        }
    }
    public static void main(String[] args){
        for(boolean blue:new boolean[]{false,true}){
            CycleRoute.Start start=blue?CycleRoute.Start.BLUE_WEST:CycleRoute.Start.RED_EAST;
            CycleRoute r=new CycleRoute(start,CycleRoute.Strategy.NO_FLOWERS,CycleRoute.Lineup.FOUR_PARK);
            Pose teammate=start.teammateParking();
            double gap=Math.abs(r.park.y()-teammate.y())*.0254-.4572;
            check(gap>.09,"at least 9 cm gap between parked 18-inch footprints");
            check(Math.abs(r.park.y()*.0254*(blue?-1:1)-.64)<1e-9,"our robot uses near slot");
            check(Math.abs(teammate.y()*.0254*(blue?-1:1)-1.19)<1e-9,"teammate uses far slot");
            Pose previous=r.initial;
            for(CycleRoute.Stage stage:r.stages){
                List<Pose> path=FrameClearance.path(previous,stage.pose(),blue);
                verify(previous,path,teammate);verify(previous,r.parkingPath(previous),teammate);
                previous=stage.pose();
            }
            verify(previous,r.parkingPath(previous),teammate);
            CycleRoute.Start park=blue?CycleRoute.Start.BLUE_EAST:CycleRoute.Start.RED_WEST;
            verify(park.wallPose(),ParkOnlyAuto.waypoints(park),teammate);
            check(Math.hypot(park.loadingZone().x()-r.park.x(),park.loadingZone().y()-r.park.y())<1e-9,"shooting and park-only share our assigned slot");
        }
        System.out.println("PASS "+checks+" teammate parking gap, mirrored destinations and occupied-slot approach checks");
    }
}

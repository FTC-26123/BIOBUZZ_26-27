package org.firstinspires.ftc.teamcode.Auton.cycles;
import com.pedropathing.math.Pose;
public final class FixedShotChecks {
    private static int checks;
    private static void check(boolean ok,String why){checks++;if(!ok)throw new AssertionError(why);}
    public static void main(String[] args){
        for(boolean blue:new boolean[]{false,true}){
            CycleRoute r=new CycleRoute(blue?CycleRoute.Start.BLUE_WEST:CycleRoute.Start.RED_EAST,CycleRoute.Strategy.NO_FLOWERS,CycleRoute.Lineup.FOUR_PARK);
            Pose shot=r.stages.get(1).pose();
            check(Math.abs(shot.x()+70.75-(blue?84.45:57.05))<1e-8,"exact requested visualizer X");
            check(Math.abs(shot.y()+70.75-(blue?113.02:28.48))<1e-8,"exact requested visualizer Y");
            check(r.stages.get(1).action()==CycleRoute.Action.FIRE,"stationary fixed shot");
            check(r.stages.get(1).count()==4,"four requested preloads");
            check(r.parkingPath(shot).size()>0,"park after shooting");
        }
        PreloadShotCounter c=new PreloadShotCounter();double now=0;
        // Motor noise without actual feeding cannot acknowledge a release.
        for(int i=0;i<30;i++){now+=.02;c.update(now,1500,i%2==0?1500:1300,2800,false);}
        check(c.shots==0,"no feeding, no release");
        // Feed travel without a flywheel load is not a ball measurement.
        for(int i=0;i<30;i++){now+=.02;c.update(now,1500,1500,2800,true);}
        check(c.shots==0,"time alone cannot count shots");
        for(int shot=1;shot<=4;shot++){
            for(int i=0;i<25;i++){now+=.02;c.update(now,1500,1500,2800,true);}
            now+=.02;c.update(now,1500,1380,2800,true);
            check(c.shots==shot,"load acknowledges one release");
            for(int i=0;i<10;i++){now+=.02;c.update(now,1500,1350,2800,true);}
            check(c.shots==shot,"sustained sag cannot double count");
        }
        PreloadShotCounter stopped=new PreloadShotCounter();
        stopped.update(0,1500,1500,0,true);stopped.update(.02,1500,1300,0,true);
        check(stopped.shots==0,"no encoder transport, no release");
        System.out.println("PASS "+checks+" fixed shot coordinates, Blue rotation, parking and release estimates");
    }
}

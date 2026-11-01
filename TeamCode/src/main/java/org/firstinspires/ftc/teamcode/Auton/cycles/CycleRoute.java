package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.math.Pose;
import java.util.*;

/** Field-centered inches, +X right, +Y up; Pedro heading 0 points +X. */
public final class CycleRoute {
    public enum Start { RED_WEST, RED_EAST, BLUE_EAST, BLUE_WEST }
    public enum Action { MOVE, COLLECT, FLOWER_COLLECT, FIRE, BONUS_FIRE, WAIT_TIP, VISION_COLLECT, VISION_NECTAR, SWEEP, ROLL_NEAR, ROLL_FAR, PARK }
    public enum Strategy { FLOWERS, TIPPED_POLLEN, ADAPTIVE_ALL, RUSH_TIPS, NO_FLOWERS }
    public static final class Stage {
        private final String name;
        private final Pose pose;
        private final Action action;
        private final int count;
        private final double limit;
        private final boolean mixedLoad;
        public Stage(String name, Pose pose, Action action, int count, double limit,boolean mixedLoad){this.name=name;this.pose=pose;this.action=action;this.count=count;this.limit=limit;this.mixedLoad=mixedLoad;}
        public String name(){return name;}
        public Pose pose(){return pose;}
        public Action action(){return action;}
        public int count(){return count;}
        public double limit(){return limit;}
        public boolean mixedLoad(){return mixedLoad;}

        public Stage(String name,Pose pose,Action action,int count,double limit){this(name,pose,action,count,limit,false);}
    }
    public final Start start;
    public final boolean blue;
    public final Strategy strategy;
    public final Pose initial;
    public final Pose park;
    public int bonusIndex;
    public final List<Stage> stages = new ArrayList<>();
    public CycleRoute(Start start) {
        this(start,Strategy.FLOWERS);
    }
    public CycleRoute(Start start,Strategy strategy) {
        this.start=start; this.strategy=strategy; blue=start==Start.BLUE_EAST||start==Start.BLUE_WEST;
        boolean east=start==Start.RED_EAST||start==Start.BLUE_WEST;
        // Both starts lie on the alliance half of the bottom/top perimeter, clear of flowers.
        initial=pose(rushStyle()?-.36:east?-.32385:-.90,-1.564259,90);
        park=rushStyle()?pose(-1.55,.90,90):pose(-1.42,strategy==Strategy.ADAPTIVE_ALL?.56:.9144,180);
        if(strategy==Strategy.RUSH_TIPS){rush();return;}
        if(strategy==Strategy.NO_FLOWERS){noFlowers();return;}
        add("Preload launch",-.32385,-1.55,90,Action.FIRE,3,6);
        add("Back away from launch",-.32385,-1.30,90,Action.MOVE,0,3);
        if(strategy!=Strategy.TIPPED_POLLEN) {
            add("Alliance-side flower",-1.415,-.5942,180,Action.FLOWER_COLLECT,4,5);
            add("Outside support lane",-1.13,-.5942,90,Action.MOVE,0,3);
        } else {
            add("Wait for first tip and falling pollen",-.32385,-1.30,90,Action.WAIT_TIP,0,2.5);
            add("Collect first released batch",-.32385,-1.30,90,Action.VISION_COLLECT,1,3);
            add("Retreat from hive",-.32385,-1.15,90,Action.MOVE,0,3);
            add("Avoid uncollected spill",-.95,-1.0,90,Action.MOVE,0,3);
        }
        add("Alliance-side crossing",strategy!=Strategy.TIPPED_POLLEN?-1.13:-.95,.98,90,Action.MOVE,0,5);
        add("Far launch approach",-.40,1.15,-90,Action.MOVE,0,4);
        add("Opposite-cell batch 1",-1.10,1.43,-53,Action.FIRE,4,7);
        add("Back away from far launch",-.40,1.30,-90,Action.MOVE,0,3);
        if(strategy==Strategy.TIPPED_POLLEN) {
            add("Align return corridor",-.26,1.0,-90,Action.MOVE,0,3);
            add("Look down into first spill",-.26,.15,-90,Action.MOVE,0,3);
            add("Recover corridor pollen",-.26,.15,-90,Action.VISION_COLLECT,1,2);
            add("Return to first spill",-.26,-.95,-90,Action.MOVE,0,4);
            add("Clear center line before turning",-.40,-1.10,-90,Action.MOVE,0,2);
            add("Turn clear of frame",-.40,-1.25,90,Action.MOVE,0,3);
            // Back up far enough for nearby settled nectar to clear the lower image edge.
            add("Wider spill scan",-.60,-1.48,90,Action.MOVE,0,3);
            add("Collect remaining released pollen",-.60,-1.48,90,Action.VISION_COLLECT,1,2.5);
            add("Leave spill",-.32385,-1.15,90,Action.MOVE,0,3);
            add("Align corridor again",-.26,-.95,90,Action.MOVE,0,3);
            add("Last alliance crossing",-.26,1.0,90,Action.MOVE,0,4);
            add("Clear center line for far turn",-.40,1.10,90,Action.MOVE,0,2);
        } else {
            add("Clear far flower before turn",-.5942,1.10,-90,Action.MOVE,0,3);
            add("Face far flower",-.5942,1.10,90,Action.MOVE,0,3);
            add("Far flower",-.5942,1.415,90,Action.FLOWER_COLLECT,4,5);
            add("Back clear of far flower",-.5942,1.10,90,Action.MOVE,0,3);
        }
        add("Return launch approach",-.40,1.30,-90,Action.MOVE,0,3);
        add("Opposite-cell batch 2",-1.10,1.43,-53,Action.FIRE,strategy!=Strategy.TIPPED_POLLEN?4:2,7);
        bonusIndex=stages.size();
        if(strategy==Strategy.ADAPTIVE_ALL){
            add("Leave far shot",-.25,1.15,-90,Action.MOVE,0,3);
            add("Outside return lane",-1.13,.98,-90,Action.MOVE,0,3);
            add("Return toward garden",-1.13,-1.20,180,Action.MOVE,0,4);
            add("View preplaced garden pollen",-1.05,-1.564259,180,Action.MOVE,0,3);
            add("Vision-guided garden sweep",-1.50,-1.564259,180,Action.SWEEP,4,3);
            add("Clear garden wall",-1.30,-1.15,180,Action.MOVE,0,3);
            add("Diagonal shot to free capacity",-1.30,-1.30,Math.toDegrees(Math.atan2(1.30-.392176,1.30-.32385)),Action.BONUS_FIRE,1,4);
            add("Face remaining flower",-1.15,-.90,180,Action.MOVE,0,3);
            add("Collect remaining near flower",-1.415,-.5942,180,Action.FLOWER_COLLECT,1,4);
        }
        // Park as soon as the planned scoring is complete. No empty third-flower detour.
    }
    private void transit(String name,boolean near,int count,double x,double y){
        double h=Math.toDegrees(Math.atan2((near?-.392176:.392176)-y,-.32385-x));
        stages.add(new Stage(name,pose(x,y,h),near?Action.ROLL_NEAR:Action.ROLL_FAR,count,7.5,false));
    }
    private void rush(){
        add("Clear starting wall",-.36,-1.43,90,Action.MOVE,0,2);
        transit("Shoot preloads en route to near flower",true,3,-1.40,-1.43);
        add("Near flower",-1.415,-.5942,180,Action.FLOWER_COLLECT,3,4);
        transit("Shoot while crossing to far flower",false,4,-1.00,1.43);
        add("Far flower",-.5942,1.415,90,Action.FLOWER_COLLECT,4,4);
        transit("Shoot while returning to near flower",false,4,-1.40,1.43);
        add("Remaining near flower",-1.415,-.5942,180,Action.FLOWER_COLLECT,1,3);
        add("Face garden before third-tip pickup",-1.10,-1.35,210,Action.MOVE,0,2);
        add("Garden pollen for third tip",-1.50,-1.564259,180,Action.SWEEP,3,2.5);
        transit("Shoot garden batch en route to cleanup",true,4,-1.10,-1.35);
        add("Face remaining garden safely",-1.40,-1.43,180,Action.MOVE,0,2);
        add("Garden cleanup before final cycle",-1.50,-1.575,180,Action.COLLECT,0,2);
        add("View first spill",-.60,-.78,270,Action.MOVE,0,3);
        add("Recycle released pollen",-.60,-.78,270,Action.VISION_COLLECT,1,2);
        transit("Complete third tip en route to parking",true,4,-1.15,-1.22);
        bonusIndex=stages.size();
        add("View teleop pickup diagonally",-1.10,-1.35,210,Action.MOVE,0,3);
        add("Collect teleop preload",-1.50,-1.564259,180,Action.SWEEP,0,2.0);
    }
    public boolean rushStyle(){return strategy==Strategy.RUSH_TIPS||strategy==Strategy.NO_FLOWERS;}
    private void noFlowers(){
        add("Clear starting wall",-.36,-1.43,90,Action.MOVE,0,2);
        transit("Shoot preloads toward garden",true,3,-1.40,-1.43);
        add("Face garden pollen",-1.10,-1.35,210,Action.MOVE,0,2);
        add("Collect first garden batch",-1.50,-1.564259,180,Action.SWEEP,3,3);
        transit("Score garden pollen while crossing",false,4,-1.00,1.43);
        add("Return outside frame to garden",-1.40,-1.43,180,Action.MOVE,0,4);
        add("Garden cleanup without pots",-1.50,-1.575,180,Action.COLLECT,0,2);
        add("Stay outside near support",-1.13,-1.05,90,Action.MOVE,0,3);
        add("View settled pollen outside metal frame",-1.13,-.78,0,Action.MOVE,0,2);
        add("Recover settled first-tip pollen",-1.13,-.78,0,Action.VISION_COLLECT,3,4);
        transit("Complete far-cell tip toward parking",false,4,-1.00,1.43);
        add("Scan far-end floor pollen",-1.05,.95,90,Action.MOVE,0,2);
        add("Recover far-end floor pollen",-1.05,.95,90,Action.VISION_COLLECT,2,2.5);
        transit("Score recovered floor pollen toward park",false,2,-1.50,1.12);
        bonusIndex=stages.size();
    }
    private void add(String name,double x,double y,double h,Action a,int count,double limit) {
        stages.add(new Stage(name,pose(x,y,h),a,count,limit));
    }
    /** Collision-clear return corridor; points are generated from the current measured pose. */
    public List<Pose> parkingPath(Pose current) {
        if(strategy==Strategy.NO_FLOWERS)return FrameClearance.path(current,park,blue);
        double x=current.x()*.0254*(blue?-1:1), y=current.y()*.0254*(blue?-1:1);
        List<Pose> result=new ArrayList<>();
        if(y>1.25) {result.add(pose(x,1.15,-90));y=1.15;}
        if(x>-.95) result.add(pose(-1.15,Math.max(-1.15,Math.min(1.15,y)),180));
        result.add(park);
        return result;
    }
    public double parkReserve(Pose current) {
        double inches=0;Pose from=current;
        for(Pose to:parkingPath(current)){inches+=Math.hypot(to.x()-from.x(),to.y()-from.y());from=to;}
        // Include braking the fast pass, turning, and settling inside the loading zone.
        return rushStyle()?1.15+inches*.0254/1.20:strategy==Strategy.ADAPTIVE_ALL?1.45+inches*.0254/1.20:1.05+inches*.0254/.85;
    }
    private Pose pose(double x,double y,double heading) {
        return new Pose((blue?-x:x)/.0254,(blue?-y:y)/.0254,Math.toRadians(heading+(blue?180:0)));
    }
}

package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.math.Pose;
import java.util.*;

/** Field-centered inches, +X right, +Y up; Pedro heading 0 points +X. */
public final class CycleRoute {
    public enum Start {
        RED_WEST(false, false), RED_EAST(false, true),
        BLUE_EAST(true, false), BLUE_WEST(true, true);

        final boolean blue;
        final boolean inner;

        Start(boolean blue, boolean inner) {
            this.blue = blue;
            this.inner = inner;
        }

        double wallX() {
            return inner ? -.36 : -.90;
        }

        Pose wallPose() {
            return alliancePose(blue, wallX(), -1.564259, 90);
        }

        Pose loadingZone() {
            return alliancePose(blue, -1.55, .90, 90);
        }
    }
    public enum Lineup { LEGACY, SOLO, PARTNER, BASKET_ONE_TEAM, BASKET_TWO_TEAM,
        FOUR_PARK, FOUR_RECOVER_TEAM }
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
    public final Lineup lineup;
    private final boolean innerStart;
    public final Pose initial;
    public final Pose park;
    public int bonusIndex;
    public int basketRecoveryIndex;
    public int basketFallbackIndex;
    public int basketReloadIndex;
    public final List<Stage> stages = new ArrayList<>();
    public CycleRoute(Start start) {
        this(start,Strategy.FLOWERS);
    }
    public CycleRoute(Start start,Strategy strategy) {
        this(start,strategy,Lineup.LEGACY);
    }
    public CycleRoute(Start start,Strategy strategy,Lineup lineup) {
        this.start=start;
        this.strategy=strategy;
        this.lineup=lineup;
        blue=start.blue;
        innerStart=start.inner;
        double startX=start.wallX();
        if(lineup==Lineup.LEGACY){
            if(rushStyle())startX=-.36;
            else if(innerStart)startX=-.32385;
        }
        initial=lineup==Lineup.BASKET_TWO_TEAM?pose(-.90,1.564259,-90)
            :lineup==Lineup.LEGACY?pose(startX,-1.564259,90):start.wallPose();
        if(floorOnly()){
            park=pose(-1.55,.90,90);
            floorRoute();
            return;
        }
        if(basketTeam()){
            // Split the loading zone along its length so both robots can park.
            park=pose(-1.55,basketOne()?.66:1.17,90);
            basketTeamRoute();
            return;
        }
        park=partnerOpener()?pose(-.36,-1.48,90)
            :rushStyle()?start.loadingZone():pose(-1.42,strategy==Strategy.ADAPTIVE_ALL?.56:.9144,180);
        if(lineup==Lineup.PARTNER){partner(startX);return;}
        if(lineup==Lineup.SOLO){rush(startX);return;}
        if(strategy==Strategy.RUSH_TIPS){rush(startX);return;}
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
    private void transit(String name,boolean near,int count){
        double x=-1.13, y=near?-.85:.85; // 86-degree reachable outer intake approach.
        double h=Math.toDegrees(Math.atan2((near?-.392176:.392176)-y,-.32385-x));
        double limit=basketTeam()?30:lineup==Lineup.PARTNER?9.5:7.5;
        stages.add(new Stage(name,pose(x,y,h),near?Action.ROLL_NEAR:Action.ROLL_FAR,count,limit,false));
    }
    private void rush(double startX){
        add("Clear starting wall",startX,-1.43,90,Action.MOVE,0,2);
        transit("Shoot preloads en route to near flower",true,3);
        add("Near flower",-1.415,-.5942,180,Action.FLOWER_COLLECT,3,4);
        transit("Shoot while crossing to far flower",false,4);
        add("Far flower",-.5942,1.415,90,Action.FLOWER_COLLECT,4,4);
        transit("Shoot while returning to near flower",false,4);
        add("Remaining near flower",-1.415,-.5942,180,Action.FLOWER_COLLECT,1,3);
        add("Face garden before third-tip pickup",-1.10,-1.35,210,Action.MOVE,0,2);
        add("Garden pollen for third tip",-1.50,-1.564259,180,Action.SWEEP,3,2.5);
        transit("Shoot garden batch en route to cleanup",true,4);
        add("Face remaining garden safely",-1.40,-1.43,180,Action.MOVE,0,2);
        add("Garden cleanup before final cycle",-1.50,-1.575,180,Action.COLLECT,0,2);
        double spillScanX=avoidFrame()?-1.13:-.60;
        double spillScanHeading=avoidFrame()?0:270;
        add("View first spill",spillScanX,-.78,spillScanHeading,Action.MOVE,0,3);
        add("Recycle released pollen",spillScanX,-.78,spillScanHeading,Action.VISION_COLLECT,1,2);
        transit("Complete third tip en route to parking",true,4);
        bonusIndex=stages.size();
        add("View teleop pickup diagonally",-1.10,-1.35,210,Action.MOVE,0,3);
        add("Collect teleop preload",-1.50,-1.564259,180,Action.SWEEP,0,2.0);
    }
    public boolean rushStyle(){return strategy==Strategy.RUSH_TIPS||strategy==Strategy.NO_FLOWERS;}
    public boolean floorOnly(){return lineup==Lineup.FOUR_PARK||lineup==Lineup.FOUR_RECOVER_TEAM;}

    private void floorRoute(){
        add("Clear starting wall",-.36,-1.43,90,Action.MOVE,0,3);
        add("Shoot four preloads into Basket 1",-.36,-1.43,90,Action.ROLL_NEAR,4,12);
        if(lineup==Lineup.FOUR_RECOVER_TEAM){
            add("Move forward toward the spill",-.36,-1.18,90,Action.MOVE,0,3);
            add("Wait for Basket 1 to tip and settle",-.36,-1.18,90,Action.WAIT_TIP,0,10);
            add("Face the loose pollen spill",-1.12,-1.42,-22,Action.MOVE,0,3);
            add("Recover fallen pollen with Limelight",-1.12,-1.42,-22,Action.VISION_COLLECT,4,7);
            add("Wait for teammate and shoot Basket 1 again",-1.13,-.85,29.6,Action.ROLL_NEAR,4,30);
        }
        bonusIndex=stages.size();
    }
    public boolean basketTeam(){
        return lineup==Lineup.BASKET_ONE_TEAM||lineup==Lineup.BASKET_TWO_TEAM;
    }
    public boolean basketOne(){return lineup==Lineup.BASKET_ONE_TEAM;}
    public boolean partnerOpener(){
        return lineup==Lineup.PARTNER&&innerStart;
    }

    public boolean avoidFrame(){
        return lineup!=Lineup.LEGACY||strategy==Strategy.NO_FLOWERS;
    }

    private void basketTeamRoute(){
        boolean near=basketOne();
        int end=near?-1:1;
        Action fire=near?Action.ROLL_NEAR:Action.ROLL_FAR;
        String basket=near?"Basket 1":"Basket 2";
        add("Clear starting wall",near?-.36:-.90,1.43*end,-90*end,Action.MOVE,0,3);
        add("Open "+basket,near?-.36:-1.13,near?-1.43:.85,near?90:-29,fire,near?3:4,near?14:30);
        basketRecoveryIndex=stages.size();
        // View the full wall-end spill lane after the dumped pollen rolls away from the frame.
        add("Look for "+basket+" spill",-1.12,1.42*end,22*end,Action.MOVE,0,3);
        add("Recover local loose pollen with Limelight",-1.12,1.42*end,22*end,Action.VISION_COLLECT,4,8.5);
        basketFallbackIndex=stages.size();
        if(near){
            add("View local garden pollen",-1.10,-1.35,210,Action.MOVE,0,3);
            add("Collect local garden fallback",-1.50,-1.564259,180,Action.SWEEP,4,3);
        }else{
            add("Collect local pot fallback",-.5942,1.415,90,Action.FLOWER_COLLECT,4,4);
        }
        basketReloadIndex=stages.size();
        transit("Score recovered pollen in "+basket,near,4);
        bonusIndex=stages.size();
    }

    private void partner(double startX){
        add("Clear starting wall in assigned lane",startX,-1.43,90,Action.MOVE,0,2);
        if(partnerOpener()){
            // Keep the near board in the webcam view at the steep launch angle.
            add("Open near hive for partner",-.36,-1.43,90,Action.ROLL_NEAR,3,6);
            add("Return to inner handoff spot",-.36,-1.48,90,Action.MOVE,0,3);
        }else{
            add("Enter outside partner lane",-1.13,-1.33,90,Action.MOVE,0,2);
            // The far-cell tag interlock holds the feeder until the opener turns the hive.
            add("Wait for partner tip and score preloads",-1.13,.85,-29,Action.ROLL_FAR,4,12);
            add("Face assigned garden pollen",-1.10,-1.35,210,Action.MOVE,0,4);
            add("Collect assigned garden pollen",-1.50,-1.564259,180,Action.SWEEP,4,3);
            transit("Score garden pollen on far cell",false,4);
        }
        bonusIndex=stages.size();
    }

    private void noFlowers(){
        add("Clear starting wall",-.36,-1.43,90,Action.MOVE,0,2);
        transit("Shoot preloads toward garden",true,3);
        add("Face garden pollen",-1.10,-1.35,210,Action.MOVE,0,2);
        add("Collect first garden batch",-1.50,-1.564259,180,Action.SWEEP,3,3);
        transit("Score garden pollen while crossing",false,4);
        add("Return outside frame to garden",-1.40,-1.43,180,Action.MOVE,0,4);
        add("Garden cleanup without pots",-1.50,-1.575,180,Action.COLLECT,0,2);
        add("Stay outside near support",-1.13,-1.05,90,Action.MOVE,0,3);
        add("View settled pollen outside metal frame",-1.13,-.78,0,Action.MOVE,0,2);
        add("Recover settled first-tip pollen",-1.13,-.78,0,Action.VISION_COLLECT,3,4);
        transit("Complete far-cell tip toward parking",false,4);
        add("Scan far-end floor pollen",-1.05,.95,90,Action.MOVE,0,2);
        add("Recover far-end floor pollen",-1.05,.95,90,Action.VISION_COLLECT,2,2.5);
        transit("Score recovered floor pollen toward park",false,2);
        bonusIndex=stages.size();
    }
    private void add(String name,double x,double y,double h,Action a,int count,double limit) {
        if(a==Action.FIRE||a==Action.BONUS_FIRE){ x=-1.13; y=(stages.isEmpty()||a==Action.BONUS_FIRE)?-.85:.85; }
        stages.add(new Stage(name,pose(x,y,h),a,count,limit));
    }
    /** Collision-clear return corridor; points are generated from the current measured pose. */
    public List<Pose> parkingPath(Pose current) {
        if(avoidFrame())return FrameClearance.path(current,park,blue);
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
        if(basketTeam()||floorOnly())return 1.90+inches*.0254/1.20;
        return rushStyle()?1.15+inches*.0254/1.20:strategy==Strategy.ADAPTIVE_ALL?1.45+inches*.0254/1.20:1.05+inches*.0254/.85;
    }
    private Pose pose(double x,double y,double heading) {
        return alliancePose(blue, x, y, heading);
    }

    private static Pose alliancePose(boolean blue, double x, double y, double heading) {
        return new Pose((blue?-x:x)/.0254,(blue?-y:y)/.0254,Math.toRadians(heading+(blue?180:0)));
    }
}

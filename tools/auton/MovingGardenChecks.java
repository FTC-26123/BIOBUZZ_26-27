package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import java.util.List;

/** Production geometry, motion profile and ballistic solver; no Android hardware required. */
public final class MovingGardenChecks {
    private static int checks;
    private static void check(boolean ok,String why){checks++;if(!ok)throw new AssertionError(why);}
    private static double wrap(double a){return Math.atan2(Math.sin(a),Math.cos(a));}
    private static void close(double a,double b,double tolerance,String why){check(Math.abs(a-b)<tolerance,why);}
    private static CycleRoute route(boolean blue){return new CycleRoute(blue?CycleRoute.Start.BLUE_WEST:CycleRoute.Start.RED_EAST,
        CycleRoute.Strategy.NO_FLOWERS,CycleRoute.Lineup.GARDEN_MOVING);}
    public static void main(String[] args){
        close(MovingAim.turnRate(Math.toRadians(179),Math.toRadians(-179),.1),Math.toRadians(20),1e-8,"positive wrap rate");
        close(MovingAim.turnRate(Math.toRadians(-179),Math.toRadians(179),.1),Math.toRadians(-20),1e-8,"negative wrap rate");
        close(MovingAim.commandHeading(1,.4,.3),.92,1e-8,"release delay and follower lead");
        check(!MovingAim.trackable(MovingAim.turnRate(0,1,0)),"invalid derivative blocked");
        check(!MovingAim.trackable(3),"untrackable turn blocked");
        CycleRoute red=route(false),blue=route(true);
        for(boolean isBlue:new boolean[]{false,true}){
            CycleRoute r=isBlue?blue:red;Pose from=r.initial;
            check(r.stages.size()==6,"single preload/garden/opposite-hive cycle");
            for(int i=0;i<r.stages.size();i++){
                CycleRoute.Stage stage=r.stages.get(i);
                check(stage.action()!=CycleRoute.Action.FIRE&&stage.action()!=CycleRoute.Action.FLOWER_COLLECT
                    &&stage.action()!=CycleRoute.Action.VISION_COLLECT,"no stationary opening or detector pickup");
                List<Pose> path=FrameClearance.path(from,stage.pose(),isBlue);
                Pose previous=from;
                for(Pose next:path){
                    check(FrameClearance.segment(previous.x()*.0254,previous.y()*.0254,next.x()*.0254,next.y()*.0254),"route avoids frame");
                    for(int j=0;j<=100;j++){
                        double u=j/100.0,x=(previous.x()+(next.x()-previous.x())*u)*.0254,
                            y=(previous.y()+(next.y()-previous.y())*u)*.0254;
                        double heading=previous.heading()+wrap(next.heading()-previous.heading())*u;
                        double half=.2286*(Math.abs(Math.cos(heading))+Math.abs(Math.sin(heading)));
                        check(Math.abs(x)+half<1.793&&Math.abs(y)+half<1.793,"18-inch chassis clears walls");
                        double sign=isBlue?-1:1;
                        check(Math.abs(x+1.55*sign)>half+.2286||Math.abs(y-1.19*sign)>half+.2286,"reserved teammate footprint clear");
                        for(double[] pot:new double[][]{{-.5942,1.7283},{1.7283,.5942},{.5942,-1.7283},{-1.7283,-.5942}}){
                            double dx=pot[0]-x,dy=pot[1]-y,c=Math.cos(heading),s=Math.sin(heading);
                            check(Math.hypot(Math.max(0,Math.abs(dx*c+dy*s)-.2286),
                                Math.max(0,Math.abs(-dx*s+dy*c)-.2286))>.08,"fixed flower pot clearance");
                        }
                    }
                    previous=next;
                }
                from=stage.pose();
            }
            check(r.stages.get(1).action()==CycleRoute.Action.ROLL_NEAR&&r.stages.get(1).count()==4,"four moving preloads");
            check(r.stages.get(3).action()==CycleRoute.Action.GARDEN_STRAFE,"sideways garden pass");
            check(r.stages.get(5).action()==CycleRoute.Action.ROLL_FAR&&r.stages.get(5).count()==4,"four opposite-hive feeds");
            // The crossing has a sustained moving-shot window, rather than only a
            // reachable stationary destination. The actual feeder still validates each release.
            int farMovingSolutions=0;
            for(double y=-1.0;y<.55;y+=.05){
                double sign=isBlue?-1:1;
                MovingShotSolver.Solution far=MovingShotSolver.rolling(new Pose(-1.13*sign/.0254,y*sign/.0254,0),
                    new Velocity(0,.40*sign/.0254,0),isBlue,false,1900,0,0,0,0);
                if(far!=null)farMovingSolutions++;
            }
            check(farMovingSolutions>=28,"opposite hive reachable through moving crossing window");
            CycleRoute.Stage fire=r.stages.get(1);
            TransitPath transit=new TransitPath(r.stages.get(0).pose(),fire.pose(),isBlue,true,.15,.30);
            check(transit.duration()>5&&transit.duration()<7,"preload movement leaves time for four feeds");
            for(int i=2;i<50;i++){
                TransitPath.Sample p=transit.sample(transit.duration()*i/50);
                Pose pose=new Pose(p.x()/.0254,p.y()/.0254,0);
                Velocity v=new Velocity(p.vx()/.0254,p.vy()/.0254,0);
                MovingShotSolver.Solution shot=MovingShotSolver.rolling(pose,v,isBlue,true,1900,0,0,0,0);
                check(shot!=null,"moving preload ballistic solution along garden approach");
                double dt=MovingAim.SAMPLE_SECONDS;
                Pose later=new Pose(pose.x()+v.vx*dt,pose.y()+v.vy*dt,0);
                MovingShotSolver.Solution next=MovingShotSolver.rolling(later,v,isBlue,true,shot.ticks(),0,0,0,0);
                check(next!=null,"future moving aim available");
                double rate=MovingAim.turnRate(shot.heading(),next.heading(),dt);
                check(MovingAim.trackable(rate),"calculated turn is trackable");
                check(MovingShotSolver.evaluate(new Pose(pose.x(),pose.y(),shot.heading()),
                    new Velocity(v.vx,v.vy,rate),isBlue,true,shot.ticks(),0,0,0)!=null,"predicted shot enters opening while moving and turning");
            }
            // End-of-pass dwell must be continuous; leaving the pickup tolerance resets it.
            GardenStrafe strafe=new GardenStrafe();Pose start=r.stages.get(2).pose(),end=r.stages.get(3).pose();
            strafe.reset();strafe.target(start,end,0);
            Pose mid=strafe.target(start,end,.7);
            close(mid.y(),start.y(),1e-9,"sideways pass holds garden row");
            close(mid.heading(),end.heading(),1e-9,"pass holds intake heading");
            check(mid.x()!=start.x(),"pass translates sideways");
            strafe.target(end,end,2);check(!strafe.complete(2.4),"capture dwell required");
            strafe.target(start,end,2.4);check(!strafe.complete(3),"lost endpoint cancels capture dwell");
            strafe.target(end,end,3);check(strafe.complete(3.51),"continuous capture dwell completes");
            check(Double.isFinite(r.parkReserve(end))&&r.parkReserve(end)>0,"parking reserve exists");
            Pose parkedFrom=r.stages.get(5).pose();
            for(Pose waypoint:r.parkingPath(parkedFrom)){
                check(FrameClearance.segment(parkedFrom.x()*.0254,parkedFrom.y()*.0254,waypoint.x()*.0254,waypoint.y()*.0254),"parking segments clear frame");
                for(int i=0;i<=100;i++){
                    double u=i/100.0,x=(parkedFrom.x()+(waypoint.x()-parkedFrom.x())*u)*.0254,
                        y=(parkedFrom.y()+(waypoint.y()-parkedFrom.y())*u)*.0254,
                        h=parkedFrom.heading()+wrap(waypoint.heading()-parkedFrom.heading())*u,
                        half=.2286*(Math.abs(Math.cos(h))+Math.abs(Math.sin(h))),s=isBlue?-1:1;
                    check(Math.abs(x)+half<1.793&&Math.abs(y)+half<1.793,"parking wall clearance");
                    check(Math.abs(x+1.55*s)>half+.2286||Math.abs(y-1.19*s)>half+.2286,"parking avoids teammate slot");
                }
                parkedFrom=waypoint;
            }
        }
        for(int i=0;i<red.stages.size();i++){
            Pose a=red.stages.get(i).pose(),b=blue.stages.get(i).pose();
            close(a.x(),-b.x(),1e-8,"mirrored X");close(a.y(),-b.y(),1e-8,"mirrored Y");
            close(Math.abs(wrap(a.heading()-b.heading())),Math.PI,1e-8,"mirrored heading");
        }
        TransitPath empty=new TransitPath(red.initial,red.initial,false,true,.15,.30);
        close(empty.duration(),0,1e-9,"zero-length transit duration");
        close(empty.sample(1).remaining(),0,1e-9,"zero-length transit sample");
        System.out.println("PASS "+checks+" moving aim, ballistic entry, route clearance, strafe and mirror checks");
    }
}

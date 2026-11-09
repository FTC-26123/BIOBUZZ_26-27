package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Continuous Pedro legs for the eight selectable autons. Each leg is ONE Pedro path, so the robot never
 * stops at a corner: it only slows where the curve bends. Control points were designed with the Pedro
 * Visualizer engine (tools/auton/design-pedro-routes.mjs) and are exported as docs/pedro/*.pp.json.
 *
 * Every designed leg keeps the robot centre at least 2 inches outside {@link FrameClearance}, the
 * 18-inch footprint at least 4 inches from the fixed flower pot on every parking curve (the curves were pulled
 * inboard of the pot for tracking error; Basket 2's wall start itself sits 1.5 inches from its pot), 3 inches from
 * the reserved teammate slot, and the whole footprint on our alliance's half of the field.
 *
 * Constants are Red Pedro-visualizer inches (bottom-left origin); Blue is the 180-degree rotation.
 */
public final class AutoPaths {
    public static final double FIELD_CENTER=70.75;
    /** Shoot mark -> our parking slot, around the outside of the frame corner. */
    static final double[][] SHOT_TO_PARK={{43.81,15.98},{18.14,21.86}};
    /** Wall start -> our parking slot (Park Only). */
    static final double[][] START_TO_PARK={{41.75,21.24},{22.73,14.99}};
    /** Garden sweep end -> our parking slot, staying inboard of the fixed flower pot. */
    static final double[][] GARDEN_TO_PARK={{31.12,25.99},{19.22,84.40}};
    /** Basket 2 wall start -> its shooting mark: drop straight off the wall before drifting toward the
     * fixed flower pot side, keeping more than 1 inch from that pot. */
    static final double[][] WALL_TO_BASKET2={{61.10,121.00}};
    /** Shot -> garden approach finishes its 167-degree turn by 60% of the leg, before nearing the wall. */
    public static final double GARDEN_TURN_DONE=.6;
    /** A designed leg is reused from the actual pose only this close to its planned start. */
    public static final double REUSE_INCHES=4;

    private AutoPaths(){}

    public static final class Leg {
        public final String name;
        public final Pose start,end;
        private final Pose[] controls;
        public final double turnDoneAt;
        private Leg(String name,Pose start,Pose end,Pose[] controls,double turnDoneAt){
            this.name=name;this.start=start;this.end=end;this.controls=controls;this.turnDoneAt=turnDoneAt;
        }
        public static Leg line(String name,Pose start,Pose end){return new Leg(name,start,end,new Pose[0],1);}
        public static Leg line(String name,Pose start,Pose end,double turnDoneAt){return new Leg(name,start,end,new Pose[0],turnDoneAt);}
        /** The same leg, starting from where the robot actually is. */
        public Leg from(Pose actual){return new Leg(name,actual,end,controls,turnDoneAt);}
        public boolean curved(){return controls.length>0;}
        public List<Pose> controlPoints(){List<Pose> c=new ArrayList<>();Collections.addAll(c,controls);return c;}
        public Path path(){
            Path p;
            if(controls.length==0)p=Paths.line(start,end);
            else{
                Pose[] all=new Pose[controls.length+2];
                all[0]=start;System.arraycopy(controls,0,all,1,controls.length);all[all.length-1]=end;
                p=Paths.curve(all);
            }
            return turnDoneAt>=1?p.linear(start,end):p.linear(start,end,turnDoneAt);
        }
        private double[] point(double t){
            double[][] pts=new double[controls.length+2][];
            pts[0]=new double[]{start.x(),start.y()};
            for(int i=0;i<controls.length;i++)pts[i+1]=new double[]{controls[i].x(),controls[i].y()};
            pts[pts.length-1]=new double[]{end.x(),end.y()};
            for(int n=pts.length-1;n>0;n--)for(int i=0;i<n;i++)
                pts[i]=new double[]{pts[i][0]*(1-t)+pts[i+1][0]*t,pts[i][1]*(1-t)+pts[i+1][1]*t};
            return pts[0];
        }
        /** Poses along the leg, heading interpolated by distance travelled (as Pedro does). */
        public List<Pose> sample(int n){
            double[][] p=new double[n+1][];double[] s=new double[n+1];
            for(int i=0;i<=n;i++){p[i]=point(i/(double)n);if(i>0)s[i]=s[i-1]+Math.hypot(p[i][0]-p[i-1][0],p[i][1]-p[i-1][1]);}
            double turn=Math.atan2(Math.sin(end.heading()-start.heading()),Math.cos(end.heading()-start.heading()));
            List<Pose> out=new ArrayList<>();
            for(int i=0;i<=n;i++){
                double f=s[n]<=0?1:Math.min(1,s[i]/s[n]/turnDoneAt);
                out.add(new Pose(p[i][0],p[i][1],start.heading()+turn*f));
            }
            return out;
        }
        public double length(){
            List<Pose> s=sample(200);double total=0;
            for(int i=1;i<s.size();i++)total+=Math.hypot(s.get(i).x()-s.get(i-1).x(),s.get(i).y()-s.get(i-1).y());
            return total;
        }
        /** True when every sampled piece of the leg stays outside the runtime frame keepout. */
        public boolean clearOfFrame(){
            List<Pose> s=sample(200);
            for(int i=1;i<s.size();i++){
                Pose a=s.get(i-1),b=s.get(i);
                if(!FrameClearance.segment(a.x()*.0254,a.y()*.0254,b.x()*.0254,b.y()*.0254))return false;
            }
            return true;
        }
    }

    /** Red visualizer inches/degrees -> field-centred Pedro pose for the requested alliance. */
    public static Pose pose(boolean blue,double visualizerX,double visualizerY,double degrees){
        double s=blue?-1:1,h=Math.toRadians(degrees+(blue?180:0));
        return new Pose(s*(visualizerX-FIELD_CENTER),s*(visualizerY-FIELD_CENTER),Math.atan2(Math.sin(h),Math.cos(h)));
    }
    public static Leg curve(String name,boolean blue,Pose start,Pose end,double[][] redControls){
        Pose[] c=new Pose[redControls.length];
        for(int i=0;i<c.length;i++)c[i]=pose(blue,redControls[i][0],redControls[i][1],0);
        return new Leg(name,start,end,c,1);
    }
    /** Straight legs through a stop-and-go waypoint list (the conservative fallback corridor). */
    public static List<Leg> lines(String name,Pose from,List<Pose> waypoints){
        List<Leg> out=new ArrayList<>();
        for(Pose to:waypoints){out.add(Leg.line(name,from,to));from=to;}
        return out;
    }
    /**
     * The designed leg from the actual pose when the robot is near its planned start and the shifted curve
     * still clears the frame; otherwise the original FrameClearance corridor to the same end pose.
     */
    public static List<Leg> plan(Leg designed,Pose actual,boolean blue){
        return plan(designed,actual,FrameClearance.path(actual,designed.end,blue));
    }
    /** As {@link #plan(Leg,Pose,boolean)} with a caller-supplied fallback corridor from {@code actual}. */
    public static List<Leg> plan(Leg designed,Pose actual,List<Pose> fallback){
        Leg shifted=designed.from(actual);
        if(Math.hypot(actual.x()-designed.start.x(),actual.y()-designed.start.y())<=REUSE_INCHES&&shifted.clearOfFrame())
            return Collections.singletonList(shifted);
        return lines(designed.name+" (corridor fallback)",actual,fallback);
    }
    /** Seconds to drive a list of legs at a conservative 1.2 m/s, plus turning and settling. */
    public static double reserveSeconds(List<Leg> legs){
        double seconds=.8;
        for(Leg l:legs){
            double turn=Math.atan2(Math.sin(l.end.heading()-l.start.heading()),Math.cos(l.end.heading()-l.start.heading()));
            seconds+=l.length()*.0254/1.2+Math.abs(turn)/2+.5;
        }
        return seconds;
    }
}

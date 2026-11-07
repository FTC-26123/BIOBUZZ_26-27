package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.math.Pose;
import java.util.*;

/** Conservative whole-assembly keepout, including swept robot radius and tracking margin.
 * Field meters. This intentionally forbids driving between/under the metal supports. */
public final class FrameClearance {
    public static final double METAL_X=.67,METAL_Y=.52,ROBOT_RADIUS=.34,MARGIN=.03;
    public static final double X=METAL_X+ROBOT_RADIUS+MARGIN,Y=METAL_Y+ROBOT_RADIUS+MARGIN;
    private FrameClearance(){}
    public static boolean clear(double x,double y){return Double.isFinite(x+y)&&(Math.abs(x)>X||Math.abs(y)>Y);}
    public static boolean segment(double x,double y,double gx,double gy){
        if(!clear(x,y)||!clear(gx,gy))return false;
        double lo=0,hi=1;double[] p={x,y},v={gx-x,gy-y},extent={X,Y};
        for(int i=0;i<2;i++){
            if(Math.abs(v[i])<1e-10){if(Math.abs(p[i])>extent[i])return true;}
            else{double a=(-extent[i]-p[i])/v[i],b=(extent[i]-p[i])/v[i];lo=Math.max(lo,Math.min(a,b));hi=Math.min(hi,Math.max(a,b));}
        }
        return lo>hi;
    }
    /** Visibility graph around the alliance-side outer corners; never uses the center gap. */
    public static List<Pose> path(Pose from,Pose to,boolean blue){
        double turn=Math.atan2(Math.sin(to.heading()-from.heading()),Math.cos(to.heading()-from.heading()));
        if(Math.abs(turn)<1e-6)return bypass(from,to,blue);
        // Retreat without turning before changing heading next to a field wall.
        // Likewise finish the turn inboard before the final wall pickup approach.
        double limit=1.42;
        Pose innerFrom=new Pose(Math.max(-limit,Math.min(limit,from.x()*.0254))/.0254,
            Math.max(-limit,Math.min(limit,from.y()*.0254))/.0254,from.heading());
        Pose innerTo=new Pose(Math.max(-limit,Math.min(limit,to.x()*.0254))/.0254,
            Math.max(-limit,Math.min(limit,to.y()*.0254))/.0254,to.heading());
        boolean retreat=Math.hypot(innerFrom.x()-from.x(),innerFrom.y()-from.y())>1e-6;
        boolean approach=Math.hypot(innerTo.x()-to.x(),innerTo.y()-to.y())>1e-6;
        List<Pose> result=new ArrayList<>();
        if(retreat){
            if(!segment(from.x()*.0254,from.y()*.0254,innerFrom.x()*.0254,innerFrom.y()*.0254))
                throw new IllegalArgumentException("Wall retreat enters metal-frame keepout");
            result.add(innerFrom);
            result.add(new Pose(innerFrom.x(),innerFrom.y(),to.heading()));
        }
        result.addAll(bypass(retreat?innerFrom:from,approach?innerTo:to,blue));
        if(approach){
            if(!segment(innerTo.x()*.0254,innerTo.y()*.0254,to.x()*.0254,to.y()*.0254))
                throw new IllegalArgumentException("Wall approach enters metal-frame keepout");
            result.add(to);
        }
        return result;
    }
    private static List<Pose> bypass(Pose from,Pose to,boolean blue){
        double sign=blue?-1:1;
        double[][] points={{from.x()*.0254,from.y()*.0254},{to.x()*.0254,to.y()*.0254},{-1.13*sign,-1.05*sign},{-1.13*sign,1.05*sign}};
        if(!clear(points[0][0],points[0][1])||!clear(points[1][0],points[1][1]))throw new IllegalArgumentException("Target enters metal-frame keepout");
        double[] cost={0,Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY,Double.POSITIVE_INFINITY};int[] parent={-1,-1,-1,-1};boolean[] visited=new boolean[4];
        for(int pass=0;pass<4;pass++){
            int k=-1;for(int i=0;i<4;i++)if(!visited[i]&&(k<0||cost[i]<cost[k]))k=i;
            if(k<0||!Double.isFinite(cost[k]))break;visited[k]=true;
            for(int i=0;i<4;i++)if(!visited[i]&&segment(points[k][0],points[k][1],points[i][0],points[i][1])){
                double next=cost[k]+Math.hypot(points[k][0]-points[i][0],points[k][1]-points[i][1]);
                if(next<cost[i]){cost[i]=next;parent[i]=k;}
            }
        }
        if(!Double.isFinite(cost[1]))throw new IllegalArgumentException("No clear alliance-side frame bypass");
        LinkedList<Pose> result=new LinkedList<>();
        for(int i=1;i!=0;i=parent[i])result.addFirst(new Pose(points[i][0]/.0254,points[i][1]/.0254,to.heading()));
        return result;
    }
}

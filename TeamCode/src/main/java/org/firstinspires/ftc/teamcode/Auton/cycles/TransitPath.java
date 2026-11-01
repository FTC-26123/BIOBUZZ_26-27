package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.math.Pose;
import java.util.*;

/** One-way scoring transport to the next pickup. No repetitions or shooting-only detours. */
public final class TransitPath {
    public static final double SPEED=1.0,ACCEL=4.0;
    public static final class Sample {
        private final double x;
        private final double y;
        private final double vx;
        private final double vy;
        private final double remaining;
        public Sample(double x,double y,double vx,double vy,double remaining){this.x=x;this.y=y;this.vx=vx;this.vy=vy;this.remaining=remaining;}
        public double x(){return x;}
        public double y(){return y;}
        public double vx(){return vx;}
        public double vy(){return vy;}
        public double remaining(){return remaining;}
}
    private final List<double[]> points=new ArrayList<>();
    public final double length;
    public TransitPath(Pose start,Pose end,boolean blue){
        double sign=blue?-1:1,x=start.x()*.0254*sign,y=start.y()*.0254*sign;
        double ex=end.x()*.0254*sign,ey=end.y()*.0254*sign;
        points.add(new double[]{x*sign,y*sign});
        // Outside corridor bypasses the hive support on transfers between field ends.
        if(Math.abs(ey-y)>.8){
            if(Math.abs(x+1.13)>.06)points.add(new double[]{-1.13*sign,y*sign});
            points.add(new double[]{-1.13*sign,ey*sign});
        }
        points.add(new double[]{end.x()*.0254,end.y()*.0254});
        double total=0;for(int i=1;i<points.size();i++)total+=Math.hypot(points.get(i)[0]-points.get(i-1)[0],points.get(i)[1]-points.get(i-1)[1]);length=total;
    }
    public double duration(){double peak=Math.min(SPEED,Math.sqrt(length*ACCEL));return peak<=0?0:length/peak+peak/ACCEL;}
    public Sample sample(double seconds){
        double peak=Math.min(SPEED,Math.sqrt(length*ACCEL)),ta=peak/ACCEL,total=duration(),t=Math.max(0,Math.min(total,seconds)),d,v;
        if(t<ta){d=.5*ACCEL*t*t;v=ACCEL*t;}
        else if(t<total-ta){d=.5*ACCEL*ta*ta+peak*(t-ta);v=peak;}
        else {double left=total-t;d=length-.5*ACCEL*left*left;v=ACCEL*left;}
        d=Math.max(0,Math.min(length,d));double remaining=length-d;
        for(int i=1;i<points.size();i++){
            double[] a=points.get(i-1),b=points.get(i);double dx=b[0]-a[0],dy=b[1]-a[1],n=Math.hypot(dx,dy);
            if(n<1e-9)continue;
            if(d<=n||i==points.size()-1)return new Sample(a[0]+dx*d/n,a[1]+dy*d/n,dx*v/n,dy*v/n,remaining);
            d-=n;
        }
        double[] end=points.get(points.size()-1);return new Sample(end[0],end[1],0,0,0);
    }
}

package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

/** Independent onboard trajectory calculation using hardware calibration estimates and
 * odometry, not simulator state. Includes drag, backspin, muzzle offset, motion and feed delay. */
public final class MovingShotSolver {
    public static final class Solution {
        private final double ticks;
        private final double heading;
        private final double flightSeconds;
        private final double error;
        public Solution(double ticks,double heading,double flightSeconds,double error){this.ticks=ticks;this.heading=heading;this.flightSeconds=flightSeconds;this.error=error;}
        public double ticks(){return ticks;}
        public double heading(){return heading;}
        public double flightSeconds(){return flightSeconds;}
        public double error(){return error;}
}
    private static final class Impact {
        private final double u;
        private final double v;
        private final double time;
        public Impact(double u,double v,double time){this.u=u;this.v=v;this.time=time;}
        public double u(){return u;}
        public double v(){return v;}
        public double time(){return time;}
}
    /** Measured above robot-forward horizontal: 90 degrees points straight up. */
    public static final double LAUNCH_ANGLE_DEGREES=86;
    private static final double R=.03556,M=.026,ANGLE=Math.toRadians(LAUNCH_ANGLE_DEGREES),G=9.80665;
    private static final double AIM_V=-.065; // center low inside the rectangular portion, clear of the lip
    private MovingShotSolver() {}
    /** Coarse acquisition followed by a coupled heading/speed Newton solve. Unlike the
     * compatibility solver, centers lateral error as well as height for a moving lane. */
    public static Solution rolling(Pose pose,Velocity velocity,boolean blue,boolean initial,double seed){
        return rolling(pose,velocity,blue,initial,seed,.04,0,0);
    }
    public static Solution rolling(Pose pose,Velocity velocity,boolean blue,boolean initial,double seed,double delay,double ax,double ay){
        return rolling(pose,velocity,blue,initial,seed,delay,ax,ay,0);
    }
    public static Solution rolling(Pose pose,Velocity velocity,boolean blue,boolean initial,double seed,double delay,double ax,double ay,double aimU){
        double vx=velocity.vx*.0254,vy=velocity.vy*.0254,w=velocity.omega;
        if(!Double.isFinite(pose.x()+pose.y()+pose.heading()+vx+vy+w+seed+delay+ax+ay+aimU)||delay<0||delay>1.5||Math.hypot(vx,vy)>1.6||Math.abs(w)>2.5)return null;
        double x=pose.x()*.0254+vx*delay+.5*ax*delay*delay,y=pose.y()*.0254+vy*delay+.5*ay*delay*delay,hx=blue?.32385:-.32385;
        vx+=ax*delay;vy+=ay*delay;
        int side=(initial?-1:1)*(blue?-1:1);double bearing=Math.atan2(side*.392176-y,hx-x);
        double lateralVelocity=-vx*Math.sin(bearing)+vy*Math.cos(bearing);
        double initialHeading=bearing-Math.atan2(lateralVelocity,Math.max(800,seed)/28*Math.PI*.072*.75*Math.cos(ANGLE));
        Solution warm=refine(x,y,vx,vy,w,initialHeading,Math.max(800,Math.min(2240,seed)),side,hx,delay,aimU);
        if(warm!=null)return warm;
        double aimV=-.065,heading=bearing,ticks=seed,best=Double.POSITIVE_INFINITY;
        for(double candidate=800;candidate<=2240;candidate+=40){
            double lateral=-vx*Math.sin(bearing)+vy*Math.cos(bearing);
            // Seed the heading from the horizontal displacement left after chassis carry.
            // Near vertical, forward carry can require aiming behind the travel direction.
            double vz=candidate/28*Math.PI*.072*.75*Math.sin(ANGLE);
            double discriminant=vz*vz-2*G*(1.46-.2889397283);
            if(discriminant<0)continue;
            double flight=(vz+Math.sqrt(discriminant))/G;
            double aim=Math.atan2(side*.392176-y-vy*flight,hx+aimU-x-vx*flight);
            Impact hit=impact(x,y,vx,vy,w,aim,candidate,side,hx);
            if(hit==null)continue;double score=(hit.u-aimU)*(hit.u-aimU)+(hit.v-aimV)*(hit.v-aimV);
            if(score<best){best=score;heading=aim;ticks=candidate;}
        }
        return refine(x,y,vx,vy,w,heading,ticks,side,hx,delay,aimU);
    }
    private static Solution refine(double x,double y,double vx,double vy,double w,double heading,double ticks,int side,double hx,double delay,double aimU){
        double aimV=AIM_V;
        for(int k=0;k<9;k++){
            Impact hit=impact(x,y,vx,vy,w,heading,ticks,side,hx);if(hit==null)return null;
            hit=new Impact(hit.u-aimU,hit.v,hit.time);
            if(Math.abs(hit.u)<.002&&Math.abs(hit.v-aimV)<.002)return new Solution(ticks,heading-w*delay,hit.time,Math.hypot(hit.u,hit.v-aimV));
            Impact dh=impact(x,y,vx,vy,w,heading+.001,ticks,side,hx),dt=impact(x,y,vx,vy,w,heading,ticks+2,side,hx);if(dh==null||dt==null)return null;
            dh=new Impact(dh.u-aimU,dh.v,dh.time);dt=new Impact(dt.u-aimU,dt.v,dt.time);
            double a=(dh.u-hit.u)/.001,b=(dt.u-hit.u)/2,c=(dh.v-hit.v)/.001,d=(dt.v-hit.v)/2,det=a*d-b*c;
            if(Math.abs(det)<1e-9)return null;
            heading+=Math.max(-.10,Math.min(.10,(-hit.u*d+b*(hit.v-aimV))/det));
            ticks=Math.max(800,Math.min(2240,ticks+Math.max(-100,Math.min(100,(-a*(hit.v-aimV)+c*hit.u)/det))));
        }return null;
    }
    public static Solution evaluate(Pose pose,Velocity velocity,boolean blue,boolean initialCell,double ticks,double ax,double ay){
        return evaluate(pose,velocity,blue,initialCell,ticks,ax,ay,.04);
    }
    public static Solution evaluate(Pose pose,Velocity velocity,boolean blue,boolean initialCell,double ticks,double ax,double ay,double delay){
        double vx=velocity.vx*.0254,vy=velocity.vy*.0254,omega=velocity.omega;
        if(!Double.isFinite(pose.x()+pose.y()+pose.heading()+vx+vy+omega+ticks+ax+ay+delay)||delay<0||delay>1.5||Math.hypot(vx,vy)>1.6||Math.abs(omega)>2.5)return null;
        double px=pose.x()*.0254+vx*delay+.5*ax*delay*delay,py=pose.y()*.0254+vy*delay+.5*ay*delay*delay;
        double h=pose.heading()+omega*delay;int side=(initialCell?-1:1)*(blue?-1:1);
        Impact hit=impact(px,py,vx+ax*delay,vy+ay*delay,omega,h,ticks,side,blue?.32385:-.32385);
        if(hit==null)return null;
        double width=.254;if(hit.v>.61*.0254)width=.254*(7*.0254-hit.v)/(6.39*.0254);
        double margin=.012;
        if(Math.abs(hit.u)>width-R-margin||hit.v< -7*.0254+R+margin||hit.v>7*.0254-R-margin)return null;
        return new Solution(ticks,pose.heading(),hit.time,0);
    }
    /** Unknown intake order: permit only speeds that clear the opening for either ball type. */
    public static Solution evaluateMixed(Pose pose,Velocity velocity,boolean blue,boolean initial,double ticks,double ax,double ay){
        return evaluateMixed(pose,velocity,blue,initial,ticks,ax,ay,.04);
    }
    public static Solution evaluateMixed(Pose pose,Velocity velocity,boolean blue,boolean initial,double ticks,double ax,double ay,double delay){
        Solution pollen=evaluate(pose,velocity,blue,initial,ticks,ax,ay,delay);if(pollen==null)return null;
        double vx=velocity.vx*.0254,vy=velocity.vy*.0254,w=velocity.omega;
        Impact nectar=impactBall(pose.x()*.0254+vx*delay+.5*ax*delay*delay,pose.y()*.0254+vy*delay+.5*ay*delay*delay,vx+ax*delay,vy+ay*delay,w,pose.heading()+w*delay,ticks,(initial?-1:1)*(blue?-1:1),blue?.32385:-.32385,.04572,.045);
        if(nectar==null)return null;double width=nectar.v>.61*.0254?.254*(7*.0254-nectar.v)/(6.39*.0254):.254;
        return Math.abs(nectar.u)<width-.04572-.012&&nectar.v> -7*.0254+.04572+.012&&nectar.v<7*.0254-.04572-.012?pollen:null;
    }
    public static Solution solve(Pose pose,Velocity inchesPerSecond,boolean blue,boolean initialCell){
        return solve(pose,inchesPerSecond,blue,initialCell,.045);
    }
    public static Solution solve(Pose pose,Velocity inchesPerSecond,boolean blue,boolean initialCell,double delay){
        return rolling(pose,inchesPerSecond,blue,initialCell,1900,delay,0,0);
    }
    private static Impact impact(double x,double y,double vx,double vy,double omega,double h,double ticks,int side,double hx){
        return impactBall(x,y,vx,vy,omega,h,ticks,side,hx,R,M);
    }
    private static Impact impactBall(double x,double y,double vx,double vy,double omega,double h,double ticks,int side,double hx,double radius,double mass){
        double c=Math.cos(h),s=Math.sin(h),speed=ticks/28*Math.PI*.072*.75;
        double mx=.05911904015*c-.000001751818*s,my=.05911904015*s+.000001751818*c;
        double[] p={x+mx,y+my,.2889397283,c*speed*Math.cos(ANGLE)+vx-my*omega,s*speed*Math.cos(ANGLE)+vy+mx*omega,speed*Math.sin(ANGLE)};
        double[] spin={s*1000*Math.PI/30,-c*1000*Math.PI/30,0};
        double old=(p[1]-side*.392176)*side*Math.sqrt(3)/2+(p[2]-1.51257)*.5;
        // A steep shot may start behind the extended face plane, rise outside the
        // cell, then enter on its descent. Only inward crossings are candidates.
        double dt=1.0/240;
        for(int k=1;k<=960;k++){
            double[] before=p.clone();rk4(p,spin,dt,radius,mass);for(int j=0;j<3;j++)spin[j]*=Math.exp(-dt*.18);
            double next=(p[1]-side*.392176)*side*Math.sqrt(3)/2+(p[2]-1.51257)*.5;
            if(old>radius&&next<=radius){double a=(old-radius)/(old-next),ix=before[0]+a*(p[0]-before[0]),iy=before[1]+a*(p[1]-before[1]),iz=before[2]+a*(p[2]-before[2]);
                return new Impact(ix-hx,(iy-side*.392176)*(-side*.5)+(iz-1.51257)*Math.sqrt(3)/2,(k-1+a)*dt);}
            old=next;if(p[2]<radius)return null;
        }return null;
    }
    private static double[] derivative(double[] p,double[] spin,double radius,double mass){
        double vx=p[3],vy=p[4],vz=p[5],speed=Math.sqrt(vx*vx+vy*vy+vz*vz),q=.5*1.225*Math.PI*radius*radius/mass;
        double ax=-q*.47*speed*vx,ay=-q*.47*speed*vy,az=-G-q*.47*speed*vz;
        double w=Math.sqrt(spin[0]*spin[0]+spin[1]*spin[1]+spin[2]*spin[2]),cx=spin[1]*vz-spin[2]*vy,cy=spin[2]*vx-spin[0]*vz,cz=spin[0]*vy-spin[1]*vx,length=Math.sqrt(cx*cx+cy*cy+cz*cz);
        if(length>1e-9&&speed>1e-9){double lift=q*Math.min(.35,.16*w*radius/speed)*speed*speed/length;ax+=cx*lift;ay+=cy*lift;az+=cz*lift;}
        return new double[]{vx,vy,vz,ax,ay,az};
    }
    private static double[] add(double[] p,double[] d,double scale){double[] q=new double[6];for(int k=0;k<6;k++)q[k]=p[k]+d[k]*scale;return q;}
    private static void rk4(double[] p,double[] spin,double dt,double radius,double mass){double[] a=derivative(p,spin,radius,mass),b=derivative(add(p,a,dt/2),spin,radius,mass),c=derivative(add(p,b,dt/2),spin,radius,mass),d=derivative(add(p,c,dt),spin,radius,mass);for(int k=0;k<6;k++)p[k]+=dt/6*(a[k]+2*b[k]+2*c[k]+d[k]);}
}

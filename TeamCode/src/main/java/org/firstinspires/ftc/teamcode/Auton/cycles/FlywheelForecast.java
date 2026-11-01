package org.firstinspires.ftc.teamcode.Auton.cycles;

/** Estimated motor/velocity-loop forecast; measure on hardware in logical encoder units; no simulator reads. */
public final class FlywheelForecast {
    private FlywheelForecast(){}
    public static double after(double speed,double command,double seconds){
        if(!Double.isFinite(speed+command+seconds))return Double.NaN;
        for(double left=Math.max(0,seconds);left>1e-9;){double dt=Math.min(.005,left);left-=dt;
            double duty=Math.max(-1,Math.min(1,(command+2.5*(command-speed))/2800));
            double target=duty*2800;speed=target+(speed-target)*Math.exp(-dt/.52303);
        }return speed;
    }
    public static double command(double measured,double releaseSpeed,double delay){
        double decay=Math.exp(-Math.max(.025,delay)/.14944);
        return Math.max(0,Math.min(2240,(releaseSpeed-measured*decay)/(1-decay)));
    }
}

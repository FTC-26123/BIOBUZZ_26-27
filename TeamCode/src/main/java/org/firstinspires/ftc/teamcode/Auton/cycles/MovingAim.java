package org.firstinspires.ftc.teamcode.Auton.cycles;

/** Heading tracking in radians and seconds, including wrap at +/- pi. */
public final class MovingAim {
    public static final double SAMPLE_SECONDS=.05,MAX_RATE=2.5,HOLD_LEAD_SECONDS=.10;
    private MovingAim(){}
    public static double turnRate(double heading,double futureHeading,double dt){
        if(!Double.isFinite(heading+futureHeading+dt)||dt<=0)return Double.NaN;
        return Math.atan2(Math.sin(futureHeading-heading),Math.cos(futureHeading-heading))/dt;
    }
    /** The solver gives aim at release; command today's aim plus a short follower lead. */
    public static double commandHeading(double releaseHeading,double rate,double releaseDelay){
        return releaseHeading+rate*(HOLD_LEAD_SECONDS-releaseDelay);
    }
    public static boolean trackable(double rate){return Double.isFinite(rate)&&Math.abs(rate)<=MAX_RATE;}
}

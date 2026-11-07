package org.firstinspires.ftc.teamcode.Auton.cycles;

/** Onboard calibration, not simulator state. CAD path and estimated slip require real measurements. */
public final class FeederModel {
    public static final double PATH=.6285866147,DIAMETER=.072,REDUCTION=1,TRANSFER=.25,TICKS_PER_REV=28;
    // Estimated transfer startup latency; measure again with the current CRServo feed.
    public static final double POWER=.8,FREE_TICKS=2800,TAU=.1089607,START_DELAY=.035,MIN_INTERVAL=.35;
    private double travelTicks=PATH/(Math.PI*DIAMETER/REDUCTION*TRANSFER)*TICKS_PER_REV;
    private double progress,lastTime=-1,lastSpeed,opened=-1;
    private boolean learnable,paused;
    public double lastPrediction,lastObserved;
    public void begin(double now){begin(now,-10);}
    public void begin(double now,double lastShot){opened=now;progress=0;paused=false;learnable=now-lastShot>=MIN_INTERVAL;lastPrediction=remaining(now,lastSpeed,lastShot);}
    public void pause(){paused=true;learnable=false;}
    public void cancel(){opened=-1;progress=0;}
    public boolean active(){return opened>=0;}
    public void observe(double now,double ticks){
        ticks=Math.abs(ticks);
        if(lastTime>=0&&now>lastTime&&active()&&!paused){
            double dt=now-Math.max(lastTime,opened+START_DELAY);
            if(dt>0)progress+=(lastSpeed+ticks)*.5*dt;
        }
        lastTime=now;lastSpeed=ticks;
    }
    public void confirm(double now){
        lastObserved=now-opened;
        // Learn encoder travel at the flywheel load event; no inventory/ball truth is read.
        double nominal=PATH/(Math.PI*DIAMETER*TRANSFER)*TICKS_PER_REV;
        if(learnable&&progress>nominal*.5&&progress<nominal*3)travelTicks=.8*travelTicks+.2*progress;
        cancel();
    }
    public double remaining(double now,double ticks,double lastShot){
        if(!Double.isFinite(now+ticks+lastShot))return Double.NaN;
        double startDelay=active()?Math.max(0,opened+START_DELAY-now):START_DELAY;
        double current=Math.abs(ticks),target=POWER*FREE_TICKS;
        double atGate=target+(current-target)*Math.exp(-startDelay/TAU),need=Math.max(0,travelTicks-progress);
        double lo=0,hi=1.5;
        for(int i=0;i<35;i++){double t=(lo+hi)/2,travel=target*t+(atGate-target)*TAU*(1-Math.exp(-t/TAU));if(travel<need)lo=t;else hi=t;}
        return Math.max(startDelay+hi,Math.max(0,MIN_INTERVAL-(now-lastShot)));
    }
    public double timeout(){return Math.max(.55,Math.min(1.5,lastPrediction*1.7+.12));}
    public double progressTicks(){return progress;}
}

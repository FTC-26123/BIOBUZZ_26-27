package org.firstinspires.ftc.teamcode.Auton.cycles;

/** Estimated releases from encoder transport and a recovered flywheel load dip. */
public final class PreloadShotCounter {
    private static final double MIN_TRAVEL=FeederModel.PATH/(Math.PI*FeederModel.DIAMETER*FeederModel.TRANSFER)*FeederModel.TICKS_PER_REV*.70;
    private double lastTime=-1,travel,lastShot=-10;
    private boolean recovered;
    public int shots;
    /** Diagnostics for telemetry: encoder travel since the last release and whether the flywheel has recovered. */
    public double travel(){return travel;}
    public boolean recovered(){return recovered;}
    public void update(double now,double target,double speed,double intakeSpeed,boolean feeding){
        if(!Double.isFinite(now+target+speed+intakeSpeed)||target<=0){lastTime=-1;recovered=false;return;}
        if(lastTime>=0&&now>lastTime&&feeding)travel+=Math.abs(intakeSpeed)*Math.min(.1,now-lastTime);
        lastTime=now;
        if(speed>=target-40)recovered=true;
        if(feeding&&recovered&&travel>=MIN_TRAVEL&&speed<=target-Math.max(40,target*.05)&&now-lastShot>=.35){
            shots++;lastShot=now;travel=0;recovered=false;
        }
    }
}

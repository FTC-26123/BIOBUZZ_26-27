package org.firstinspires.ftc.teamcode.Auton.cycles;

/** Conservative encoder travel budget, not an inventory sensor. Measure constants on robot. */
public final class FlowerPickup {
    public static final double DRAW_METERS=.10, STACK_SETTLE_SECONDS=.13, FINAL_DWELL=.03, MIN_CONTACT=.18;
    private double last=-1,draw,settleUntil,ballStarted;private int extracted;
    public void reset(){last=-1;draw=0;extracted=0;settleUntil=0;ballStarted=-1;}
    public boolean update(double now,double intakeTicks,int wanted){
        if(!Double.isFinite(now+intakeTicks)||wanted<=0){reset();return false;}
        if(last<0){last=now;return false;}
        double dt=Math.max(0,Math.min(.1,now-last));last=now;
        if(extracted>=wanted)return now>=settleUntil+FINAL_DWELL;
        if(now<settleUntil)return false;
        if(ballStarted<0)ballStarted=now;
        draw+=Math.abs(intakeTicks)/FeederModel.TICKS_PER_REV*Math.PI*FeederModel.DIAMETER/FeederModel.REDUCTION*FeederModel.TRANSFER*dt;
        if(draw>=DRAW_METERS&&now-ballStarted>=MIN_CONTACT){draw=0;ballStarted=-1;extracted++;settleUntil=now+STACK_SETTLE_SECONDS;}
        return false;
    }
}

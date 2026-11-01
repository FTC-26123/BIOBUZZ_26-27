package org.firstinspires.ftc.teamcode.Auton.cycles;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="26123 Red Rush - pollen cycles",group="26123 Pollen cycles")
public final class RedRush extends ThirtySecondAuto {
    protected CycleRoute.Start startingPoint(){return CycleRoute.Start.RED_EAST;}
    protected CycleRoute.Strategy strategy(){return CycleRoute.Strategy.RUSH_TIPS;}
}

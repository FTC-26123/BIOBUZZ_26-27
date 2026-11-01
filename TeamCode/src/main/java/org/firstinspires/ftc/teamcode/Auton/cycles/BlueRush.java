package org.firstinspires.ftc.teamcode.Auton.cycles;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="26123 Blue Rush - pollen cycles",group="26123 Pollen cycles")
public final class BlueRush extends ThirtySecondAuto {
    protected CycleRoute.Start startingPoint(){return CycleRoute.Start.BLUE_WEST;}
    protected CycleRoute.Strategy strategy(){return CycleRoute.Strategy.RUSH_TIPS;}
}

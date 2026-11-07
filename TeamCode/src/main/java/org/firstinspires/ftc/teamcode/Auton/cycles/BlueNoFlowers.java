package org.firstinspires.ftc.teamcode.Auton.cycles;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@com.qualcomm.robotcore.eventloop.opmode.Disabled
@Autonomous(name="26123 Blue No Flowers",group="26123 Pollen cycles")
public final class BlueNoFlowers extends ThirtySecondAuto {
    protected CycleRoute.Start startingPoint(){return CycleRoute.Start.BLUE_WEST;}
    protected CycleRoute.Strategy strategy(){return CycleRoute.Strategy.NO_FLOWERS;}
}

package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.Auton.cycles.CycleRoute;
import org.firstinspires.ftc.teamcode.Auton.cycles.ThirtySecondAuto;

@Autonomous(name="Blue Moving Shots + Garden + Hive",group="26123 Flowerless")
@Disabled // Hidden: replaced by BioBuzzAuto. Delete this line to show this program on the Driver Station again.
public final class BlueGardenMovingShots extends ThirtySecondAuto {
    @Override protected CycleRoute.Start startingPoint(){return CycleRoute.Start.BLUE_WEST;}
    @Override protected CycleRoute.Strategy strategy(){return CycleRoute.Strategy.NO_FLOWERS;}
    @Override protected CycleRoute.Lineup lineup(){return CycleRoute.Lineup.GARDEN_MOVING;}
}

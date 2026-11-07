package org.firstinspires.ftc.teamcode.Auton;

import org.firstinspires.ftc.teamcode.Auton.cycles.CycleRoute;
import org.firstinspires.ftc.teamcode.Auton.cycles.ThirtySecondAuto;


public final class BlueBasket1 extends ThirtySecondAuto {
    @Override protected CycleRoute.Start startingPoint() { return CycleRoute.Start.BLUE_WEST; }
    @Override protected CycleRoute.Strategy strategy() { return CycleRoute.Strategy.RUSH_TIPS; }
    @Override protected CycleRoute.Lineup lineup() { return CycleRoute.Lineup.SOLO; }
}

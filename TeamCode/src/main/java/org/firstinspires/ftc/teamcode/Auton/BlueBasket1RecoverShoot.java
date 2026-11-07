package org.firstinspires.ftc.teamcode.Auton;

import org.firstinspires.ftc.teamcode.Auton.cycles.CycleRoute;
import org.firstinspires.ftc.teamcode.Auton.cycles.ThirtySecondAuto;

public final class BlueBasket1RecoverShoot extends ThirtySecondAuto {
    @Override protected CycleRoute.Start startingPoint() { return CycleRoute.Start.BLUE_WEST; }
    @Override protected CycleRoute.Strategy strategy() { return CycleRoute.Strategy.NO_FLOWERS; }
    @Override protected CycleRoute.Lineup lineup() { return CycleRoute.Lineup.FOUR_RECOVER_TEAM; }
}

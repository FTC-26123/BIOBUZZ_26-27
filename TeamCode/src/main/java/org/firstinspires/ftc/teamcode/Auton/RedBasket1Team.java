package org.firstinspires.ftc.teamcode.Auton;

import org.firstinspires.ftc.teamcode.Auton.cycles.CycleRoute;
import org.firstinspires.ftc.teamcode.Auton.cycles.ThirtySecondAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Basket 1 Team", group = "26123 Match Lineup")
public final class RedBasket1Team extends ThirtySecondAuto {
    @Override protected CycleRoute.Start startingPoint() { return CycleRoute.Start.RED_EAST; }
    @Override protected CycleRoute.Strategy strategy() { return CycleRoute.Strategy.RUSH_TIPS; }
    @Override protected CycleRoute.Lineup lineup() { return CycleRoute.Lineup.BASKET_ONE_TEAM; }
}

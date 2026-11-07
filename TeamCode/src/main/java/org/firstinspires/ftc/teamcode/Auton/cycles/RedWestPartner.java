package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@com.qualcomm.robotcore.eventloop.opmode.Disabled
@Autonomous(name = "26123 Red West Partner (Scorer)", group = "26123 Match Lineup")
public final class RedWestPartner extends ThirtySecondAuto {
    @Override protected CycleRoute.Start startingPoint() { return CycleRoute.Start.RED_WEST; }
    @Override protected CycleRoute.Strategy strategy() { return CycleRoute.Strategy.RUSH_TIPS; }
    @Override protected CycleRoute.Lineup lineup() { return CycleRoute.Lineup.PARTNER; }
}

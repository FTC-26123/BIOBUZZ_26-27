package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@com.qualcomm.robotcore.eventloop.opmode.Disabled
@Autonomous(name = "26123 Blue West Partner (Opener)", group = "26123 Match Lineup")
public final class BlueWestPartner extends ThirtySecondAuto {
    @Override protected CycleRoute.Start startingPoint() { return CycleRoute.Start.BLUE_WEST; }
    @Override protected CycleRoute.Strategy strategy() { return CycleRoute.Strategy.RUSH_TIPS; }
    @Override protected CycleRoute.Lineup lineup() { return CycleRoute.Lineup.PARTNER; }
}

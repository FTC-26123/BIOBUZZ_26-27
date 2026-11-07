package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@com.qualcomm.robotcore.eventloop.opmode.Disabled
@Autonomous(name = "26123 Blue East Solo", group = "26123 Match Lineup")
public final class BlueEastSolo extends ThirtySecondAuto {
    @Override protected CycleRoute.Start startingPoint() { return CycleRoute.Start.BLUE_EAST; }
    @Override protected CycleRoute.Strategy strategy() { return CycleRoute.Strategy.RUSH_TIPS; }
    @Override protected CycleRoute.Lineup lineup() { return CycleRoute.Lineup.SOLO; }
}

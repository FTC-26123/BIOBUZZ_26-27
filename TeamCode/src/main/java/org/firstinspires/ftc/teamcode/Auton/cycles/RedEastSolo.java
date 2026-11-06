package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "26123 Red East Solo", group = "26123 Match Lineup")
public final class RedEastSolo extends ThirtySecondAuto {
    @Override protected CycleRoute.Start startingPoint() { return CycleRoute.Start.RED_EAST; }
    @Override protected CycleRoute.Strategy strategy() { return CycleRoute.Strategy.RUSH_TIPS; }
    @Override protected CycleRoute.Lineup lineup() { return CycleRoute.Lineup.SOLO; }
}

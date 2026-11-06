package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "26123 Red East Park Only", group = "26123 Match Lineup")
public final class RedEastParkOnly extends ParkOnlyAuto {
    @Override protected CycleRoute.Start startingPoint() { return CycleRoute.Start.RED_EAST; }
}

package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@com.qualcomm.robotcore.eventloop.opmode.Disabled
@Autonomous(name = "26123 Blue West Park Only", group = "26123 Match Lineup")
public final class BlueWestParkOnly extends ParkOnlyAuto {
    @Override protected CycleRoute.Start startingPoint() { return CycleRoute.Start.BLUE_WEST; }
}

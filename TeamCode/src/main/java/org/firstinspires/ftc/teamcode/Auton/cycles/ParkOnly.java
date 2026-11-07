package org.firstinspires.ftc.teamcode.Auton.cycles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Park Only", group = "26123 Match Lineup")
public final class ParkOnly extends ParkOnlyAuto {
    private boolean blue;

    @Override protected CycleRoute.Start startingPoint() {
        return blue ? CycleRoute.Start.BLUE_WEST : CycleRoute.Start.RED_EAST;
    }

    @Override public void init_loop() {
        boolean selectedBlue = blue;
        if (gamepad1.x) selectedBlue = true;
        if (gamepad1.b) selectedBlue = false;
        if (selectedBlue != blue) {
            blue = selectedBlue;
            prepareStart();
        }
        telemetry.addData("Park alliance", blue ? "Blue" : "Red");
        telemetry.addLine("Before Start, X selects Blue and B selects Red. Place at the matching Basket 1 mark.");
        telemetry.update();
    }
}

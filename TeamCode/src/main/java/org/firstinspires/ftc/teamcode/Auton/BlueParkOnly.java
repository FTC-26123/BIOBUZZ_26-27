package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.Auton.cycles.CycleRoute;
import org.firstinspires.ftc.teamcode.Auton.cycles.ParkOnlyAuto;

@Autonomous(name = "Blue Park Only", group = "26123 Flowerless")
@Disabled // Hidden: replaced by BioBuzzAuto. Delete this line to show this program on the Driver Station again.
public final class BlueParkOnly extends ParkOnlyAuto {
    @Override protected CycleRoute.Start startingPoint() { return CycleRoute.Start.BLUE_WEST; }
}

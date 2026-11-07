package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Auton.cycles.CycleRoute;
import org.firstinspires.ftc.teamcode.Auton.cycles.FixedShotParkAuto;

@Autonomous(name = "Red Basket 1 Shoot 4 and Park", group = "26123 Flowerless")
public final class RedBasket1Shoot4Park extends FixedShotParkAuto {
    @Override protected CycleRoute.Start startingPoint() { return CycleRoute.Start.RED_EAST; }
}

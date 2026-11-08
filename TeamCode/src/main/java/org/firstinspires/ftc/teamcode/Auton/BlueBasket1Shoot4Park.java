package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Auton.cycles.CycleRoute;
import org.firstinspires.ftc.teamcode.Auton.cycles.ShootFourAndParkAuto;

@Autonomous(name = "Blue Basket 1 Shoot 4 and Park", group = "26123 Flowerless")
public final class BlueBasket1Shoot4Park extends ShootFourAndParkAuto {
    @Override protected CycleRoute.Start startingPoint() { return CycleRoute.Start.BLUE_WEST; }
}

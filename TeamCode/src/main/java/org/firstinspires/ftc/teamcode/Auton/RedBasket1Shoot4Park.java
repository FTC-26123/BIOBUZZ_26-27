package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.Auton.cycles.CycleRoute;
import org.firstinspires.ftc.teamcode.Auton.cycles.ShootFourAndParkAuto;

@Autonomous(name = "Red Basket 1 Shoot 4 and Park", group = "26123 Flowerless")
@Disabled // Hidden: replaced by BioBuzzAuto. Delete this line to show this program on the Driver Station again.
public final class RedBasket1Shoot4Park extends ShootFourAndParkAuto {
    @Override protected CycleRoute.Start startingPoint() { return CycleRoute.Start.RED_EAST; }
}

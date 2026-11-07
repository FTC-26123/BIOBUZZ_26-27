package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Auton.cycles.DelayedBasketTwoAuto;

@Autonomous(name="Blue Basket 2 Wait 15 Shoot 4 and Park",group="26123 Flowerless")
public final class BlueBasket2Wait15Shoot4Park extends DelayedBasketTwoAuto {
    @Override protected boolean blueAlliance(){return true;}
}

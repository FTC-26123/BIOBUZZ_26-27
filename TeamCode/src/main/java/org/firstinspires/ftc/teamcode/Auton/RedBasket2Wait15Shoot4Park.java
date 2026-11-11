package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.Auton.cycles.DelayedBasketTwoAuto;

@Autonomous(name="Red Basket 2 Wait 15 Shoot 4 and Park",group="26123 Flowerless")
@Disabled // Hidden: replaced by BioBuzzAuto. Delete this line to show this program on the Driver Station again.
public final class RedBasket2Wait15Shoot4Park extends DelayedBasketTwoAuto {
    @Override protected boolean blueAlliance(){return false;}
}

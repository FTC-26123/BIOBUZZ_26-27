package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.Auton.cycles.GardenParkAuto;

@Autonomous(name="Blue Shoot 4 + Garden and Park",group="26123 Flowerless")
@Disabled // Hidden: replaced by BioBuzzAuto. Delete this line to show this program on the Driver Station again.
public final class BlueShoot4GardenPark extends GardenParkAuto {
    @Override protected boolean blueAlliance(){return true;}
}

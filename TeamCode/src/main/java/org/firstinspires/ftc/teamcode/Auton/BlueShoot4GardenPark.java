package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Auton.cycles.GardenParkAuto;

@Autonomous(name="Blue Shoot 4 + Garden and Park",group="26123 Flowerless")
public final class BlueShoot4GardenPark extends GardenParkAuto {
    @Override protected boolean blueAlliance(){return true;}
}

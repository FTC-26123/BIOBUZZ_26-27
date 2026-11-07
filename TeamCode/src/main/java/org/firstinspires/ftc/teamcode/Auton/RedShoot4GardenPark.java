package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Auton.cycles.GardenParkAuto;

@Autonomous(name="Red Shoot 4 + Garden and Park",group="26123 Flowerless")
public final class RedShoot4GardenPark extends GardenParkAuto {
    @Override protected boolean blueAlliance(){return false;}
}

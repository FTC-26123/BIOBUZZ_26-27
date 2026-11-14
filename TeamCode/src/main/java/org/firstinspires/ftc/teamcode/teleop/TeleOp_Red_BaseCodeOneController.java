package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** RED alliance version of {@link TeleOp_BaseCodeOneController}. */
@TeleOp(name = "Red TeleOp One Controller", group = "26123 Red")
public class TeleOp_Red_BaseCodeOneController extends TeleOp_BaseCodeOneController {
    @Override protected boolean blueAlliance() { return false; }
}

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** RED alliance version of {@link TeleOp_BaseCode}. */
@TeleOp(name = "Red TeleOp", group = "26123 Red")
public class TeleOp_Red_BaseCode extends TeleOp_BaseCode {
    @Override protected boolean blueAlliance() { return false; }
}

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** RED alliance version of {@link TeleOp_BaseCodeSpeedAdjust}. */
@TeleOp(name = "Red TeleOp Speed Adjust", group = "26123 Red")
public class TeleOp_Red_BaseCodeSpeedAdjust extends TeleOp_BaseCodeSpeedAdjust {
    @Override protected boolean blueAlliance() { return false; }
}

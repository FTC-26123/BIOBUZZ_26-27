package org.firstinspires.ftc.teamcode.pedro;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeftMotor");
        c.frontRightName.set("frontRightMotor");
        c.backLeftName.set("backLeftMotor");
        c.backRightName.set("backRightMotor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });


    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-5.161768995870755);
        c.yPodOffset.set(-3.5600169624869284);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    /** Team-provided hardware tuning; simulator cycles use their separate SimPedro model. */
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.30024079620465516);
        Controller secondaryTranslationalForward = Controller.proportional(0.11093095161579151);
        Controller primaryTranslationalLateral = Controller.proportional(0.4587871728667344);
        Controller secondaryTranslationalLateral = Controller.proportional(0.1695096013552218);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(primaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
        c.coast.set(Controller.proportionalFeedforward(0.013344395361087693));
        c.brake.set(Controller.proportionalFeedforward(0.011342736056924539));
        c.headingFeedback.set(Controller.proportional(4.993589337321498));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05153506843112024, 0.0041538205190538215));
        c.linearBrakeCoefficients.set(Matrix.diag(0.08004254048686771, 0.060005110933381595));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014483123800480356, 0.001590024294588757));
        c.maxAchievableForwardVelocity.set(74.57640736619945);
        c.maxAchievableStrafeVelocity.set(60.78246577619209);
        c.naturalForwardDeceleration.set(29.473446284393383);
        c.naturalStrafeDeceleration.set(58.541903035672625);
    });

    public static Follower create(HardwareMap h) {
        return new Follower(new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig), new Foresight(foresightConfig));
    }
}
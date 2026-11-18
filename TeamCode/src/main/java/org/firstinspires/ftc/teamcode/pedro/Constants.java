package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.constants.RobotSpecs;

public class Constants {
    public static Follower create(HardwareMap h) {
         return new Follower(drivetrainConfig, localizerConfig, foresightConfig);
        return null;
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odometry");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        c.xPodOffset.set(-1.7925025158979762);
        c.yPodOffset.set(0.4967154855803242);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.17388225600632276);
                Controller secondaryTranslationalForward = Controller.proportional(0.06424484737488512);
                Controller primaryTranslationalLateral = Controller.proportional(0.21460845765608474);
                Controller secondaryTranslationalLateral = Controller.proportional(0.07929209066032199);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.009868626467638782));
                c.brake.set(Controller.proportionalFeedforward(0.008388332497492965));

                c.headingFeedback.set(Controller.proportional(2.7062516240633965));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04126591492796379, 0.010109557759567792));

                c.linearBrakeCoefficients.set(Matrix.diag(0.05966608670870589, 0.03154404324027167));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.00214655067234575, 0.003273038239461142));

                c.maxAchievableForwardVelocity.set(95.12400765555927);
                c.maxAchievableStrafeVelocity.set(0.8662977050742882);
                c.naturalForwardDeceleration.set(28.820031447127796);
                c.naturalStrafeDeceleration.set(31.901595240042894);
            }
    );
}
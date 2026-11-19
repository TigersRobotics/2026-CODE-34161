package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.PathGenV3;
import org.firstinspires.ftc.teamcode.robotControllers.MecanumDriveTrainController;
import org.firstinspires.ftc.teamcode.sensors.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;
import org.firstinspires.ftc.teamcode.utils.VectorMath;

@TeleOp (name = "PedroPathing Test")
public class PedroTest extends OpMode {

    GoBildaOdometry odometry;
    Follower follower;
    @Override
    public void init() {
//        PathGenV2.init(odometry, );
        Timer.init();
        TimedTelemetry.init(telemetry);
        odometry = new GoBildaOdometry(hardwareMap, HardwareNames.ODOMETRY_NAME);
        MecanumDriveTrainController.init(hardwareMap);
        follower = Constants.create(hardwareMap);
        PathGenV3.init(odometry, follower, true);
        TimedTelemetry.update();
        telemetry.update();
    }

    @Override
    public void loop() {
        Timer.update();
        odometry.update();
        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;
        MecanumDriveTrainController.update(VectorMath.rotateVector(x, y, -odometry.getHeading(AngleUnit.DEGREES), false)[1], VectorMath.rotateVector(x, y, -odometry.getHeading(AngleUnit.DEGREES), false)[0], turn);
        TimedTelemetry.addData("X: ", odometry.getX(DistanceUnit.INCH));
        TimedTelemetry.addData("Y: ", odometry.getY(DistanceUnit.INCH));
        TimedTelemetry.addData("Heading: ", odometry.getHeading(AngleUnit.DEGREES));
        if (gamepad1.circleWasPressed()) {
            PathGenV3.goPark();
        }
        PathGenV3.update();
        TimedTelemetry.update();
        telemetry.update();

    }
}

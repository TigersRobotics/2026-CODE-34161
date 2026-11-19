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

import java.nio.file.Path;

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
        MecanumDriveTrainController.update(-y, -x, turn);
        TimedTelemetry.addData("X: ", odometry.getX(DistanceUnit.INCH));
        TimedTelemetry.addData("Y: ", odometry.getY(DistanceUnit.INCH));
        TimedTelemetry.addData("Heading: ", odometry.getHeading(AngleUnit.DEGREES));
        if (gamepad1.circleWasPressed()) {
            PathGenV3.goPark();
        } else if (gamepad1.dpadLeftWasPressed()) {
            PathGenV3.goShoot(PathGenV3.Hive.LEFT_HIVE);
        } else if (gamepad1.dpadRightWasPressed()) {
            PathGenV3.goShoot(PathGenV3.Hive.RIGHT_HIVE);
        } else if (gamepad1.rightBumperWasPressed()) {
            PathGenV3.switchOn();
        } else if (gamepad1.dpadUpWasPressed()) {
            PathGenV3.goShoot(PathGenV3.Hive.NEAREST);
        } else if (gamepad1.dpadDownWasPressed()) {
            PathGenV3.goScoop();
        }


        PathGenV3.update(gamepad1.right_trigger > 0.5);
        TimedTelemetry.update();
        telemetry.update();

    }
}

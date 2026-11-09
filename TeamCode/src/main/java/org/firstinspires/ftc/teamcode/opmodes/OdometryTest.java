package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.robotControllers.MecanumDriveTrainController;
import org.firstinspires.ftc.teamcode.sensors.OdometryV2;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;

@TeleOp (name = "Odometry Test")
public class OdometryTest extends OpMode {
    OdometryV2 odometry = new OdometryV2(hardwareMap, HardwareNames.ODOMETRY_NAME);
    @Override
    public void init() {
        TimedTelemetry.init(telemetry);
//        MecanumDriveTrainController.init(hardwareMap,telemetry, true);
    }

    @Override
    public void loop() {
//        MecanumDriveTrainController.update(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        TimedTelemetry.addData("X: ", odometry.getX(DistanceUnit.INCH));
        TimedTelemetry.addData("Y: ", odometry.getY(DistanceUnit.INCH));
        TimedTelemetry.addData("Heading: ", odometry.getHeading(AngleUnit.DEGREES));


    }
}

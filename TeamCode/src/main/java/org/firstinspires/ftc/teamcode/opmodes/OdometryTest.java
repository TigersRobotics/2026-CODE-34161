package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.robotControllers.MecanumDriveTrainController;
import org.firstinspires.ftc.teamcode.sensors.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;
import org.firstinspires.ftc.teamcode.utils.VectorMath;

@TeleOp (name = "Odometry Test")
public class OdometryTest extends OpMode {
    GoBildaOdometry odometry;
    @Override
    public void init() {

        Timer.init();
        TimedTelemetry.init(telemetry);
        odometry = new GoBildaOdometry(hardwareMap, HardwareNames.ODOMETRY_NAME);
        MecanumDriveTrainController.init(hardwareMap, true);
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
        TimedTelemetry.update();
        telemetry.update();

    }
}

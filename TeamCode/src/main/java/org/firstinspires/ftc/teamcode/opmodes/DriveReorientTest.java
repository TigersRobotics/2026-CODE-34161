package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.robotControllers.MecanumDriveTrainController;
import org.firstinspires.ftc.teamcode.sensors.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;
import org.firstinspires.ftc.teamcode.utils.VectorMath;

@Disabled
@Deprecated
@TeleOp(name = "Driving Test With Reorientation")
public class DriveReorientTest extends OpMode {
    GoBildaOdometry odometry;

    @Override
    public void init() {
        Timer.init();
        TimedTelemetry.init(telemetry);
        MecanumDriveTrainController.init(hardwareMap);
        odometry = new GoBildaOdometry(hardwareMap, HardwareNames.ODOMETRY_NAME);

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
        TimedTelemetry.addData("Heading", odometry.getHeading(AngleUnit.DEGREES));
        TimedTelemetry.addData("Odometry Status", odometry.getStatus());
        TimedTelemetry.addData("Position", odometry.getPose());
        MecanumDriveTrainController.update(VectorMath.rotateVector(x, y, -odometry.getHeading(AngleUnit.DEGREES), false)[1], VectorMath.rotateVector(x, y, -odometry.getHeading(AngleUnit.DEGREES), false)[0], turn);

        TimedTelemetry.update();
        telemetry.update();
    }
}

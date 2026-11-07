package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotControllers.MecanumDriveTrainController;
import org.firstinspires.ftc.teamcode.robotControllers.PollenTurretController;
import org.firstinspires.ftc.teamcode.robotControllers.SingleIntakeController;
import org.firstinspires.ftc.teamcode.robotControllers.UptakeServoController;
import org.firstinspires.ftc.teamcode.sensors.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.utils.VectorMath;

@TeleOp
public class TeleOpMain extends OpMode {
    final boolean ENABLE_TURRET_CONTROLLER = false;
    final boolean ENABLE_DRIVE_CONTROLLER = true;
    final boolean ENABLE_UPTAKE_CONTROLLER = true;
    final boolean ENABLE_INTAKE_CONTROLLER = true;
    AprilTagWebcam webcam;

    @Override
    public void init() {
        webcam = new AprilTagWebcam(hardwareMap);
        MecanumDriveTrainController.init(hardwareMap, telemetry, ENABLE_DRIVE_CONTROLLER);
        PollenTurretController.init(hardwareMap, ENABLE_TURRET_CONTROLLER);
        UptakeServoController.init(hardwareMap, telemetry, ENABLE_UPTAKE_CONTROLLER);
        SingleIntakeController.init(hardwareMap, telemetry, ENABLE_INTAKE_CONTROLLER);
    }

    @Override
    public void loop() {
        webcam.updateTagDetection();
        MecanumDriveTrainController.update(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        PollenTurretController.update();
        UptakeServoController.update();
        SingleIntakeController.update(gamepad1.left_trigger);

        double angles[] = VectorMath.shotAngle(webcam.getTagByID(21).ftcPose.range, webcam.getTagByID(21).ftcPose.elevation, 0, webcam.getTagByID(21).ftcPose.bearing, telemetry);
    }

    @Override
    public void stop() {
        MecanumDriveTrainController.stop();
        PollenTurretController.stop();
        UptakeServoController.stop();
        SingleIntakeController.stop();
    }
}

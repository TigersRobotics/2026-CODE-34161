package org.firstinspires.ftc.teamcode.blue.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blue.drive.SwerveDrive;
import org.firstinspires.ftc.teamcode.blue.drive.SwerveModule;
import org.firstinspires.ftc.teamcode.blue.util.BlueConstants;
import org.firstinspires.ftc.teamcode.blue.util.DataLogger;

@TeleOp(name = "Swerve Drive Test", group = "Blue Test")
public class SwerveDriveTest extends OpMode {
    SwerveDrive drive;
    DataLogger logger;
    ElapsedTime timer = new ElapsedTime();
    boolean fieldCentric = false;

    @Override
    public void init() {
        drive = new SwerveDrive(hardwareMap);
        logger = new DataLogger("swerve_drive", "time", "heading",
                "flTarget", "flAngle", "frTarget", "frAngle", "blTarget", "blAngle", "brTarget", "brAngle");
    }

    @Override
    public void start() {
        timer.reset();
        drive.resetHeading();
    }

    @Override
    public void loop() {
        if (gamepad1.optionsWasPressed()) drive.resetHeading();
        if (gamepad1.shareWasPressed()) fieldCentric = !fieldCentric;

        if (gamepad1.dpadUpWasPressed()) BlueConstants.TURN_P += 0.05;
        if (gamepad1.dpadDownWasPressed()) BlueConstants.TURN_P = Math.max(0, BlueConstants.TURN_P - 0.05);
        if (gamepad1.dpadRightWasPressed()) BlueConstants.TURN_D += 0.005;
        if (gamepad1.dpadLeftWasPressed()) BlueConstants.TURN_D = Math.max(0, BlueConstants.TURN_D - 0.005);

        if (gamepad1.x) {
            drive.lock();
        } else {
            drive.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, fieldCentric);
        }

        logger.log(String.format("%.3f", timer.seconds()), String.format("%.2f", drive.getHeadingDegrees()),
                deg(drive.fl.getTargetAngle()), deg(drive.fl.getAngle()),
                deg(drive.fr.getTargetAngle()), deg(drive.fr.getAngle()),
                deg(drive.bl.getTargetAngle()), deg(drive.bl.getAngle()),
                deg(drive.br.getTargetAngle()), deg(drive.br.getAngle()));

        telemetry.addLine("dpad U/D = turn P, dpad L/R = turn D, X = lock");
        telemetry.addData("Field centric", fieldCentric);
        telemetry.addData("Heading", "%.1f", drive.getHeadingDegrees());
        telemetry.addData("TURN_P", "%.3f", BlueConstants.TURN_P);
        telemetry.addData("TURN_D", "%.4f", BlueConstants.TURN_D);
        for (SwerveModule m : drive.modules) {
            telemetry.addData(m.name, "target %.1f  angle %.1f  speed %.2f",
                    Math.toDegrees(m.getTargetAngle()), Math.toDegrees(m.getAngle()), m.getSpeed());
        }
        telemetry.addData("Log", logger.isWorking() ? logger.getPath() : "not saving");
    }

    String deg(double radians) {
        return String.format("%.1f", Math.toDegrees(radians));
    }

    @Override
    public void stop() {
        drive.stop();
        logger.close();
    }
}

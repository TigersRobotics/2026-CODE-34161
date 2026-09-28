package org.firstinspires.ftc.teamcode.blue.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blue.drive.SwerveDrive;
import org.firstinspires.ftc.teamcode.blue.drive.SwerveModule;
import org.firstinspires.ftc.teamcode.blue.util.DataLogger;

@TeleOp(name = "Swerve Module Test", group = "Blue Test")
public class SwerveModuleTest extends OpMode {
    SwerveDrive drive;
    DataLogger logger;
    ElapsedTime timer = new ElapsedTime();

    int selected = 0;
    boolean holdZero = false;
    double[] savedOffsets = new double[4];

    @Override
    public void init() {
        drive = new SwerveDrive(hardwareMap);
        logger = new DataLogger("swerve_module", "time", "module", "voltage", "raw", "angle", "target", "drive");
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        if (gamepad1.dpadRightWasPressed()) selected = (selected + 1) % 4;
        if (gamepad1.dpadLeftWasPressed()) selected = (selected + 3) % 4;
        if (gamepad1.aWasPressed()) holdZero = !holdZero;

        SwerveModule m = drive.modules[selected];

        if (gamepad1.bWasPressed()) {
            savedOffsets[selected] = m.getRawAngle();
            m.setOffset(savedOffsets[selected]);
        }

        for (SwerveModule other : drive.modules) {
            if (other != m) other.stop();
        }

        if (holdZero) {
            m.set(-gamepad1.left_stick_y, 0);
        } else {
            m.setRaw(-gamepad1.left_stick_y, gamepad1.right_stick_x);
        }

        for (SwerveModule s : drive.modules) {
            logger.log(String.format("%.3f", timer.seconds()), s.name,
                    String.format("%.3f", s.getVoltage()),
                    String.format("%.3f", s.getRawAngle()),
                    String.format("%.3f", s.getAngle()),
                    String.format("%.3f", s.getTargetAngle()),
                    String.format("%.2f", s.getSpeed()));
        }

        telemetry.addLine("dpad L/R = pick module, left stick = drive, right stick = turn");
        telemetry.addLine("A = hold at 0, B = save offset (wheel pointing forward)");
        telemetry.addData("Selected", m.name);
        telemetry.addData("Hold zero", holdZero);
        for (SwerveModule s : drive.modules) {
            telemetry.addData(s.name, "%.2fV  raw %.1f  angle %.1f", s.getVoltage(),
                    Math.toDegrees(s.getRawAngle()), Math.toDegrees(s.getAngle()));
        }
        telemetry.addLine();
        telemetry.addLine("Copy into BlueConstants:");
        telemetry.addData("FL_OFFSET", "%.4f", savedOffsets[0]);
        telemetry.addData("FR_OFFSET", "%.4f", savedOffsets[1]);
        telemetry.addData("BL_OFFSET", "%.4f", savedOffsets[2]);
        telemetry.addData("BR_OFFSET", "%.4f", savedOffsets[3]);
        telemetry.addData("Log", logger.isWorking() ? logger.getPath() : "not saving");
    }

    @Override
    public void stop() {
        drive.stop();
        logger.close();
    }
}

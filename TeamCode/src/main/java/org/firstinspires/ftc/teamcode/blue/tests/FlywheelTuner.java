package org.firstinspires.ftc.teamcode.blue.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blue.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.blue.util.BlueConstants;
import org.firstinspires.ftc.teamcode.blue.util.DataLogger;

@TeleOp(name = "Flywheel Tuner", group = "Blue Test")
public class FlywheelTuner extends OpMode {
    enum Mode { IDLE, MEASURING, STEP }

    Launcher launcher;
    DataLogger logger;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime stepTimer = new ElapsedTime();

    Mode mode = Mode.IDLE;
    double maxTicks = 0;
    double lowRpm = 2500;
    double highRpm = 3500;
    boolean high = false;
    double riseTime = -1;

    @Override
    public void init() {
        launcher = new Launcher(hardwareMap);
        logger = new DataLogger("flywheel", "time", "mode", "targetRpm", "rpm", "p", "i", "d", "f");
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            mode = Mode.MEASURING;
            maxTicks = 0;
            stepTimer.reset();
        }
        if (gamepad1.xWasPressed() && maxTicks > 0) {
            BlueConstants.LAUNCHER_F = 32767 / maxTicks;
            BlueConstants.LAUNCHER_P = 0.1 * BlueConstants.LAUNCHER_F;
            BlueConstants.LAUNCHER_I = 0.1 * BlueConstants.LAUNCHER_P;
            BlueConstants.LAUNCHER_D = 0;
            launcher.applyPidf();
        }
        if (gamepad1.bWasPressed()) {
            mode = Mode.STEP;
            high = !high;
            riseTime = -1;
            stepTimer.reset();
        }
        if (gamepad1.yWasPressed()) {
            mode = Mode.IDLE;
            launcher.stop();
        }

        boolean changed = false;
        if (gamepad1.dpadUpWasPressed()) {
            BlueConstants.LAUNCHER_P += 0.1;
            changed = true;
        }
        if (gamepad1.dpadDownWasPressed()) {
            BlueConstants.LAUNCHER_P = Math.max(0, BlueConstants.LAUNCHER_P - 0.1);
            changed = true;
        }
        if (gamepad1.dpadRightWasPressed()) {
            BlueConstants.LAUNCHER_F += 0.1;
            changed = true;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            BlueConstants.LAUNCHER_F = Math.max(0, BlueConstants.LAUNCHER_F - 0.1);
            changed = true;
        }
        if (changed) launcher.applyPidf();
        if (gamepad1.rightBumperWasPressed()) highRpm += 100;
        if (gamepad1.leftBumperWasPressed()) highRpm -= 100;

        double target = 0;
        if (mode == Mode.MEASURING) {
            launcher.setRawPower(1.0);
            maxTicks = Math.max(maxTicks, launcher.getTicksPerSecond());
            if (stepTimer.seconds() > 4) {
                launcher.stop();
                mode = Mode.IDLE;
            }
        } else if (mode == Mode.STEP) {
            target = high ? highRpm : lowRpm;
            launcher.setRpm(target);
            if (riseTime < 0 && launcher.isReady()) riseTime = stepTimer.seconds();
        }

        logger.log(String.format("%.3f", timer.seconds()), mode, String.format("%.0f", target),
                String.format("%.0f", launcher.getRpm()),
                String.format("%.3f", BlueConstants.LAUNCHER_P), String.format("%.3f", BlueConstants.LAUNCHER_I),
                String.format("%.3f", BlueConstants.LAUNCHER_D), String.format("%.3f", BlueConstants.LAUNCHER_F));

        telemetry.addLine("1) A = full power 4 s to find max speed");
        telemetry.addLine("2) X = use those numbers");
        telemetry.addLine("3) B = jump between low/high rpm, watch rise time");
        telemetry.addLine("dpad U/D = P, dpad L/R = F, bumpers = high rpm, Y = stop");
        telemetry.addData("Mode", mode);
        telemetry.addData("Max ticks/s", "%.0f", maxTicks);
        telemetry.addData("Max rpm", "%.0f", maxTicks / BlueConstants.LAUNCHER_TICKS_PER_REV * 60);
        telemetry.addData("RPM", "%.0f / %.0f", launcher.getRpm(), target);
        telemetry.addData("Rise time", riseTime < 0 ? "-" : String.format("%.2f s", riseTime));
        telemetry.addData("Low / high", "%.0f / %.0f", lowRpm, highRpm);
        telemetry.addLine();
        telemetry.addLine("Copy into BlueConstants:");
        telemetry.addData("LAUNCHER_P", "%.3f", BlueConstants.LAUNCHER_P);
        telemetry.addData("LAUNCHER_I", "%.3f", BlueConstants.LAUNCHER_I);
        telemetry.addData("LAUNCHER_D", "%.3f", BlueConstants.LAUNCHER_D);
        telemetry.addData("LAUNCHER_F", "%.3f", BlueConstants.LAUNCHER_F);
    }

    @Override
    public void stop() {
        launcher.stop();
        logger.close();
    }
}

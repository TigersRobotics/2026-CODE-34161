package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.constants.RobotSpecs;
import org.firstinspires.ftc.teamcode.robotControllers.MecanumDriveTrainController;
import org.firstinspires.ftc.teamcode.robotControllers.SingleIntakeController;
import org.firstinspires.ftc.teamcode.robotControllers.UptakeController;
import org.firstinspires.ftc.teamcode.sensors.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.utils.ShotCalculator;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;
import org.firstinspires.ftc.teamcode.utils.UtilMethods;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name = "Shot Calibration", group = "Tuning")
public class ShotCalibration extends OpMode {
    private static final int STOPS = 10;
    private static final double START_X_IN = 24;
    private static final double START_Y_IN = 24;
    private static final double START_HEADING_DEG = 90;
    private static final double GOAL_X_IN = 72;
    private static final double GOAL_Y_IN = 132;
    private static final double GOAL_HEIGHT_IN = 16;
    private static final double ARRIVE_IN = 3;
    private static final double ARRIVE_DEG = 8;
    private static final double DRIVE_POWER = 0.4;
    private static final double EFFICIENCY_STEP = 0.02;
    private static final long FEED_PULSE_MS = 350;

    private static final double[][] STOPS_IN = {
            {36, 36},
            {36, 60},
            {36, 84},
            {36, 108},
            {60, 108},
            {60, 84},
            {60, 60},
            {84, 60},
            {84, 84},
            {84, 108}
    };

    private enum Phase {
        DRIVE, AIM, MARK, DONE
    }

    private Servo hoodServo;
    private DcMotorEx flywheel;
    private GoBildaOdometry odometry;
    private BufferedWriter log;
    private File logFile;

    private Phase phase = Phase.DRIVE;
    private int stopIndex = 0;
    private int made = 0;
    private int over = 0;
    private int under = 0;
    private ShotCalculator.Shot shot = null;

    private double commandedHood = ShotCalculator.MIN_HOOD_DEG;
    private double previousHood = ShotCalculator.MIN_HOOD_DEG;
    private long hoodCommandNanos = 0;
    private double targetRpm = 0;
    private double feedRpm = 0;
    private boolean feeding = false;
    private boolean feedArmed = true;
    private long feedStartedNanos = 0;
    private File tuningFile;

    @Override
    public void init() {
        Timer.init();
        TimedTelemetry.init(telemetry);
        hoodServo = hardwareMap.get(Servo.class, "hood");
        flywheel = hardwareMap.get(DcMotorEx.class, HardwareNames.FLYWHEEL_NAME);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MecanumDriveTrainController.init(hardwareMap, true);
        SingleIntakeController.init(hardwareMap, true);
        UptakeController.init(hardwareMap, true);
        odometry = new GoBildaOdometry(hardwareMap, HardwareNames.ODOMETRY_NAME);
        odometry.setPose(new Pose2D(DistanceUnit.INCH, START_X_IN, START_Y_IN, AngleUnit.DEGREES, START_HEADING_DEG));
        tuningFile = new File(hardwareMap.appContext.getFilesDir(), "shot-tuning.txt");
        ShotCalculator.loadTuning(tuningFile);
        openLog();
        hoodServo.setPosition(ShotCalculator.hoodServoPosition(commandedHood));
        telemetry.addLine("Place the robot in the near-left corner, facing up the field");
        telemetry.addData("Stops", STOPS);
        if (logFile != null) {
            telemetry.addData("Log", logFile.getAbsolutePath());
        }
    }

    @Override
    public void loop() {
        Timer.update();
        odometry.update();
        if (phase == Phase.DRIVE) {
            drive();
        } else if (phase == Phase.AIM) {
            MecanumDriveTrainController.update(0, 0, 0);
            if (shot != null) {
                holdShot();
                feed();
            }
            if (gamepad1.leftBumperWasPressed() && !feeding) {
                nextStop();
            }
        } else if (phase == Phase.MARK) {
            MecanumDriveTrainController.update(0, 0, 0);
            holdShot();
            if (gamepad1.xWasPressed()) {
                record("MADE");
            } else if (gamepad1.circleWasPressed()) {
                record("OVER");
            } else if (gamepad1.squareWasPressed()) {
                record("UNDER");
            }
        } else {
            MecanumDriveTrainController.update(0, 0, 0);
            if (flywheel != null) {
                flywheel.setVelocity(0);
            }
        }
        draw();
    }

    @Override
    public void stop() {
        if (flywheel != null) {
            flywheel.setVelocity(0);
        }
        SingleIntakeController.update(0);
        UptakeController.update(0);
        if (MecanumDriveTrainController.isInitialized) {
            MecanumDriveTrainController.update(0, 0, 0);
            MecanumDriveTrainController.stop();
        }
        SingleIntakeController.stop();
        UptakeController.stop();
        closeLog();
    }

    private void drive() {
        double x = odometry.getX(DistanceUnit.INCH);
        double y = odometry.getY(DistanceUnit.INCH);
        double heading = odometry.getHeading(AngleUnit.DEGREES);
        double targetX = STOPS_IN[stopIndex][0];
        double targetY = STOPS_IN[stopIndex][1];
        double dx = targetX - x;
        double dy = targetY - y;
        double headingRad = Math.toRadians(heading);
        double forward = dx * Math.cos(headingRad) + dy * Math.sin(headingRad);
        double right = dx * Math.sin(headingRad) - dy * Math.cos(headingRad);
        double face = Math.toDegrees(Math.atan2(GOAL_Y_IN - y, GOAL_X_IN - x));
        double turnError = wrap(face - heading);
        if (Math.hypot(dx, dy) <= ARRIVE_IN && Math.abs(turnError) <= ARRIVE_DEG) {
            MecanumDriveTrainController.update(0, 0, 0);
            prepareShot();
            phase = Phase.AIM;
            return;
        }
        MecanumDriveTrainController.update(
                -axisPower(forward),
                -axisPower(right),
                -UtilMethods.clip(turnError * 0.012, -DRIVE_POWER, DRIVE_POWER));
    }

    private void prepareShot() {
        feeding = false;
        double dx = GOAL_X_IN - odometry.getX(DistanceUnit.INCH);
        double dy = GOAL_Y_IN - odometry.getY(DistanceUnit.INCH);
        double distanceM = inchesToMeters(Math.hypot(dx, dy));
        double heightM = inchesToMeters(GOAL_HEIGHT_IN);
        ShotCalculator.Shot[] choices = ShotCalculator.shots(distanceM, heightM, commandedHood, currentRpm());
        if (choices.length == 0) {
            shot = null;
            flywheel.setVelocity(0);
            return;
        }
        shot = choices[0];
        command(shot);
    }

    private void command(ShotCalculator.Shot next) {
        previousHood = commandedHood;
        commandedHood = next.hoodDeg;
        targetRpm = next.rpm;
        hoodCommandNanos = System.nanoTime();
    }

    private void holdShot() {
        hoodServo.setPosition(ShotCalculator.hoodServoPosition(commandedHood));
        flywheel.setVelocity(targetRpm * RobotSpecs.TICKS_PER_REV / 60.0);
    }

    private void feed() {
        boolean trigger = gamepad1.right_trigger > 0.5;
        if (!trigger) {
            feedArmed = true;
        }
        if (!feeding) {
            if (trigger && feedArmed && calculatorReady()) {
                feeding = true;
                feedArmed = false;
                feedStartedNanos = System.nanoTime();
                feedRpm = currentRpm();
            } else {
                SingleIntakeController.update(0);
                UptakeController.update(0);
                return;
            }
        }
        SingleIntakeController.update(1);
        UptakeController.update(1);
        if ((System.nanoTime() - feedStartedNanos) / 1000000L >= FEED_PULSE_MS) {
            feeding = false;
            SingleIntakeController.update(0);
            UptakeController.update(0);
            phase = Phase.MARK;
        }
    }

    private void record(String mark) {
        writeRow(mark);
        if ("MADE".equals(mark)) {
            made++;
        } else if ("OVER".equals(mark)) {
            over++;
            ShotCalculator.EFFICIENCY = clamp(ShotCalculator.EFFICIENCY + EFFICIENCY_STEP, 0.2, 0.9);
        } else {
            under++;
            ShotCalculator.EFFICIENCY = clamp(ShotCalculator.EFFICIENCY - EFFICIENCY_STEP, 0.2, 0.9);
        }
        saveTuning();
        nextStop();
    }

    private void nextStop() {
        flywheel.setVelocity(0);
        feeding = false;
        SingleIntakeController.update(0);
        UptakeController.update(0);
        stopIndex++;
        if (stopIndex >= STOPS) {
            phase = Phase.DONE;
            return;
        }
        phase = Phase.DRIVE;
    }

    private boolean calculatorReady() {
        return shot != null && hoodArrived();
    }

    private boolean hoodArrived() {
        double wait = Math.abs(commandedHood - previousHood) / ShotCalculator.HOOD_DEG_PER_SEC;
        return (System.nanoTime() - hoodCommandNanos) / 1e9 >= wait;
    }

    private double currentRpm() {
        return flywheel.getVelocity() * 60.0 / RobotSpecs.TICKS_PER_REV;
    }

    private void draw() {
        TimedTelemetry.update();
        telemetry.addData("Phase", phase);
        telemetry.addData("Stop", "%d / %d", Math.min(stopIndex + 1, STOPS), STOPS);
        telemetry.addData("Pose in", "%.0f, %.0f  %.0f deg",
                odometry.getX(DistanceUnit.INCH),
                odometry.getY(DistanceUnit.INCH),
                odometry.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Efficiency", "%.2f", ShotCalculator.EFFICIENCY);
        telemetry.addData("Made / over / under", "%d / %d / %d", made, over, under);
        if (phase == Phase.DRIVE) {
            telemetry.addData("Driving to", "%.0f, %.0f", STOPS_IN[stopIndex][0], STOPS_IN[stopIndex][1]);
        } else if (phase == Phase.AIM || phase == Phase.MARK) {
            if (shot == null) {
                telemetry.addLine("No shot reaches the goal from here");
                telemetry.addLine("Left bumper: next stop");
            } else {
                double dx = GOAL_X_IN - odometry.getX(DistanceUnit.INCH);
                double dy = GOAL_Y_IN - odometry.getY(DistanceUnit.INCH);
                telemetry.addData("Distance in", "%.0f", Math.hypot(dx, dy));
                telemetry.addData("Hood", "%.1f deg", shot.hoodDeg);
                telemetry.addData("Target rpm", "%.0f", shot.rpm);
                telemetry.addData("Wheel rpm", "%.0f", currentRpm());
                telemetry.addData("Feed", calculatorReady() ? "ready" : "wait");
                if (phase == Phase.AIM) {
                    telemetry.addLine("Tap right trigger for one ball");
                    telemetry.addLine("Left bumper skips this stop");
                } else {
                    telemetry.addLine("X made, circle over, square under");
                }
            }
        } else {
            telemetry.addLine("10 shots logged");
            telemetry.addData("Tuned efficiency", "%.2f", ShotCalculator.EFFICIENCY);
            if (logFile != null) {
                telemetry.addData("Log", logFile.getAbsolutePath());
            }
        }
        telemetry.update();
    }

    private void openLog() {
        try {
            logFile = new File(hardwareMap.appContext.getFilesDir(), "shot-route-calibration.csv");
            boolean fresh = !logFile.exists() || logFile.length() == 0;
            log = new BufferedWriter(new FileWriter(logFile, true));
            if (fresh) {
                log.write("millis,stop,xIn,yIn,distanceM,hoodDeg,launchDeg,speedMps,commandedRpm,actualRpm,efficiency,mark");
                log.newLine();
                log.flush();
            }
        } catch (IOException e) {
            log = null;
            telemetry.addData("Log error", e.getMessage());
        }
    }

    private void writeRow(String mark) {
        if (log == null || shot == null) {
            return;
        }
        try {
            double x = odometry.getX(DistanceUnit.INCH);
            double y = odometry.getY(DistanceUnit.INCH);
            double distanceM = inchesToMeters(Math.hypot(GOAL_X_IN - x, GOAL_Y_IN - y));
            log.write(System.currentTimeMillis()
                    + "," + (stopIndex + 1)
                    + "," + x
                    + "," + y
                    + "," + distanceM
                    + "," + shot.hoodDeg
                    + "," + shot.launchDeg
                    + "," + shot.speedMps
                    + "," + shot.rpm
                    + "," + feedRpm
                    + "," + ShotCalculator.EFFICIENCY
                    + "," + mark);
            log.newLine();
            log.flush();
        } catch (IOException e) {
            telemetry.addData("Log error", e.getMessage());
        }
    }

    private void closeLog() {
        if (log == null) {
            return;
        }
        try {
            log.close();
        } catch (IOException ignored) {
        }
        log = null;
    }

    private void saveTuning() {
        if (tuningFile == null) {
            return;
        }
        try {
            ShotCalculator.saveTuning(tuningFile);
        } catch (IOException e) {
            telemetry.addData("Tune error", e.getMessage());
        }
    }

    private static double axisPower(double inches) {
        double power = UtilMethods.clip(inches * 0.035, -DRIVE_POWER, DRIVE_POWER);
        if (Math.abs(inches) > ARRIVE_IN && Math.abs(power) < 0.12) {
            power = 0.12 * Math.signum(inches);
        }
        return power;
    }

    private static double wrap(double degrees) {
        while (degrees > 180) {
            degrees -= 360;
        }
        while (degrees < -180) {
            degrees += 360;
        }
        return degrees;
    }

    private static double inchesToMeters(double inches) {
        return inches * 0.0254;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

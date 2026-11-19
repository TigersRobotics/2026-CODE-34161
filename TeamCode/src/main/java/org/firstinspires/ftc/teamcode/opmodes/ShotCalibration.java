package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.constants.MapSpecs;
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

@Deprecated
@Disabled
@TeleOp(name = "Shot Calibration", group = "Tuning")
public class ShotCalibration extends OpMode {
    private static final int LOOP_STOPS = 9;
    private static final int STOPS = 11;
    private static final int MAX_TRIALS = 8;
    private static final int SHOT_CAP = 100;
    private static final int EFF_STEPS_PER_VISIT = 2;
    private static final int FRESH_SHOTS = 4;
    private static final double CLOSE_IN = 40;
    private static final double FAR_IN = 68;
    private static final double[][] STOPS_IN = {
            {9, 9},
            {42, 36},
            {72, 20},
            {72, 9},
            {30, 28},
            {108, 36},
            {120, 18},
            {18, 72},
            {18, 108},
            {100, 108},
            {126, 108}
    };
    private static final String[] NAMES = {
            "corner",
            "close",
            "middle",
            "wall",
            "mid corner",
            "other close",
            "other side",
            "left side",
            "far left",
            "behind",
            "far right"
    };
    private static final double HOOD_SEARCH_DEG = 3;
    private static final double APPROACH_X = 28;
    private static final double APPROACH_Y = 28;
    private static final double PRESS_HEADING = 90;
    private static final double PRESS_BOX_IN = 40;
    private static final double PRESS_RUNAWAY_IN = 8;
    private static final double ROBOT_HALF_IN = RobotSpecs.LENGTH_X_IN / 0.0254 / 2.0;
    private static final double ARRIVE_IN = 3;
    private static final double ARRIVE_DEG = 8;
    private static final double DRIVE_POWER = 0.4;
    private static final double EFFICIENCY_STEP = 0.02;
    private static final double WEIGHT_STEP = 0.1;
    private static final double SPEED_STEP = 0.05;
    private static final long FEED_PULSE_MS = 350;
    private static final long PRESS_NS = 1500000000L;

    private enum Phase {
        SQUARE, PRESS, DRIVE, AIM, MARK, DONE
    }

    private enum RouteMode {
        LOOP, FRESH, TRICK
    }

    private static final class Trial {
        final ShotCalculator.Shot shot;
        final String mark;
        final double efficiency;

        Trial(ShotCalculator.Shot shot, String mark, double efficiency) {
            this.shot = shot;
            this.mark = mark;
            this.efficiency = efficiency;
        }
    }

    private Servo hoodServo;
    private DcMotorEx flywheel;
    private DcMotorEx yawMotor;
    private GoBildaOdometry odometry;
    private BufferedWriter log;
    private File logFile;
    private File tuningFile;

    private Phase phase = Phase.PRESS;
    private final RouteMode route = RouteMode.LOOP;
    private int spotIndex = 0;
    private final int loopNumber = 1;
    private final int freshLeft = 0;
    private final int resumeAt = -1;
    private int totalShots = 0;
    private int effStepsThisVisit = 0;
    private int pendingConfirm = -1;
    private boolean confirmMode = false;
    private boolean effChanged = false;
    private boolean bracketTight = false;
    private double effLo = 0.2;
    private double effHi = 0.9;
    private double effSnapshot = ShotCalculator.EFFICIENCY;
    private double lastDelta = 0;
    private String why = "Pressing into the corner to reset position.";

    private final Trial[][] trials = new Trial[STOPS][MAX_TRIALS];
    private final int[] trialCount = new int[STOPS];
    private final boolean[] madeHere = new boolean[STOPS];
    private final double[] madeHood = new double[STOPS];
    private final boolean[] visited = new boolean[STOPS];
    private final boolean[] settled = new boolean[STOPS];
    private final boolean[] passMade = new boolean[LOOP_STOPS];
    private final boolean[] hoodUsed = new boolean[41];
    private final double[] stopX = new double[STOPS];
    private final double[] stopY = new double[STOPS];
    private final String[] names = new String[STOPS];
    private final int[] settleKind = new int[STOPS];
    private final boolean[] inPlace = new boolean[STOPS];
    private final int[] hiveOf = new int[STOPS];
    private final boolean[] steep = new boolean[STOPS];
    private final double[] spotDistanceIn = new double[STOPS];
    private final double leftHiveX = 58;
    private final double leftHiveY = 60;
    private final double rightHiveX = 91;
    private final double rightHiveY = 60;
    private final double approachX = 28;
    private final double approachY = 28;
    private final double pressTargetX = -8;
    private final double pressTargetY = -8;
    private final double poseX = 9;
    private final double poseY = 9;

    private ShotCalculator.Shot[] choices = new ShotCalculator.Shot[0];
    private ShotCalculator.Shot shot = null;
    private double hiveX = 72;
    private double hiveY = 60;
    private double goalHeightM = 0.4;

    private double commandedHood = ShotCalculator.MIN_HOOD_DEG;
    private double previousHood = ShotCalculator.MIN_HOOD_DEG;
    private long hoodCommandNanos = 0;
    private double targetRpm = 0;
    private double feedRpm = 0;
    private boolean feeding = false;
    private boolean feedArmed = true;
    private long feedStartedNanos = 0;
    private long pressUntil = 0;
    private double pressStartX = 0;
    private double pressStartY = 0;
    private boolean pressRejected = false;

    @Override
    public void init() {
        Timer.init();
        TimedTelemetry.init(telemetry);
        if (MapSpecs.HIVE_COORDINATES.length > 0) {
            hiveX = MapSpecs.HIVE_COORDINATES[0].getX();
            hiveY = MapSpecs.HIVE_COORDINATES[0].getY();
        }
        double heightIn = MapSpecs.HIVE_HEIGHT - RobotSpecs.HOOD_HEIGHT;
        if (heightIn < 2) {
            heightIn = 2;
        }
        goalHeightM = inchesToMeters(heightIn);
        for (int i = 0; i < STOPS; i++) {
            spotDistanceIn[i] = Math.hypot(STOPS_IN[i][0] - hiveX, STOPS_IN[i][1] - hiveY);
        }

        hoodServo = hardwareMap.get(Servo.class, HardwareNames.TURRET_UP_ANGLE_SERVO_NAME);
        flywheel = hardwareMap.get(DcMotorEx.class, HardwareNames.FLYWHEEL_NAME);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        yawMotor = hardwareMap.get(DcMotorEx.class, HardwareNames.TURRET_YAW_MOTOR_NAME);
        yawMotor.setTargetPosition(0);
        yawMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        yawMotor.setDirection(DcMotor.Direction.FORWARD);
        yawMotor.setPower(1.0);

        MecanumDriveTrainController.init(hardwareMap, true);
        SingleIntakeController.init(hardwareMap, true);
        UptakeController.init(hardwareMap, true);
        odometry = new GoBildaOdometry(hardwareMap, HardwareNames.ODOMETRY_NAME);
        odometry.setPose(new Pose2D(DistanceUnit.INCH, ROBOT_HALF_IN, ROBOT_HALF_IN, AngleUnit.DEGREES, PRESS_HEADING));
        tuningFile = new File(hardwareMap.appContext.getFilesDir(), "shot-tuning.txt");
        ShotCalculator.loadTuning(tuningFile);
        ShotCalculator.DRAG_PER_M = 0;
        saveTuning();
        openLog();
        hoodServo.setPosition(ShotCalculator.hoodServoPosition(commandedHood));
        telemetry.addLine("Point the turret forward before init. That position is yaw zero.");
        telemetry.addLine("Robot in the near-left corner, facing up the field.");
        telemetry.addLine("No camera. Hive is the map point. Turret aims from odometry.");
        telemetry.addLine("Hood first. Speed changes only when the hoods agree.");
        telemetry.addData("Hive in", "%.0f, %.0f", hiveX, hiveY);
        telemetry.addData("Efficiency", "%.2f", ShotCalculator.EFFICIENCY);
    }

    @Override
    public void loop() {
        Timer.update();
        odometry.update();
        trackHive();
        if (phase == Phase.SQUARE) {
            follow(APPROACH_X, APPROACH_Y, PRESS_HEADING);
            if (arrived(APPROACH_X, APPROACH_Y, PRESS_HEADING)) {
                pressUntil = 0;
                phase = Phase.PRESS;
            }
        } else if (phase == Phase.PRESS) {
            if (pressUntil == 0) {
                pressStartX = odometry.getX(DistanceUnit.INCH);
                pressStartY = odometry.getY(DistanceUnit.INCH);
                pressRejected = false;
                pressUntil = System.nanoTime() + PRESS_NS;
            }
            if (!pressRejected && !pressIntoCorner()) {
                pressRejected = true;
                MecanumDriveTrainController.update(0, 0, 0);
            }
            if (pressRejected || System.nanoTime() >= pressUntil) {
                endPress();
            }
        } else if (phase == Phase.DRIVE) {
            double face = hiveBearing();
            follow(STOPS_IN[spotIndex][0], STOPS_IN[spotIndex][1], face);
            if (arrived(STOPS_IN[spotIndex][0], STOPS_IN[spotIndex][1], face) && prepareShot()) {
                phase = Phase.AIM;
            }
        } else if (phase == Phase.AIM) {
            MecanumDriveTrainController.update(0, 0, 0);
            if (shot == null) {
                settled[spotIndex] = true;
                why = "No shot at " + NAMES[spotIndex] + ".";
                leaveForNext();
            } else if (gamepad1.leftBumperWasPressed() && !feeding) {
                skip();
            } else {
                holdShot();
                feed();
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
            } else if (gamepad1.leftBumperWasPressed() && !feeding) {
                skip();
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
        if (yawMotor != null) {
            yawMotor.setPower(0);
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

    private void endPress() {
        MecanumDriveTrainController.update(0, 0, 0);
        pressUntil = 0;
        if (pressRejected) {
            finish("Corner press left the near-left corner. Pose was not reset.");
            return;
        }
        odometry.setPose(new Pose2D(DistanceUnit.INCH, ROBOT_HALF_IN, ROBOT_HALF_IN, AngleUnit.DEGREES, PRESS_HEADING));
        if (spotIndex < 0 && !selectNext()) {
            return;
        }
        phase = Phase.DRIVE;
    }

    private boolean pressIntoCorner() {
        double x = odometry.getX(DistanceUnit.INCH);
        double y = odometry.getY(DistanceUnit.INCH);
        double heading = odometry.getHeading(AngleUnit.DEGREES);
        if (x > PRESS_BOX_IN || y > PRESS_BOX_IN || x > pressStartX + PRESS_RUNAWAY_IN || y > pressStartY + PRESS_RUNAWAY_IN) {
            why = "Corner press stopped. The robot moved away from the near-left walls.";
            return false;
        }
        if (Math.abs(wrap(PRESS_HEADING - heading)) > 25) {
            why = "Corner press stopped. Heading is not facing up the field.";
            return false;
        }
        MecanumDriveTrainController.update(
                DRIVE_POWER,
                DRIVE_POWER,
                -UtilMethods.clip(wrap(PRESS_HEADING - heading) * 0.012, -DRIVE_POWER, DRIVE_POWER));
        return true;
    }

    private boolean selectNext() {
        if (totalShots >= SHOT_CAP) {
            finish("Stopped at " + SHOT_CAP + " shots.");
            return false;
        }
        int next = chooseSpot();
        if (next < 0) {
            finish(doneReason());
            return false;
        }
        spotIndex = next;
        confirmMode = pendingConfirm == next;
        if (confirmMode) {
            pendingConfirm = -1;
            why = "Realigning, then checking a shot that already went in.";
        } else {
            visited[next] = true;
            effStepsThisVisit = 0;
            why = "Realigning, then " + NAMES[next] + " (" + Math.round(spotDistanceIn[next]) + " in).";
        }
        return true;
    }

    private int chooseSpot() {
        if (pendingConfirm >= 0 && madeHere[pendingConfirm]) {
            return pendingConfirm;
        }
        if (covered() && !effChanged) {
            return -1;
        }
        if (!bandMake(0, CLOSE_IN)) {
            int close = pickUnvisited(0, CLOSE_IN, false);
            if (close >= 0) {
                return close;
            }
        }
        if (!bandMake(FAR_IN, 1000)) {
            int far = pickUnvisited(FAR_IN, 1000, true);
            if (far >= 0) {
                return far;
            }
        }
        int any = pickUnvisited(0, 1000, true);
        if (any >= 0) {
            return any;
        }
        for (int i = 0; i < STOPS; i++) {
            if (!madeHere[i] && !settled[i]) {
                return i;
            }
        }
        return -1;
    }

    private int pickUnvisited(double minIn, double maxIn, boolean farthest) {
        int best = -1;
        double bestDistance = farthest ? -1 : 1e9;
        for (int i = 0; i < STOPS; i++) {
            if (visited[i] || settled[i] || madeHere[i]) {
                continue;
            }
            if (spotDistanceIn[i] < minIn || spotDistanceIn[i] >= maxIn) {
                continue;
            }
            if (farthest ? spotDistanceIn[i] > bestDistance : spotDistanceIn[i] < bestDistance) {
                best = i;
                bestDistance = spotDistanceIn[i];
            }
        }
        return best;
    }

    private boolean prepareShot() {
        loadChoices();
        if (choices.length == 0 && !anyMake() && stepEff(-EFFICIENCY_STEP)) {
            loadChoices();
        }
        if (choices.length == 0) {
            settled[spotIndex] = true;
            why = "No hood reaches the hive from " + NAMES[spotIndex] + ".";
            leaveForNext();
            return false;
        }
        ShotCalculator.Shot next;
        if (confirmMode) {
            next = untriedNear(madeHood[spotIndex]);
            if (next == null) {
                next = nearest(madeHood[spotIndex]);
            }
            why = "Checking a make at the new speed, " + NAMES[spotIndex] + ".";
        } else {
            next = firstFreshHood();
            why = NAMES[spotIndex] + ", " + Math.round(spotDistanceIn[spotIndex])
                    + " in. Hood first. Speed stays until the hoods agree.";
        }
        if (next == null) {
            settled[spotIndex] = true;
            why = "No new hood left at " + NAMES[spotIndex] + ".";
            leaveForNext();
            return false;
        }
        command(next);
        return true;
    }

    private void record(String mark) {
        writeRow(mark);
        if (shot == null || trialCount[spotIndex] >= MAX_TRIALS) {
            settled[spotIndex] = true;
            leaveForNext();
            return;
        }
        trials[spotIndex][trialCount[spotIndex]++] = new Trial(shot, mark, ShotCalculator.EFFICIENCY);
        totalShots++;
        if (totalShots >= SHOT_CAP) {
            if ("MADE".equals(mark)) {
                madeHere[spotIndex] = true;
                madeHood[spotIndex] = shot.hoodDeg;
            }
            finish("Stopped at " + SHOT_CAP + " shots.");
            return;
        }
        if ("MADE".equals(mark)) {
            onMake();
        } else if (confirmMode) {
            onConfirmMiss();
        } else {
            onMiss(mark);
        }
    }

    private void onMake() {
        boolean wasConfirm = confirmMode;
        madeHere[spotIndex] = true;
        madeHood[spotIndex] = shot.hoodDeg;
        confirmMode = false;
        boolean nudged = nudgeFromThisSpot();
        int other = otherMake();
        if (!wasConfirm && effChanged && other >= 0) {
            pendingConfirm = other;
            why = "Made at " + NAMES[spotIndex] + ". Checking an older make before trusting the speed.";
            leaveForNext();
            return;
        }
        effChanged = false;
        pendingConfirm = -1;
        why = "Made at " + NAMES[spotIndex] + ". Efficiency stays " + num(ShotCalculator.EFFICIENCY) + ".";
        if (nudged) {
            why = why + " Weights moved toward that hood.";
        }
        saveTuning();
        if (covered()) {
            finish(why + " Close, middle, and far are in.");
            return;
        }
        leaveForNext();
    }

    private void onConfirmMiss() {
        ShotCalculator.EFFICIENCY = effSnapshot;
        if (lastDelta > 0) {
            effHi = effSnapshot;
        } else {
            effLo = effSnapshot;
        }
        effChanged = false;
        confirmMode = false;
        saveTuning();
        why = "Speed change broke a shot that was in. Efficiency back to " + num(ShotCalculator.EFFICIENCY) + ".";
        leaveForNext();
    }

    private void onMiss(String mark) {
        ShotCalculator.Shot between = bracketHood();
        if (between != null) {
            why = "One hood was long and one was short. Trying the hood between them.";
            command(between);
            phase = Phase.AIM;
            return;
        }
        if (bracketTight) {
            settled[spotIndex] = true;
            why = "Hoods landed on both sides of the hive. Efficiency stays " + num(ShotCalculator.EFFICIENCY) + ".";
            leaveForNext();
            return;
        }
        if ("OVER".equals(mark)) {
            ShotCalculator.Shot lower = untriedNear(shot.hoodDeg - HOOD_SEARCH_DEG);
            if (lower != null) {
                why = "Long. Trying a steeper hood before changing speed.";
                command(lower);
                phase = Phase.AIM;
                return;
            }
        } else if ("UNDER".equals(mark)) {
            ShotCalculator.Shot higher = untriedNear(shot.hoodDeg + HOOD_SEARCH_DEG);
            if (higher != null) {
                why = "Short. Trying a flatter hood before changing speed.";
                command(higher);
                phase = Phase.AIM;
                return;
            }
        }
        int overs = countMark("OVER");
        int unders = countMark("UNDER");
        if (unders == 0 && overs >= 1 && effStepsThisVisit < EFF_STEPS_PER_VISIT && stepEff(EFFICIENCY_STEP)) {
            why = "The hoods that can land shorter still went long. " + stepNote() + " Reshooting here.";
            retestHood();
            return;
        }
        if (overs == 0 && unders >= 1 && effStepsThisVisit < EFF_STEPS_PER_VISIT && stepEff(-EFFICIENCY_STEP)) {
            why = "The hoods that can land longer still came up short. " + stepNote() + " Reshooting here.";
            retestHood();
            return;
        }
        settled[spotIndex] = true;
        why = NAMES[spotIndex] + " stayed " + (overs >= unders ? "long" : "short")
                + ". Efficiency stays " + num(ShotCalculator.EFFICIENCY) + ".";
        leaveForNext();
    }

    private void retestHood() {
        double hood = shot.hoodDeg;
        loadChoices();
        ShotCalculator.Shot next = untriedNear(hood);
        if (next == null) {
            next = firstFreshHood();
        }
        if (next == null) {
            settled[spotIndex] = true;
            leaveForNext();
            return;
        }
        command(next);
        phase = Phase.AIM;
    }

    private void skip() {
        if (confirmMode) {
            confirmMode = false;
            effChanged = false;
            pendingConfirm = -1;
            why = "Skipped the check. Efficiency stays " + num(ShotCalculator.EFFICIENCY) + ".";
        } else if (spotIndex >= 0) {
            settled[spotIndex] = true;
            why = "Skipped " + NAMES[spotIndex] + ".";
        }
        leaveForNext();
    }

    private void leaveForNext() {
        rest();
        if (!selectNext()) {
            return;
        }
        phase = Phase.SQUARE;
    }

    private void finish(String reason) {
        why = reason;
        phase = Phase.DONE;
        rest();
        saveTuning();
    }

    private ShotCalculator.Shot bracketHood() {
        bracketTight = false;
        double bestGap = 1e9;
        double mid = 0;
        boolean found = false;
        for (int i = 0; i < trialCount[spotIndex]; i++) {
            Trial over = trials[spotIndex][i];
            if (!sameEff(over) || !"OVER".equals(over.mark)) {
                continue;
            }
            for (int j = 0; j < trialCount[spotIndex]; j++) {
                Trial under = trials[spotIndex][j];
                if (!sameEff(under) || !"UNDER".equals(under.mark)) {
                    continue;
                }
                double gap = Math.abs(over.shot.hoodDeg - under.shot.hoodDeg);
                if (gap < bestGap) {
                    bestGap = gap;
                    mid = (over.shot.hoodDeg + under.shot.hoodDeg) / 2.0;
                    found = true;
                }
            }
        }
        if (!found) {
            return null;
        }
        ShotCalculator.Shot midShot = untriedNear(mid);
        if (bestGap < 2.5 || midShot == null) {
            bracketTight = true;
            return null;
        }
        return midShot;
    }

    private ShotCalculator.Shot firstFreshHood() {
        for (int i = 0; i < choices.length; i++) {
            if (!triedHood(choices[i].hoodDeg)) {
                return choices[i];
            }
        }
        return null;
    }

    private ShotCalculator.Shot untriedNear(double hood) {
        ShotCalculator.Shot best = null;
        double error = 999;
        for (int i = 0; i < choices.length; i++) {
            if (triedHood(choices[i].hoodDeg)) {
                continue;
            }
            double gap = Math.abs(choices[i].hoodDeg - hood);
            if (gap < error) {
                best = choices[i];
                error = gap;
            }
        }
        if (best == null || error > 4) {
            return null;
        }
        return best;
    }

    private ShotCalculator.Shot nearest(double hood) {
        ShotCalculator.Shot best = null;
        double error = 999;
        for (int i = 0; i < choices.length; i++) {
            double gap = Math.abs(choices[i].hoodDeg - hood);
            if (gap < error) {
                best = choices[i];
                error = gap;
            }
        }
        return best;
    }

    private boolean triedHood(double hood) {
        for (int i = 0; i < trialCount[spotIndex]; i++) {
            Trial trial = trials[spotIndex][i];
            if (sameEff(trial) && Math.abs(trial.shot.hoodDeg - hood) < 1.25) {
                return true;
            }
        }
        return false;
    }

    private int countMark(String mark) {
        int count = 0;
        for (int i = 0; i < trialCount[spotIndex]; i++) {
            Trial trial = trials[spotIndex][i];
            if (sameEff(trial) && mark.equals(trial.mark)) {
                count++;
            }
        }
        return count;
    }

    private boolean sameEff(Trial trial) {
        return Math.abs(trial.efficiency - ShotCalculator.EFFICIENCY) < 1e-6;
    }

    private int otherMake() {
        for (int i = 0; i < STOPS; i++) {
            if (i != spotIndex && madeHere[i]) {
                return i;
            }
        }
        return -1;
    }

    private boolean anyMake() {
        for (int i = 0; i < STOPS; i++) {
            if (madeHere[i]) {
                return true;
            }
        }
        return false;
    }

    private boolean bandMake(double minIn, double maxIn) {
        for (int i = 0; i < STOPS; i++) {
            if (madeHere[i] && spotDistanceIn[i] >= minIn && spotDistanceIn[i] < maxIn) {
                return true;
            }
        }
        return false;
    }

    private int makeCount() {
        int makes = 0;
        for (int i = 0; i < STOPS; i++) {
            if (madeHere[i]) {
                makes++;
            }
        }
        return makes;
    }

    private boolean covered() {
        return makeCount() >= 4
                && bandMake(0, CLOSE_IN)
                && bandMake(CLOSE_IN, FAR_IN)
                && bandMake(FAR_IN, 1000);
    }

    private String doneReason() {
        if (covered()) {
            return "Close, middle, and far went in across " + makeCount()
                    + " spots. Efficiency " + num(ShotCalculator.EFFICIENCY) + ".";
        }
        return "Stopped with " + makeCount()
                + " makes. Need a close, a middle, and a far shot. Efficiency "
                + num(ShotCalculator.EFFICIENCY) + ".";
    }

    private boolean stepEff(double delta) {
        double before = ShotCalculator.EFFICIENCY;
        double next = clamp(before + delta, effLo, effHi);
        if (Math.abs(next - before) < 1e-9) {
            return false;
        }
        effSnapshot = before;
        lastDelta = delta;
        ShotCalculator.EFFICIENCY = next;
        effChanged = true;
        effStepsThisVisit++;
        saveTuning();
        return true;
    }

    private String stepNote() {
        return "Efficiency " + num(effSnapshot) + " to " + num(ShotCalculator.EFFICIENCY) + ".";
    }

    private boolean nudgeFromThisSpot() {
        Trial made = null;
        Trial missed = null;
        for (int i = 0; i < trialCount[spotIndex]; i++) {
            Trial trial = trials[spotIndex][i];
            if (!sameEff(trial)) {
                continue;
            }
            if ("MADE".equals(trial.mark)) {
                made = trial;
            } else if (missed == null) {
                missed = trial;
            }
        }
        if (made == null || missed == null) {
            return false;
        }
        if (Double.isNaN(made.shot.seconds) || Double.isNaN(missed.shot.seconds)) {
            return false;
        }
        return stepWeight(made.shot, missed.shot);
    }

    private boolean stepWeight(ShotCalculator.Shot better, ShotCalculator.Shot worse) {
        double dTime = better.seconds - worse.seconds;
        double dFlight = better.flightSec - worse.flightSec;
        double dTol = better.tolerancePct - worse.tolerancePct;
        double dSpeed = better.speedMps - worse.speedMps;
        double[] benefit = {
                -dTime, dTime,
                -dFlight, dFlight,
                dTol, -dTol,
                -dSpeed, dSpeed
        };
        boolean[] used = new boolean[benefit.length];
        for (int attempt = 0; attempt < benefit.length; attempt++) {
            int best = -1;
            for (int i = 0; i < benefit.length; i++) {
                if (!used[i] && (best < 0 || benefit[i] > benefit[best])) {
                    best = i;
                }
            }
            if (best < 0 || benefit[best] <= 1e-6) {
                return false;
            }
            used[best] = true;
            if (applyWeight(best)) {
                return true;
            }
        }
        return false;
    }

    private boolean applyWeight(int nudge) {
        double before;
        double after;
        if (nudge == 0 || nudge == 1) {
            before = ShotCalculator.TIME_WEIGHT;
            after = clamp(before + (nudge == 0 ? WEIGHT_STEP : -WEIGHT_STEP), 0, 3);
            ShotCalculator.TIME_WEIGHT = after;
        } else if (nudge == 2 || nudge == 3) {
            before = ShotCalculator.FLIGHT_WEIGHT;
            after = clamp(before + (nudge == 2 ? WEIGHT_STEP : -WEIGHT_STEP), 0, 3);
            ShotCalculator.FLIGHT_WEIGHT = after;
        } else if (nudge == 4 || nudge == 5) {
            before = ShotCalculator.TOLERANCE_WEIGHT;
            after = clamp(before + (nudge == 4 ? WEIGHT_STEP : -WEIGHT_STEP), 0, 3);
            ShotCalculator.TOLERANCE_WEIGHT = after;
        } else {
            before = ShotCalculator.SPEED_BIAS;
            after = clamp(before + (nudge == 6 ? SPEED_STEP : -SPEED_STEP), -1, 1);
            ShotCalculator.SPEED_BIAS = after;
        }
        return Math.abs(after - before) > 1e-9;
    }

    private void loadChoices() {
        choices = ShotCalculator.shots(distanceMeters(), goalHeightM, commandedHood, currentRpm());
    }

    private void command(ShotCalculator.Shot next) {
        previousHood = commandedHood;
        commandedHood = next.hoodDeg;
        targetRpm = next.rpm;
        hoodCommandNanos = System.nanoTime();
        shot = next;
        feeding = false;
    }

    private void holdShot() {
        hoodServo.setPosition(ShotCalculator.hoodServoPosition(commandedHood));
        flywheel.setVelocity(targetRpm * RobotSpecs.FLYWHEEL_TICKS_PER_REV / 60.0);
    }

    private void feed() {
        boolean trigger = gamepad1.right_trigger > 0.5;
        if (!trigger) {
            feedArmed = true;
        }
        if (!feeding) {
            if (trigger && feedArmed && shot != null && hoodArrived()
                    && shot.isReady(commandedHood, currentRpm(), 1.0)) {
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

    private void trackHive() {
        if (yawMotor == null) {
            return;
        }
        double yaw = wrap(odometry.getHeading(AngleUnit.DEGREES) - hiveBearing());
        yawMotor.setTargetPosition((int) Math.round(yaw * RobotSpecs.TICKS_PER_REV / 360.0));
    }

    private double hiveBearing() {
        return Math.toDegrees(Math.atan2(
                hiveY - odometry.getY(DistanceUnit.INCH),
                hiveX - odometry.getX(DistanceUnit.INCH)));
    }

    private void follow(double targetX, double targetY, double faceDeg) {
        double x = odometry.getX(DistanceUnit.INCH);
        double y = odometry.getY(DistanceUnit.INCH);
        double heading = odometry.getHeading(AngleUnit.DEGREES);
        double dx = targetX - x;
        double dy = targetY - y;
        double headingRad = Math.toRadians(heading);
        double forward = dx * Math.cos(headingRad) + dy * Math.sin(headingRad);
        double right = dx * Math.sin(headingRad) - dy * Math.cos(headingRad);
        MecanumDriveTrainController.update(
                -axisPower(forward),
                -axisPower(right),
                -UtilMethods.clip(wrap(faceDeg - heading) * 0.012, -DRIVE_POWER, DRIVE_POWER));
    }

    private boolean arrived(double targetX, double targetY, double faceDeg) {
        double dx = targetX - odometry.getX(DistanceUnit.INCH);
        double dy = targetY - odometry.getY(DistanceUnit.INCH);
        return Math.hypot(dx, dy) <= ARRIVE_IN
                && Math.abs(wrap(faceDeg - odometry.getHeading(AngleUnit.DEGREES))) <= ARRIVE_DEG;
    }

    private boolean hoodArrived() {
        double wait = Math.abs(commandedHood - previousHood) / ShotCalculator.HOOD_DEG_PER_SEC;
        return (System.nanoTime() - hoodCommandNanos) / 1e9 >= wait;
    }

    private double currentRpm() {
        if (flywheel == null) {
            return 0;
        }
        return flywheel.getVelocity() * 60.0 / RobotSpecs.FLYWHEEL_TICKS_PER_REV;
    }

    private double distanceMeters() {
        return inchesToMeters(Math.hypot(
                hiveX - odometry.getX(DistanceUnit.INCH),
                hiveY - odometry.getY(DistanceUnit.INCH)));
    }

    private void rest() {
        if (flywheel != null) {
            flywheel.setVelocity(0);
        }
        feeding = false;
        SingleIntakeController.update(0);
        UptakeController.update(0);
    }

    private void draw() {
        TimedTelemetry.update();
        telemetry.addData("Doing", phase == Phase.SQUARE || phase == Phase.PRESS ? "Reset corner" : phase);
        if (spotIndex >= 0 && spotIndex < STOPS) {
            telemetry.addData("Spot", NAMES[spotIndex] + "  " + Math.round(spotDistanceIn[spotIndex]) + " in");
        }
        telemetry.addData("Shots", totalShots + " / " + SHOT_CAP);
        telemetry.addData("Hive in", "%.0f, %.0f  no camera", hiveX, hiveY);
        telemetry.addData("Turret yaw", "%.0f commanded  %.0f encoder",
                wrap(odometry.getHeading(AngleUnit.DEGREES) - hiveBearing()),
                yawMotor.getCurrentPosition() * 360.0 / RobotSpecs.TICKS_PER_REV);
        telemetry.addData("Pose in", "%.0f, %.0f  %.0f deg",
                odometry.getX(DistanceUnit.INCH),
                odometry.getY(DistanceUnit.INCH),
                odometry.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Efficiency", "%.2f", ShotCalculator.EFFICIENCY);
        telemetry.addData("Weights", "time %.2f  flight %.2f  tol %.2f  speed %.2f",
                ShotCalculator.TIME_WEIGHT, ShotCalculator.FLIGHT_WEIGHT,
                ShotCalculator.TOLERANCE_WEIGHT, ShotCalculator.SPEED_BIAS);
        telemetry.addLine("Drag stays 0");
        telemetry.addLine(why);
        if (phase == Phase.AIM || phase == Phase.MARK) {
            if (shot != null) {
                telemetry.addData("Hood", "%.1f deg", shot.hoodDeg);
                telemetry.addData("Target rpm", "%.0f", shot.rpm);
                telemetry.addData("Wheel rpm", "%.0f", currentRpm());
                telemetry.addData("Feed", hoodArrived() && shot.isReady(commandedHood, currentRpm(), 1.0) ? "ready" : "wait");
            }
            if (phase == Phase.AIM) {
                telemetry.addLine("Right trigger feeds one ball");
            } else {
                telemetry.addLine("X made, circle long, square short");
            }
            telemetry.addLine("Left bumper skips this spot");
        } else if (phase == Phase.DONE) {
            telemetry.addData("Makes", makeCount() + "  need close, middle, and far");
            telemetry.addData("Close", bandMake(0, CLOSE_IN) ? "yes" : "no");
            telemetry.addData("Middle", bandMake(CLOSE_IN, FAR_IN) ? "yes" : "no");
            telemetry.addData("Far", bandMake(FAR_IN, 1000) ? "yes" : "no");
            telemetry.addLine("Saved. Run it again to keep going from these numbers.");
        }
        telemetry.update();
    }

    private void openLog() {
        try {
            logFile = new File(hardwareMap.appContext.getFilesDir(), "shot-adaptive.csv");
            boolean fresh = !logFile.exists() || logFile.length() == 0;
            log = new BufferedWriter(new FileWriter(logFile, true));
            if (fresh) {
                log.write("millis,spot,distanceIn,hoodDeg,rpm,efficiency,timeWeight,flightWeight,toleranceWeight,speedBias,mark");
                log.newLine();
                log.flush();
            }
        } catch (IOException e) {
            log = null;
        }
    }

    private void writeRow(String mark) {
        if (log == null || shot == null || spotIndex < 0) {
            return;
        }
        try {
            log.write(System.currentTimeMillis()
                    + "," + NAMES[spotIndex]
                    + "," + spotDistanceIn[spotIndex]
                    + "," + shot.hoodDeg
                    + "," + shot.rpm
                    + "," + ShotCalculator.EFFICIENCY
                    + "," + ShotCalculator.TIME_WEIGHT
                    + "," + ShotCalculator.FLIGHT_WEIGHT
                    + "," + ShotCalculator.TOLERANCE_WEIGHT
                    + "," + ShotCalculator.SPEED_BIAS
                    + "," + mark);
            log.newLine();
            log.flush();
        } catch (IOException ignored) {
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
            why = "Tune error: " + e.getMessage();
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

    private static String num(double value) {
        return String.format("%.2f", value);
    }
}

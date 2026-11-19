package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.robotControllers.MecanumDriveTrainController;
import org.firstinspires.ftc.teamcode.robotControllers.TurretController;
import org.firstinspires.ftc.teamcode.sensors.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.utils.ShotCalculator;
import org.firstinspires.ftc.teamcode.utils.ShotUtil;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;
import org.firstinspires.ftc.teamcode.utils.UtilMethods;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Drive Tracking Test")
public class DriveTrackingTest extends OpMode {
    static final double START_HEADING_DEG = 90; // deg
    static final double MIN_STEP = 0.1; // deg
    static final double RPM_BUMP = 100;
    static final String LOG_NAME = "drive-tracking-shots.csv";

    GoBildaOdometry odometry;
    List<LynxModule> hubs;
    boolean flywheelOn = true;
    boolean movingComp;
    boolean needZero = true;
    boolean rightDown;
    boolean leftDown;

    double searchOffset; // deg
    double step = TurretConstants.TRIM_STEP_START; // deg
    int lastDirection;
    String lastResult = "";
    double hoodBase = 15; // deg
    double hoodCommand = 15; // deg
    double flywheelRpm = TurretConstants.RPM_A;
    int makes;
    int attempts;

    double a0;
    double a1;
    double a2;
    final ArrayList<double[]> spots = new ArrayList<>(); // R m, |beta| deg, totalTrim deg

    @Override
    public void init() {
        Timer.init();
        TimedTelemetry.init(telemetry);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        ShotCalculator.loadTuning(new File(hardwareMap.appContext.getFilesDir(), "shot-tuning.txt"));
        TurretController.init(hardwareMap);
        MecanumDriveTrainController.init(hardwareMap);
        ShotUtil.reset();
        odometry = new GoBildaOdometry(hardwareMap, HardwareNames.ODOMETRY_NAME);
        zeroPose();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        Timer.update();

        if (gamepad1.dpadUpWasPressed()) flywheelRpm += RPM_BUMP;
        if (gamepad1.dpadDownWasPressed()) flywheelRpm -= RPM_BUMP;
        flywheelRpm = Math.max(0, Math.min(TurretConstants.FLYWHEEL_RPM_MAX, flywheelRpm));
        if (gamepad1.squareWasPressed()) flywheelOn = !flywheelOn;
        if (gamepad1.dpadLeftWasPressed()) movingComp = !movingComp;
        if (gamepad1.dpadRightWasPressed()) zeroPose();

        odometry.update();
        if (needZero && odometry.getStatus() != GoBildaPinpointDriver.DeviceStatus.NOT_READY) {
            zeroPose();
            needZero = false;
        }
        Pose2D pose = odometry.getPose();
        double x = pose.getX(DistanceUnit.METER); // m
        double y = pose.getY(DistanceUnit.METER); // m
        double heading = pose.getHeading(AngleUnit.DEGREES); // deg
        double vx = odometry.getVelX(DistanceUnit.METER); // m/s
        double vy = odometry.getVelY(DistanceUnit.METER); // m/s

        MecanumDriveTrainController.update(
                UtilMethods.expo(gamepad1.left_stick_y, 2),
                UtilMethods.expo(-gamepad1.left_stick_x, 2),
                UtilMethods.expo(-gamepad1.right_stick_x, 2));

        ShotUtil.ShotResult r = ShotUtil.compute(x, y, vx, vy, flywheelRpm, movingComp);
        double relX = x - TurretConstants.HIVE_X;
        double relY = y - TurretConstants.HIVE_Y;
        double beta = Math.toDegrees(Math.atan2(Math.abs(relX), Math.abs(relY))); // deg

        boolean over = gamepad1.right_trigger > 0.5;
        boolean under = gamepad1.left_trigger > 0.5;
        if (over && !rightDown) adjust(r, true, false, false);
        if (under && !leftDown) adjust(r, false, false, false);
        rightDown = over;
        leftDown = under;
        if (gamepad1.circleWasPressed()) adjust(r, true, true, true);
        if (gamepad1.triangleWasPressed()) adjust(r, false, true, true);
        if (gamepad1.crossWasPressed()) lock(r, x, y, beta);
        if (gamepad1.leftBumperWasPressed()) undo();
        if (gamepad1.rightBumperWasPressed()) resetSearch();
        if (gamepad1.backWasPressed()) {
            makes = 0;
            attempts = 0;
            lastResult = "hits cleared";
        }

        if (r.haveHood) hoodBase = r.hoodDeg;
        double hoodSent = hoodBase + fitTrim(r.range, beta) + searchOffset; // deg
        TurretController.setRpm(flywheelOn ? flywheelRpm : 0);
        TurretController.setHood(hoodSent);
        hoodCommand = hoodSent;
        TurretController.setYawFieldCentric(
                r.yawFieldDeg,
                heading,
                (movingComp && r.valid) ? r.vPerp : 0,
                r.ballSpeed,
                r.thetaDeg,
                TurretConstants.W_T);
        ShotUtil.Ready ready = ShotUtil.readyToShoot(
                r,
                TurretController.getYaw(),
                TurretController.getTargetYaw(),
                hoodCommand,
                TurretController.getRpm(),
                movingComp,
                vx,
                vy);
        TurretController.update();

        if (TurretConstants.HIVE_X == 0 && TurretConstants.HIVE_Y == 0) {
            TimedTelemetry.addLine("HIVE_X and HIVE_Y are 0");
        }
        TimedTelemetry.addData("x", "%.2f m", 1, x);
        TimedTelemetry.addData("y", "%.2f m", 1, y);
        TimedTelemetry.addData("heading", "%.1f deg", 1, heading);
        TimedTelemetry.addData("odo", odometry.getStatus());
        TimedTelemetry.addData("vx", "%.2f m/s", 1, vx);
        TimedTelemetry.addData("vy", "%.2f m/s", 1, vy);
        TimedTelemetry.addData("range", "%.2f m", 1, r.range);
        TimedTelemetry.addData("beta", "%.1f deg", 1, beta);
        TimedTelemetry.addData("mode", r.behind ? "BEHIND" : "FRONT");
        TimedTelemetry.addData("arc", r.steepArc ? "STEEP" : "SHALLOW");
        TimedTelemetry.addData("theta", "%.1f deg", 1, r.thetaDeg);
        TimedTelemetry.addData("phi", "%.1f deg", 1, r.phiDeg);
        TimedTelemetry.addData("hood", "%.1f deg", 1, r.hoodDeg);
        TimedTelemetry.addData("hood sent", "%.1f deg", 1, hoodSent);
        TimedTelemetry.addData("rpm target", "%.0f", 1, flywheelRpm);
        TimedTelemetry.addData("rpm", "%.0f", 1, TurretController.getRpm());
        TimedTelemetry.addData("target yaw", "%.1f deg", 1, TurretController.getTargetYaw());
        TimedTelemetry.addData("actual yaw", "%.1f deg", 1, TurretController.getYaw());
        TimedTelemetry.addData("valid", r.valid + " " + r.reason);
        TimedTelemetry.addData("ready", ready.ready + " " + ready.reason);
        TimedTelemetry.addData("flywheelOn", flywheelOn);
        TimedTelemetry.addData("movingComp", movingComp);
        TimedTelemetry.addData("searchOffset", "%.2f deg", 1, searchOffset);
        TimedTelemetry.addData("step", "%.2f deg", 1, step);
        TimedTelemetry.addData("last", lastResult);
        TimedTelemetry.addData("spots", spots.size());
        TimedTelemetry.addData("fit", "a0 %.2f  a1 %.2f  a2 %.3f", 1, a0, a1, a2);
        double pct = attempts == 0 ? 0 : 100.0 * makes / attempts;
        TimedTelemetry.addData("hits", makes + " / " + attempts + "  " + Math.round(pct) + "%");
        TimedTelemetry.addLine("dpadU rpm+ | dpadD rpm- | □ fly | dpadL move | dpadR zero | RT over | LT under | O top | △ bottom | X lock | LB undo | RB reset");
        TimedTelemetry.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        MecanumDriveTrainController.stop();
        TurretController.stop();
    }

    private void zeroPose() {
        odometry.setPose(new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, START_HEADING_DEG));
    }

    private void adjust(ShotUtil.ShotResult r, boolean shorter, boolean half, boolean make) {
        if (r.behind) {
            lastResult = "behind: not tuned";
            return;
        }
        int direction = shorter ? 1 : -1;
        if (lastDirection != 0 && lastDirection != direction) step = Math.max(MIN_STEP, step / 2.0);
        double move = half ? step / 2.0 : step; // deg
        if (shorter) searchOffset += TurretConstants.TRIM_SHORTER_SIGN * move;
        else searchOffset -= TurretConstants.TRIM_SHORTER_SIGN * move;
        lastDirection = direction;
        attempts++;
        if (make) makes++;
        lastResult = shorter ? (half ? "in top" : "over") : (half ? "in bottom" : "under");
    }

    private void lock(ShotUtil.ShotResult r, double x, double y, double beta) {
        if (r.behind) {
            lastResult = "behind: not tuned";
            return;
        }
        double total = fitTrim(r.range, beta) + searchOffset; // deg
        double hoodSent = r.hoodDeg + total; // deg
        spots.add(new double[] {r.range, Math.abs(beta), total});
        appendLog(x, y, r.range, beta, TurretController.getRpm(), hoodSent, searchOffset, total);
        refit();
        searchOffset = 0;
        step = TurretConstants.TRIM_STEP_START;
        lastDirection = 0;
        makes++;
        attempts++;
        if (!"log failed".equals(lastResult)) lastResult = "locked";
    }

    private void undo() {
        if (spots.isEmpty()) {
            lastResult = "nothing to undo";
            return;
        }
        spots.remove(spots.size() - 1);
        refit();
        lastResult = "undone";
    }

    private void resetSearch() {
        searchOffset = 0;
        step = TurretConstants.TRIM_STEP_START;
        lastDirection = 0;
        lastResult = "reset search";
    }

    private double fitTrim(double range, double betaDeg) {
        return a0 + a1 * range + a2 * Math.abs(betaDeg); // deg
    }

    private void refit() {
        int n = spots.size();
        if (n == 0) {
            a0 = 0;
            a1 = 0;
            a2 = 0;
            return;
        }
        if (n < 3) {
            double sum = 0;
            for (double[] spot : spots) sum += spot[2];
            a0 = sum / n;
            a1 = 0;
            a2 = 0;
            return;
        }
        double sumR = 0, sumB = 0, sumT = 0, sumR2 = 0, sumB2 = 0, sumRB = 0, sumRT = 0, sumBT = 0;
        for (double[] spot : spots) {
            double range = spot[0];
            double b = spot[1];
            double trim = spot[2];
            sumR += range;
            sumB += b;
            sumT += trim;
            sumR2 += range * range;
            sumB2 += b * b;
            sumRB += range * b;
            sumRT += range * trim;
            sumBT += b * trim;
        }
        double[][] rows = {
                {n, sumR, sumB, sumT},
                {sumR, sumR2, sumRB, sumRT},
                {sumB, sumRB, sumB2, sumBT}
        };
        if (!solve(rows)) {
            a0 = sumT / n;
            a1 = 0;
            a2 = 0;
        }
    }

    private boolean solve(double[][] rows) {
        for (int col = 0; col < 3; col++) {
            int pivot = col;
            for (int row = col + 1; row < 3; row++) {
                if (Math.abs(rows[row][col]) > Math.abs(rows[pivot][col])) pivot = row;
            }
            double[] swap = rows[col];
            rows[col] = rows[pivot];
            rows[pivot] = swap;
            if (Math.abs(rows[col][col]) < 1e-9) return false;
            for (int row = col + 1; row < 3; row++) {
                double factor = rows[row][col] / rows[col][col];
                for (int k = col; k < 4; k++) rows[row][k] -= factor * rows[col][k];
            }
        }
        double[] answer = new double[3];
        for (int row = 2; row >= 0; row--) {
            double value = rows[row][3];
            for (int k = row + 1; k < 3; k++) value -= rows[row][k] * answer[k];
            answer[row] = value / rows[row][row];
        }
        a0 = answer[0];
        a1 = answer[1];
        a2 = answer[2];
        return true;
    }

    private void appendLog(double x, double y, double range, double beta, double rpm, double hood, double offset, double total) {
        try {
            File file = AppUtil.getInstance().getSettingsFile(LOG_NAME);
            String prior = file.exists() ? ReadWriteFile.readFile(file) : "";
            if (prior.isEmpty()) prior = "time,x,y,R,beta,rpm,hood,searchOffset,totalTrim\n";
            else if (!prior.endsWith("\n")) prior += "\n";
            ReadWriteFile.writeFile(file, prior
                    + System.currentTimeMillis() + ","
                    + x + "," + y + "," + range + "," + beta + ","
                    + rpm + "," + hood + "," + offset + "," + total + "\n");
        } catch (RuntimeException e) {
            lastResult = "log failed";
        }
    }
}

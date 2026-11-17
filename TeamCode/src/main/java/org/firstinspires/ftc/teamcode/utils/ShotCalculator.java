package org.firstinspires.ftc.teamcode.utils;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Comparator;

public class ShotCalculator {
    public static double MIN_HOOD_DEG = 5;
    public static double MAX_HOOD_DEG = 25;
    public static double HOOD_STEP_DEG = 0.5;
    public static double LAUNCH_SUM_DEG = 90;

    public static final double G = 9.80665;

    public static final double FLYWHEEL_DIAMETER_MM = 90;
    public static final double POLLEN_DIAMETER_IN = 2.8;
    public static final double COMPRESSION_MM = 5;

    public static final double FLYWHEEL_DIAMETER_M = FLYWHEEL_DIAMETER_MM / 1000.0;
    public static final double POLLEN_DIAMETER_M = POLLEN_DIAMETER_IN * 0.0254;
    public static final double COMPRESSION_M = COMPRESSION_MM / 1000.0;
    public static final double BALL_DIAMETER_M = POLLEN_DIAMETER_M;

    public static double EFFICIENCY = 0.5;
    public static double MAX_RPM = 5500;
    public static double HOOD_DEG_PER_SEC = 40;
    public static double SPINUP_RPM_PER_SEC = 3000;
    public static double SPINDOWN_RPM_PER_SEC = 600;

    public static double SHOOTER_AHEAD_OF_CAMERA_M = 0;
    public static double SHOOTER_ABOVE_CAMERA_M = 0;
    public static double SHOOTER_RIGHT_OF_CAMERA_M = 0;
    public static double OPENING_AHEAD_OF_TAG_M = 0;
    public static double OPENING_ABOVE_TAG_M = 0;

    public static double OPENING_M = 0.25;
    public static double OPENING_TILT_DEG = 30;
    public static double DRAG_PER_M = 0;

    public static double TIME_WEIGHT = 1;
    public static double FLIGHT_WEIGHT = 0.5;
    public static double TOLERANCE_WEIGHT = 1.0;
    public static double SPEED_BIAS = 0;
    public static double READY_FRACTION = 0.5;
    public static double READY_RPM_CAP_PCT = 1;
    public static double HOOD_SERVO_AT_MIN = 0.0074074074;
    public static double HOOD_SERVO_AT_MAX = 0.148148148;

    private static final double SLOPE_EPS = 1e-9;
    private static final double DRAG_DT = 0.002;
    private static final int MAX_HOOD_STEPS = 500;

    public static class Arc {
        public final double launchDeg;
        public final double speedMps;
        public final double landingMeters;

        public Arc(double launchDeg, double speedMps, double landingMeters) {
            this.launchDeg = launchDeg;
            this.speedMps = speedMps;
            this.landingMeters = landingMeters;
        }

        public double heightAt(double xMeters) {
            if (xMeters < 0 || speedMps <= 0) {
                return Double.NaN;
            }
            if (DRAG_PER_M > 0) {
                Flight flight = flyUntilX(speedMps, launchDeg, xMeters);
                if (flight == null) {
                    return Double.NaN;
                }
                return flight.y;
            }
            double theta = Math.toRadians(launchDeg);
            double cos = Math.cos(theta);
            if (Math.abs(cos) < SLOPE_EPS) {
                return Double.NaN;
            }
            return xMeters * Math.tan(theta) - G * xMeters * xMeters / (2 * speedMps * speedMps * cos * cos);
        }

        public double rangeAtHeight(double heightMeters) {
            return crossingAtHeight(heightMeters, true);
        }

        public double risingRangeAtHeight(double heightMeters) {
            return crossingAtHeight(heightMeters, false);
        }

        private double crossingAtHeight(double heightMeters, boolean descending) {
            if (speedMps <= 0 || Double.isNaN(heightMeters)) {
                return Double.NaN;
            }
            if (DRAG_PER_M > 0) {
                return landingWithDrag(speedMps, launchDeg, heightMeters, descending);
            }
            double theta = Math.toRadians(launchDeg);
            double cos = Math.cos(theta);
            if (Math.abs(cos) < SLOPE_EPS) {
                return Double.NaN;
            }
            double a = G / (2 * speedMps * speedMps * cos * cos);
            double b = -Math.tan(theta);
            double disc = b * b - 4 * a * heightMeters;
            if (disc < 0 || a == 0) {
                return Double.NaN;
            }
            double sqrt = Math.sqrt(disc);
            double rising = (-b - sqrt) / (2 * a);
            double falling = (-b + sqrt) / (2 * a);
            double x = descending ? falling : rising;
            if (x <= 1e-6) {
                return Double.NaN;
            }
            return x;
        }
    }

    public static class Shot {
        public final double hoodDeg;
        public final double launchDeg;
        public final double speedMps;
        public final double seconds;
        public final double tolerancePct;
        public final double rpm;
        public final double flightSec;

        public Shot(double hoodDeg, double launchDeg, double speedMps, double seconds, double tolerancePct, double rpm, double flightSec) {
            this.hoodDeg = hoodDeg;
            this.launchDeg = launchDeg;
            this.speedMps = speedMps;
            this.seconds = seconds;
            this.tolerancePct = tolerancePct;
            this.rpm = rpm;
            this.flightSec = flightSec;
        }

        public boolean isReady(double currentHoodDeg, double currentRpm, double hoodTolDeg) {
            if (Double.isNaN(tolerancePct) || rpm <= 0 || hoodTolDeg < 0) {
                return false;
            }
            if (Math.abs(currentHoodDeg - hoodDeg) > hoodTolDeg) {
                return false;
            }
            if (READY_RPM_CAP_PCT <= 0) {
                return false;
            }
            double allowedPct = Math.min(tolerancePct * READY_FRACTION, READY_RPM_CAP_PCT);
            double rpmWindow = rpm * allowedPct / 100.0;
            return Math.abs(currentRpm - rpm) <= rpmWindow;
        }
    }

    public static double launchAngle(double hoodDeg) {
        return LAUNCH_SUM_DEG - hoodDeg;
    }

    public static double hoodServoPosition(double hoodDeg) {
        double span = MAX_HOOD_DEG - MIN_HOOD_DEG;
        if (span <= 0) {
            return HOOD_SERVO_AT_MIN;
        }
        double t = (hoodDeg - MIN_HOOD_DEG) / span;
        if (t < 0) {
            t = 0;
        }
        if (t > 1) {
            t = 1;
        }
        return HOOD_SERVO_AT_MIN + t * (HOOD_SERVO_AT_MAX - HOOD_SERVO_AT_MIN);
    }

    public static void loadTuning(File file) {
        if (file == null || !file.isFile()) {
            return;
        }
        BufferedReader reader = null;
        try {
            reader = new BufferedReader(new FileReader(file));
            String line;
            while ((line = reader.readLine()) != null) {
                int split = line.indexOf('=');
                if (split <= 0) {
                    continue;
                }
                applyTuning(line.substring(0, split).trim(), Double.parseDouble(line.substring(split + 1).trim()));
            }
        } catch (IOException ignored) {
        } catch (NumberFormatException ignored) {
        } finally {
            if (reader != null) {
                try {
                    reader.close();
                } catch (IOException ignored) {
                }
            }
        }
    }

    public static void saveTuning(File file) throws IOException {
        BufferedWriter writer = new BufferedWriter(new FileWriter(file, false));
        try {
            writer.write("EFFICIENCY=" + EFFICIENCY);
            writer.newLine();
            writer.write("HOOD_SERVO_AT_MIN=" + HOOD_SERVO_AT_MIN);
            writer.newLine();
            writer.write("HOOD_SERVO_AT_MAX=" + HOOD_SERVO_AT_MAX);
            writer.newLine();
            writer.flush();
        } finally {
            writer.close();
        }
    }

    private static void applyTuning(String key, double value) {
        if ("EFFICIENCY".equals(key) && value > 0 && value < 1) {
            EFFICIENCY = value;
        } else if ("HOOD_SERVO_AT_MIN".equals(key)) {
            HOOD_SERVO_AT_MIN = value;
        } else if ("HOOD_SERVO_AT_MAX".equals(key)) {
            HOOD_SERVO_AT_MAX = value;
        }
    }

    public static double rpmForSpeed(double ballMps) {
        if (ballMps < 0 || EFFICIENCY <= 0 || FLYWHEEL_DIAMETER_M <= 0) {
            return Double.NaN;
        }
        return ballMps * 60.0 / (EFFICIENCY * Math.PI * FLYWHEEL_DIAMETER_M);
    }

    public static double speedForRpm(double rpm) {
        if (rpm < 0 || EFFICIENCY <= 0 || FLYWHEEL_DIAMETER_M <= 0) {
            return Double.NaN;
        }
        return rpm * EFFICIENCY * Math.PI * FLYWHEEL_DIAMETER_M / 60.0;
    }

    public static double shooterHorizontal(double rangeMeters, double elevationDeg) {
        return shooterHorizontal(rangeMeters, elevationDeg, 0);
    }

    public static double shooterHorizontal(double rangeMeters, double elevationDeg, double bearingDeg) {
        if (bad(rangeMeters) || bad(elevationDeg) || bad(bearingDeg) || rangeMeters <= 0) {
            return Double.NaN;
        }
        double ground = rangeMeters * Math.cos(Math.toRadians(elevationDeg));
        double bearing = Math.toRadians(bearingDeg);
        double dirX = Math.sin(bearing);
        double dirZ = Math.cos(bearing);
        double along = openingOffsetX();
        double dx = (ground + along) * dirX - SHOOTER_RIGHT_OF_CAMERA_M;
        double dz = (ground + along) * dirZ - SHOOTER_AHEAD_OF_CAMERA_M;
        double forward = dx * dirX + dz * dirZ;
        if (forward <= SLOPE_EPS) {
            return Double.NaN;
        }
        return Math.hypot(dx, dz);
    }

    public static double shooterHeight(double rangeMeters, double elevationDeg) {
        return shooterHeight(rangeMeters, elevationDeg, 0);
    }

    public static double shooterHeight(double rangeMeters, double elevationDeg, double bearingDeg) {
        if (bad(rangeMeters) || bad(elevationDeg) || bad(bearingDeg) || rangeMeters <= 0) {
            return Double.NaN;
        }
        return rangeMeters * Math.sin(Math.toRadians(elevationDeg)) - SHOOTER_ABOVE_CAMERA_M + openingOffsetY();
    }

    public static Arc arc(double launchDeg, double speedMps) {
        if (bad(launchDeg) || bad(speedMps) || speedMps <= 0) {
            return new Arc(launchDeg, speedMps, Double.NaN);
        }
        double landing;
        if (DRAG_PER_M > 0) {
            landing = landingWithDrag(speedMps, launchDeg, 0, true);
        } else {
            double theta = Math.toRadians(launchDeg);
            double sin2 = Math.sin(2 * theta);
            landing = sin2 > 0 ? speedMps * speedMps * sin2 / G : Double.NaN;
        }
        return new Arc(launchDeg, speedMps, landing);
    }

    public static double speedForShot(double horizontalMeters, double heightMeters, double launchDeg) {
        if (bad(horizontalMeters) || bad(heightMeters) || bad(launchDeg) || horizontalMeters <= 0) {
            return Double.NaN;
        }
        double theta = Math.toRadians(launchDeg);
        double cos = Math.cos(theta);
        if (Math.abs(cos) < SLOPE_EPS) {
            return Double.NaN;
        }
        if (DRAG_PER_M > 0) {
            return speedForShotWithDrag(horizontalMeters, heightMeters, launchDeg);
        }
        double denom = horizontalMeters * Math.sin(2 * theta) - 2 * heightMeters * cos * cos;
        if (denom <= 0) {
            return Double.NaN;
        }
        return Math.sqrt(G * horizontalMeters * horizontalMeters / denom);
    }

    public static double tolerancePct(double horizontalMeters, double heightMeters, double launchDeg) {
        if (bad(horizontalMeters) || bad(heightMeters) || bad(launchDeg) || horizontalMeters <= 0) {
            return Double.NaN;
        }
        double speed = speedForShot(horizontalMeters, heightMeters, launchDeg);
        if (Double.isNaN(speed) || speed <= 0 || !hitsOpening(speed, launchDeg, horizontalMeters, heightMeters)) {
            return Double.NaN;
        }
        double slower = speedBound(speed, launchDeg, horizontalMeters, heightMeters, false);
        double faster = speedBound(speed, launchDeg, horizontalMeters, heightMeters, true);
        if (Double.isNaN(slower) || Double.isNaN(faster)) {
            return Double.NaN;
        }
        return 100.0 * Math.min(speed - slower, faster - speed) / speed;
    }

    public static double timeToReady(double currentHoodDeg, double currentRpm, double targetHoodDeg, double targetRpm) {
        if (bad(currentHoodDeg) || bad(currentRpm) || bad(targetHoodDeg) || bad(targetRpm)) {
            return Double.NaN;
        }
        if (HOOD_DEG_PER_SEC <= 0 || SPINUP_RPM_PER_SEC <= 0 || SPINDOWN_RPM_PER_SEC <= 0) {
            return Double.NaN;
        }
        double hoodTime = Math.abs(targetHoodDeg - currentHoodDeg) / HOOD_DEG_PER_SEC;
        double rpmDelta = targetRpm - currentRpm;
        double wheelTime = rpmDelta >= 0
                ? rpmDelta / SPINUP_RPM_PER_SEC
                : -rpmDelta / SPINDOWN_RPM_PER_SEC;
        return Math.max(hoodTime, wheelTime);
    }

    public static Shot[] shots(double horizontalMeters, double heightMeters, double currentHoodDeg, double currentRpm) {
        if (!canSearch(horizontalMeters, heightMeters, currentHoodDeg, currentRpm)) {
            return new Shot[0];
        }
        int stepCount = (int) Math.floor((MAX_HOOD_DEG - MIN_HOOD_DEG) / HOOD_STEP_DEG + 1e-9);
        if (stepCount < 0 || stepCount > MAX_HOOD_STEPS) {
            return new Shot[0];
        }
        ArrayList<Shot> found = new ArrayList<Shot>();
        for (int i = 0; i <= stepCount; i++) {
            double hood = MIN_HOOD_DEG + i * HOOD_STEP_DEG;
            if (hood > MAX_HOOD_DEG + 1e-6) {
                break;
            }
            addShot(found, hood, horizontalMeters, heightMeters, currentHoodDeg, currentRpm);
        }
        double lastStep = MIN_HOOD_DEG + stepCount * HOOD_STEP_DEG;
        if (lastStep < MAX_HOOD_DEG - 1e-6) {
            addShot(found, MAX_HOOD_DEG, horizontalMeters, heightMeters, currentHoodDeg, currentRpm);
        }
        found.sort(Comparator.comparingDouble(new java.util.function.ToDoubleFunction<Shot>() {
            @Override
            public double applyAsDouble(Shot shot) {
                return score(shot);
            }
        }));
        return found.toArray(new Shot[found.size()]);
    }

    public static Shot[] shotsFromRange(double hypotenuseMeters, double angleBetweenDeg, double currentHoodDeg, double currentRpm) {
        return shotsFromRange(hypotenuseMeters, angleBetweenDeg, 0, currentHoodDeg, currentRpm);
    }

    public static Shot[] shotsFromRange(double hypotenuseMeters, double angleBetweenDeg, double bearingDeg, double currentHoodDeg, double currentRpm) {
        double horizontal = shooterHorizontal(hypotenuseMeters, angleBetweenDeg, bearingDeg);
        double height = shooterHeight(hypotenuseMeters, angleBetweenDeg, bearingDeg);
        return shots(horizontal, height, currentHoodDeg, currentRpm);
    }

    public static Shot best(double horizontalMeters, double heightMeters, double currentHoodDeg, double currentRpm) {
        Shot[] all = shots(horizontalMeters, heightMeters, currentHoodDeg, currentRpm);
        if (all.length == 0) {
            return null;
        }
        return all[0];
    }

    public static Shot bestFromRange(double hypotenuseMeters, double angleBetweenDeg, double currentHoodDeg, double currentRpm) {
        return bestFromRange(hypotenuseMeters, angleBetweenDeg, 0, currentHoodDeg, currentRpm);
    }

    public static Shot bestFromRange(double hypotenuseMeters, double angleBetweenDeg, double bearingDeg, double currentHoodDeg, double currentRpm) {
        Shot[] all = shotsFromRange(hypotenuseMeters, angleBetweenDeg, bearingDeg, currentHoodDeg, currentRpm);
        if (all.length == 0) {
            return null;
        }
        return all[0];
    }

    private static void addShot(ArrayList<Shot> found, double hood, double horizontalMeters, double heightMeters, double currentHoodDeg, double currentRpm) {
        double launch = launchAngle(hood);
        double speed = speedForShot(horizontalMeters, heightMeters, launch);
        if (Double.isNaN(speed) || speed <= 0) {
            return;
        }
        double rpm = rpmForSpeed(speed);
        if (Double.isNaN(rpm) || rpm <= 0 || rpm > MAX_RPM) {
            return;
        }
        double tolerance = tolerancePct(horizontalMeters, heightMeters, launch);
        if (Double.isNaN(tolerance)) {
            return;
        }
        double flightSec = flightTime(horizontalMeters, launch, speed);
        if (Double.isNaN(flightSec) || flightSec <= 0) {
            return;
        }
        double seconds = timeToReady(currentHoodDeg, currentRpm, hood, rpm);
        if (Double.isNaN(seconds) || seconds < 0) {
            return;
        }
        found.add(new Shot(hood, launch, speed, seconds, tolerance, rpm, flightSec));
    }

    private static double score(Shot shot) {
        return TIME_WEIGHT * shot.seconds
                + FLIGHT_WEIGHT * shot.flightSec
                - TOLERANCE_WEIGHT * shot.tolerancePct
                + SPEED_BIAS * shot.speedMps;
    }

    private static boolean canSearch(double horizontalMeters, double heightMeters, double currentHoodDeg, double currentRpm) {
        if (bad(horizontalMeters) || bad(heightMeters) || bad(currentHoodDeg) || bad(currentRpm)) {
            return false;
        }
        if (horizontalMeters <= 0 || currentRpm < 0 || HOOD_STEP_DEG <= 0 || MAX_RPM <= 0) {
            return false;
        }
        return MAX_HOOD_DEG + 1e-9 >= MIN_HOOD_DEG;
    }

    private static double flightTime(double horizontalMeters, double launchDeg, double speedMps) {
        if (speedMps <= 0) {
            return Double.NaN;
        }
        if (DRAG_PER_M > 0) {
            Flight flight = flyUntilX(speedMps, launchDeg, horizontalMeters);
            if (flight == null || flight.vy >= 0) {
                return Double.NaN;
            }
            return flight.t;
        }
        double theta = Math.toRadians(launchDeg);
        double cos = Math.cos(theta);
        if (Math.abs(cos) < SLOPE_EPS) {
            return Double.NaN;
        }
        double arrivalVy = speedMps * Math.sin(theta) - G * horizontalMeters / (speedMps * cos);
        if (arrivalVy >= 0) {
            return Double.NaN;
        }
        return horizontalMeters / (speedMps * cos);
    }

    private static double speedForShotWithDrag(double horizontalMeters, double heightMeters, double launchDeg) {
        double hi = speedForRpm(MAX_RPM);
        if (Double.isNaN(hi) || hi <= 0) {
            return Double.NaN;
        }
        Flight atMax = flyUntilX(hi, launchDeg, horizontalMeters);
        if (atMax == null || atMax.y < heightMeters) {
            return Double.NaN;
        }
        double lo = 0;
        for (int i = 0; i < 48; i++) {
            double mid = (lo + hi) / 2.0;
            Flight flight = flyUntilX(mid, launchDeg, horizontalMeters);
            if (flight != null && flight.y >= heightMeters) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        Flight check = flyUntilX(hi, launchDeg, horizontalMeters);
        if (check == null || check.y < heightMeters) {
            return Double.NaN;
        }
        return hi;
    }

    private static double landingWithDrag(double speedMps, double launchDeg, double heightMeters, boolean descending) {
        double theta = Math.toRadians(launchDeg);
        double vx = speedMps * Math.cos(theta);
        double vy = speedMps * Math.sin(theta);
        double x = 0;
        double y = 0;
        double dt = DRAG_DT;
        double[] next = new double[4];
        boolean leftStart = false;
        boolean reachedPlane = false;
        for (int i = 0; i < 8000; i++) {
            heunStep(x, y, vx, vy, dt, next);
            double nextX = next[0];
            double nextY = next[1];
            double nextVx = next[2];
            double nextVy = next[3];
            if (y > 0.02 || nextY > 0.02) {
                leftStart = true;
            }
            if (leftStart && y >= heightMeters) {
                reachedPlane = true;
            }
            boolean falling = vy < 0 || nextVy < 0;
            boolean rising = vy > 0 || nextVy > 0;
            boolean downCross = leftStart && y >= heightMeters && nextY < heightMeters && falling;
            boolean upCross = leftStart && y < heightMeters && nextY >= heightMeters && rising;
            if ((descending && downCross) || (!descending && upCross)) {
                double frac = (y - heightMeters) / (y - nextY);
                return x + (nextX - x) * frac;
            }
            vx = nextVx;
            vy = nextVy;
            x = nextX;
            y = nextY;
            if (reachedPlane && falling && y < heightMeters - 1) {
                return Double.NaN;
            }
        }
        return Double.NaN;
    }

    private static double openingOffsetX() {
        double tilt = Math.toRadians(OPENING_TILT_DEG);
        return OPENING_AHEAD_OF_TAG_M * Math.cos(tilt) - OPENING_ABOVE_TAG_M * Math.sin(tilt);
    }

    private static double openingOffsetY() {
        double tilt = Math.toRadians(OPENING_TILT_DEG);
        return OPENING_AHEAD_OF_TAG_M * Math.sin(tilt) + OPENING_ABOVE_TAG_M * Math.cos(tilt);
    }

    private static double speedBound(double speed, double launchDeg, double horizontalMeters, double heightMeters, boolean faster) {
        double limit = faster ? speedForRpm(MAX_RPM) : Math.max(speed * 0.05, 0.05);
        if (Double.isNaN(limit) || limit <= 0) {
            return Double.NaN;
        }
        double span = limit - speed;
        double step = Math.max(Math.abs(speed) * 0.002, 1e-4);
        if (step <= 0 || span == 0) {
            return speed;
        }
        int guard = (int) Math.min(400, Math.ceil(Math.abs(span) / step));
        double hit = speed;
        for (int i = 1; i <= guard; i++) {
            double candidate = speed + span * i / (double) guard;
            if (!hitsOpening(candidate, launchDeg, horizontalMeters, heightMeters)) {
                return hit;
            }
            hit = candidate;
        }
        return hit;
    }

    private static boolean hitsOpening(double speed, double launchDeg, double horizontalMeters, double heightMeters) {
        if (speed <= 0 || horizontalMeters <= 0 || OPENING_M <= 0) {
            return false;
        }
        double tilt = Math.toRadians(OPENING_TILT_DEG);
        double half = OPENING_M / 2.0;
        double x0 = horizontalMeters - half * Math.cos(tilt);
        double y0 = heightMeters - half * Math.sin(tilt);
        double x1 = horizontalMeters + half * Math.cos(tilt);
        double y1 = heightMeters + half * Math.sin(tilt);
        if (DRAG_PER_M > 0) {
            return dragClearsOpening(speed, launchDeg, x0, y0, x1, y1);
        }
        return parabolaClearsOpening(speed, launchDeg, x0, y0, x1, y1);
    }

    private static boolean parabolaClearsOpening(double speed, double launchDeg, double x0, double y0, double x1, double y1) {
        double theta = Math.toRadians(launchDeg);
        double cos = Math.cos(theta);
        if (Math.abs(cos) < SLOPE_EPS || speed <= 0) {
            return false;
        }
        double sin = Math.sin(theta);
        double radius = BALL_DIAMETER_M / 2.0;
        double xLo = Math.max(0, Math.min(x0, x1) - BALL_DIAMETER_M);
        double xHi = Math.max(x0, x1) + BALL_DIAMETER_M;
        int samples = (int) Math.ceil((xHi - xLo) / 0.002);
        if (samples < 1) {
            samples = 1;
        }
        if (samples > 500) {
            samples = 500;
        }
        boolean cleared = false;
        for (int i = 0; i <= samples; i++) {
            double x = xLo + (xHi - xLo) * i / (double) samples;
            if (x <= 0) {
                continue;
            }
            double y = x * Math.tan(theta) - G * x * x / (2 * speed * speed * cos * cos);
            double vy = speed * sin - G * x / (speed * cos);
            int fit = diskOnOpening(x, y, x0, y0, x1, y1, radius);
            if (fit == 0 || vy >= 0) {
                continue;
            }
            if (fit < 0) {
                return false;
            }
            cleared = true;
        }
        return cleared;
    }

    private static boolean dragClearsOpening(double speed, double launchDeg, double x0, double y0, double x1, double y1) {
        double theta = Math.toRadians(launchDeg);
        double vx = speed * Math.cos(theta);
        double vy = speed * Math.sin(theta);
        double x = 0;
        double y = 0;
        double radius = BALL_DIAMETER_M / 2.0;
        double xHi = Math.max(x0, x1) + BALL_DIAMETER_M;
        double[] next = new double[4];
        boolean cleared = false;
        for (int i = 0; i < 8000 && x < xHi; i++) {
            heunStep(x, y, vx, vy, DRAG_DT, next);
            double nextX = next[0];
            double nextY = next[1];
            double nextVx = next[2];
            double nextVy = next[3];
            if (nextY < y) {
                int fit = diskOnOpening(nextX, nextY, x0, y0, x1, y1, radius);
                if (fit < 0 && nextVy < 0) {
                    return false;
                }
                if (fit > 0 && nextVy < 0) {
                    cleared = true;
                }
            }
            vx = nextVx;
            vy = nextVy;
            x = nextX;
            y = nextY;
            if (cleared && y < Math.min(y0, y1) - BALL_DIAMETER_M && vy < 0) {
                return true;
            }
        }
        return cleared;
    }

    private static int diskOnOpening(double cx, double cy, double x0, double y0, double x1, double y1, double radius) {
        double lx = x1 - x0;
        double ly = y1 - y0;
        double len = Math.hypot(lx, ly);
        if (len <= SLOPE_EPS || radius <= 0) {
            return 0;
        }
        double tx = lx / len;
        double ty = ly / len;
        double d = (cx - x0) * (-ty) + (cy - y0) * tx;
        if (Math.abs(d) > radius) {
            return 0;
        }
        double along = (cx - x0) * tx + (cy - y0) * ty;
        double half = Math.sqrt(Math.max(0, radius * radius - d * d));
        if (along - half >= -1e-6 && along + half <= len + 1e-6) {
            return 1;
        }
        return -1;
    }

    private static Flight flyUntilX(double speedMps, double launchDeg, double stopX) {
        if (speedMps <= 0 || stopX <= 0) {
            return null;
        }
        double theta = Math.toRadians(launchDeg);
        double vx = speedMps * Math.cos(theta);
        double vy = speedMps * Math.sin(theta);
        double x = 0;
        double y = 0;
        double t = 0;
        double dt = DRAG_DT;
        double[] next = new double[4];
        for (int i = 0; i < 8000 && x < stopX && t < 8; i++) {
            heunStep(x, y, vx, vy, dt, next);
            double nextX = next[0];
            double nextY = next[1];
            double nextVx = next[2];
            double nextVy = next[3];
            double nextT = t + dt;
            if (nextX >= stopX) {
                double frac = (stopX - x) / (nextX - x);
                Flight flight = new Flight();
                flight.x = stopX;
                flight.y = y + (nextY - y) * frac;
                flight.vx = vx + (nextVx - vx) * frac;
                flight.vy = vy + (nextVy - vy) * frac;
                flight.t = t + (nextT - t) * frac;
                return flight;
            }
            vx = nextVx;
            vy = nextVy;
            x = nextX;
            y = nextY;
            t = nextT;
        }
        return null;
    }

    private static void heunStep(double x, double y, double vx, double vy, double dt, double[] next) {
        double v = Math.hypot(vx, vy);
        double drag = DRAG_PER_M * v;
        double ax = -drag * vx;
        double ay = -G - drag * vy;
        double vx1 = vx + ax * dt;
        double vy1 = vy + ay * dt;
        double v1 = Math.hypot(vx1, vy1);
        double drag1 = DRAG_PER_M * v1;
        double ax1 = -drag1 * vx1;
        double ay1 = -G - drag1 * vy1;
        next[0] = x + 0.5 * (vx + vx1) * dt;
        next[1] = y + 0.5 * (vy + vy1) * dt;
        next[2] = vx + 0.5 * (ax + ax1) * dt;
        next[3] = vy + 0.5 * (ay + ay1) * dt;
    }

    private static boolean bad(double value) {
        return Double.isNaN(value) || Double.isInfinite(value);
    }

    private static class Flight {
        double x;
        double y;
        double vx;
        double vy;
        double t;
    }
}

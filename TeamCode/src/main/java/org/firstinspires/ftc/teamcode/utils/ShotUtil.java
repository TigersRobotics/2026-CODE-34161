package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.constants.TurretConstants;

public class ShotUtil {
    private static double lastTf = 0.6; // s

    private static double lastHood = Double.NaN;
    private static long hoodChangedAtMs;
    private static double lastRpm = Double.NaN;
    private static long rpmChangedAtMs;
    private static boolean haveArc;
    private static boolean lastSteep;
    private static boolean behind;

    public static class ShotResult {
        public boolean valid;
        public String reason = "";
        public double hoodDeg;
        public double rpmTarget;
        public double yawFieldDeg;
        public double vPerp;
        public double ballSpeed;
        public double thetaDeg;
        public double phiDeg;
        public double flightTime;
        public boolean steepArc;
        public double range;
        public boolean behind;
    }

    public static class Ready {
        public boolean ready;
        public String reason = "";
    }

    public static double ballSpeed(double rpm, double r) {
        return TurretConstants.W_V * (Math.PI * TurretConstants.WHEEL_DIAMETER * rpm / 60.0)
                * Math.exp(-TurretConstants.W_D * TurretConstants.DRAG_K * r); // m/s
    }

    public static double[] launchAngles(double v, double r) {
        if (r <= 1e-9) return null;
        double dh = TurretConstants.HIVE_Z - TurretConstants.SHOOTER_Z; // m
        double g = TurretConstants.G;
        double v2 = v * v;
        double s = v2 * v2 - g * (g * r * r + 2 * dh * v2);
        if (s < 0) return null;
        double root = Math.sqrt(s);
        double gr = g * r;
        return new double[] {
                Math.toDegrees(Math.atan((v2 + root) / gr)), // deg
                Math.toDegrees(Math.atan((v2 - root) / gr)) // deg
        };
    }

    public static double entryAngle(double thetaDeg, double r) {
        double dh = TurretConstants.HIVE_Z - TurretConstants.SHOOTER_Z; // m
        return Math.toDegrees(Math.atan(Math.tan(Math.toRadians(thetaDeg)) - 2 * dh / r)); // deg
    }

    public static double hoodFor(double thetaDeg, double betaDeg) {
        return TurretConstants.W_H1 * thetaDeg + TurretConstants.W_H0
                + TurretConstants.W_BETA * Math.abs(betaDeg); // deg
    }

    public static double rpmTarget(double r) {
        return TurretConstants.RPM_A + TurretConstants.RPM_B * r;
    }

    public static double hiveYaw(double x, double y) {
        return Math.toDegrees(Math.atan2(-y, -x)); // deg
    }

    public static double vPerp(double x, double y, double vx, double vy) {
        double r = Math.hypot(x, y);
        if (r < 1e-9) return 0;
        return (vx * y - vy * x) / r; // m/s
    }

    public static double vPar(double x, double y, double vx, double vy) {
        double r = Math.hypot(x, y);
        if (r < 1e-9) return 0;
        return -(vx * x + vy * y) / r; // m/s
    }

    public static double effectiveRange(double r, double vPar, double tf) {
        return r - TurretConstants.W_T * vPar * tf; // m
    }

    public static ShotResult compute(double x, double y, double vx, double vy, double measuredRpm, boolean moving) {
        ShotResult result = new ShotResult();
        x -= TurretConstants.HIVE_X;
        y -= TurretConstants.HIVE_Y;
        if (behind) {
            if (y > TurretConstants.BEHIND_HYST) behind = false;
        } else if (y < -TurretConstants.BEHIND_HYST) {
            behind = true;
        }
        result.behind = behind;
        if (moving) {
            x += vx * TurretConstants.T_LAT;
            y += vy * TurretConstants.T_LAT;
        }

        double r = Math.hypot(x, y);
        double beta = Math.toDegrees(Math.atan2(Math.abs(x), Math.abs(y))); // deg
        double rCalc = moving ? effectiveRange(r, vPar(x, y, vx, vy), lastTf) : r;

        result.range = r;
        result.yawFieldDeg = hiveYaw(x, y);
        result.vPerp = vPerp(x, y, vx, vy);
        result.rpmTarget = rpmTarget(rCalc);
        result.ballSpeed = ballSpeed(measuredRpm, rCalc);

        if (rCalc <= 1e-9) {
            result.reason = "too close/fast";
            return result;
        }

        double[] angles = launchAngles(result.ballSpeed, rCalc);
        if (angles == null) {
            result.reason = "too slow";
            return result;
        }

        boolean tooFast = false;
        boolean tooSlow = false;
        boolean rising = false;
        boolean behindShallow = false;
        double bestDelta = Double.POSITIVE_INFINITY;
        boolean found = false;
        int arcCount = result.behind ? 1 : angles.length;
        for (int i = 0; i < arcCount; i++) {
            double theta = angles[i];
            double phi = entryAngle(theta, rCalc);
            double hood = hoodFor(theta, beta);
            if (result.behind) {
                if (phi <= TurretConstants.BEHIND_MIN_PHI) {
                    behindShallow = true;
                    continue;
                }
            } else if (phi <= 0) {
                rising = true;
                continue;
            }
            if (hood > TurretConstants.HOOD_MAX_DEG || hood < TurretConstants.HOOD_MIN_DEG) {
                if (i == 0) {
                    boolean tooSteep = TurretConstants.W_H1 < 0
                            ? hood < TurretConstants.HOOD_MIN_DEG
                            : hood > TurretConstants.HOOD_MAX_DEG;
                    if (tooSteep) tooFast = true;
                    else tooSlow = true;
                }
                continue;
            }
            double delta = Math.abs(phi - TurretConstants.PHI_STAR);
            if (delta < bestDelta) {
                bestDelta = delta;
                found = true;
                result.steepArc = i == 0;
                result.thetaDeg = theta;
                result.phiDeg = phi;
                result.hoodDeg = hood;
            }
        }

        if (!found) {
            if (behindShallow) result.reason = "behind too shallow";
            else if (tooFast) result.reason = "too fast";
            else if (tooSlow) result.reason = "too slow";
            else if (rising) result.reason = "arc rising";
            else result.reason = "too slow";
            return result;
        }

        double horiz = result.ballSpeed * Math.cos(Math.toRadians(result.thetaDeg));
        if (Math.abs(horiz) < 1e-6) {
            result.reason = "too slow";
            return result;
        }
        result.flightTime = rCalc / horiz; // s
        lastTf = result.flightTime;
        result.valid = true;
        return result;
    }

    public static Ready readyToShoot(ShotResult result, double actualYawDeg, double targetYawDeg,
            double hoodCommandDeg, double measuredRpm, boolean moving, double vx, double vy) {
        long now = System.currentTimeMillis();
        boolean hoodMoving = hoodUnsettled(hoodCommandDeg, now);
        boolean rpmUnstable = rpmUnsettled(measuredRpm, now);
        boolean flipped = false;
        if (result.valid) {
            flipped = haveArc && result.steepArc != lastSteep;
            haveArc = true;
            lastSteep = result.steepArc;
        }

        Ready ready = new Ready();
        if (!result.valid) {
            ready.reason = "no shot";
            return ready;
        }

        double yawErr = actualYawDeg - targetYawDeg;
        yawErr %= 360;
        if (yawErr > 180) yawErr -= 360;
        if (yawErr < -180) yawErr += 360;
        double yawTol = result.range < 1e-6
                ? 90
                : Math.toDegrees(Math.atan(TurretConstants.YAW_TOL / result.range)); // deg
        if (Math.abs(yawErr) > yawTol) {
            ready.reason = "turret moving";
            return ready;
        }
        if (hoodMoving) {
            ready.reason = "hood moving";
            return ready;
        }
        if (rpmUnstable) {
            ready.reason = "rpm unstable";
            return ready;
        }
        if (flipped) {
            ready.reason = "arc flip";
            return ready;
        }
        if (result.hoodDeg <= TurretConstants.HOOD_MIN_DEG + TurretConstants.HOOD_EDGE_DEG
                || result.hoodDeg >= TurretConstants.HOOD_MAX_DEG - TurretConstants.HOOD_EDGE_DEG) {
            ready.reason = "hood at limit";
            return ready;
        }
        if (moving) {
            double speed = Math.hypot(vx, vy); // m/s
            double horiz = result.ballSpeed * Math.cos(Math.toRadians(result.thetaDeg));
            double lead = Math.abs(horiz) < 1e-6
                    ? Double.POSITIVE_INFINITY
                    : Math.abs(TurretConstants.W_T * result.vPerp / horiz);
            if (speed > TurretConstants.MAX_MOVING_SPEED || lead > TurretConstants.MAX_LEAD_RATIO) {
                ready.reason = "too fast to shoot";
                return ready;
            }
        }

        ready.ready = true;
        return ready;
    }

    private static boolean hoodUnsettled(double hood, long now) {
        if (Double.isNaN(lastHood) || Math.abs(hood - lastHood) > TurretConstants.HOOD_SETTLE_DEG) {
            lastHood = hood;
            hoodChangedAtMs = now;
        }
        return now - hoodChangedAtMs < TurretConstants.HOOD_SETTLE_MS;
    }

    private static boolean rpmUnsettled(double rpm, long now) {
        if (Double.isNaN(lastRpm) || Math.abs(rpm - lastRpm) > TurretConstants.RPM_STABLE) {
            lastRpm = rpm;
            rpmChangedAtMs = now;
        }
        return now - rpmChangedAtMs < TurretConstants.RPM_STABLE_MS;
    }

    public static void reset() {
        lastTf = 0.6; // s
        lastHood = Double.NaN;
        hoodChangedAtMs = 0;
        lastRpm = Double.NaN;
        rpmChangedAtMs = 0;
        haveArc = false;
        lastSteep = false;
        behind = false;
    }
}

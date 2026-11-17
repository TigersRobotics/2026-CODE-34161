package org.firstinspires.ftc.teamcode.sensing;

import static org.firstinspires.ftc.teamcode.constants.SensingConstants.ODOM_MAX_SIGMA_IN;
import static org.firstinspires.ftc.teamcode.constants.SensingConstants.ODOM_VARIANCE_PER_INCH;
import static org.firstinspires.ftc.teamcode.constants.SensingConstants.ODOM_VARIANCE_PER_SECOND;
import static org.firstinspires.ftc.teamcode.constants.SensingConstants.RELOCK_AGREE_IN;
import static org.firstinspires.ftc.teamcode.constants.SensingConstants.RELOCK_FRAMES;
import static org.firstinspires.ftc.teamcode.constants.SensingConstants.VISION_GATE_CHI_SQUARED;

/**
 * Blends the odometry with position fixes from the camera.
 * <p>
 * The odometry is smooth and fast but slowly drifts. The camera never drifts but is noisy, late,
 * and sometimes plain wrong. So the odometry is always the thing that moves the position, and the
 * camera only nudges a correction that is added on top of it:
 * <pre>    position = odometry + correction</pre>
 * How hard each camera fix nudges depends on how much each side is trusted at that moment
 * (a one-number Kalman filter). Three things make it hold up with a poor camera:
 * <ul>
 * <li><b>Lateness:</b> a camera frame is ~50-100 ms old when it arrives. Recent odometry is kept
 *     in a small history, and each fix is compared with where the robot was when the picture was
 *     taken, not where it is now.</li>
 * <li><b>Glitches:</b> a fix that disagrees too much with the current position is thrown away.</li>
 * <li><b>Getting shoved:</b> if the thrown-away fixes all agree with each other for a while, the
 *     odometry is the one that is wrong and the position jumps to the camera's answer.</li>
 * </ul>
 * Heading always comes from the Pinpoint's gyro: it is far steadier than anything a webcam can
 * measure from a small tag.
 * <p>
 * This class has no FTC SDK in it and allocates nothing after it is constructed, so it adds no
 * garbage-collector pauses to the loop.
 */
public final class PoseFusion {
    private static final int HISTORY = 256; // must be a power of two; ~2 seconds at a 8 ms loop
    private static final int MASK = HISTORY - 1;
    private static final double RATE_SMOOTHING = 0.5;
    private static final long RELOCK_CANDIDATE_TIMEOUT_NS = 1_000_000_000L;
    private static final double MAX_VARIANCE = ODOM_MAX_SIGMA_IN * ODOM_MAX_SIGMA_IN;
    private static final double LEAN_SMOOTHING = 0.25;
    // Pure noise, smoothed like this, has a variance of total x s/(2-s). Trigger at 3 sigma of that.
    private static final double LEAN_TRIGGER = 9.0 * LEAN_SMOOTHING / (2.0 - LEAN_SMOOTHING);

    // Odometry history, newest at index 'head'
    private final long[] timeNs = new long[HISTORY];
    private final double[] histX = new double[HISTORY];
    private final double[] histY = new double[HISTORY];
    private final double[] histHeading = new double[HISTORY]; // unwrapped, so it can be interpolated
    private final double[] histTurret = new double[HISTORY];
    private int head = -1;
    private int count = 0;

    // The correction added to the odometry, and how unsure of it we are (inches squared)
    private double offsetX, offsetY;
    private double variance;

    // Rates worked out from the odometry itself
    private double velX, velY, turnRate, turretRate;

    // The average direction recent accepted camera fixes have been pulling in
    private double leanX, leanY;

    // A run of rejected camera fixes that agree with each other
    private double candidateX, candidateY;
    private int candidateCount;
    private long candidateTimeNs;

    // Statistics for telemetry
    private int accepted, rejected, relocks, catchUps;
    private double lastInnovationIn = Double.NaN;
    private long lastAcceptNs = Long.MIN_VALUE;

    /** Filled in by {@link #sample(long)}. */
    public double sampleX, sampleY, sampleHeading, sampleTurret;

    public PoseFusion(double startSigmaIn) {
        reset(startSigmaIn);
    }

    /** Forgets everything. Call when the odometry is given a new starting pose. */
    public void reset(double startSigmaIn) {
        head = -1;
        count = 0;
        offsetX = 0;
        offsetY = 0;
        variance = startSigmaIn * startSigmaIn;
        velX = velY = turnRate = turretRate = 0;
        candidateCount = 0;
        leanX = leanY = 0;
        accepted = rejected = relocks = catchUps = 0;
        lastInnovationIn = Double.NaN;
        lastAcceptNs = Long.MIN_VALUE;
    }

    /**
     * Call once per loop with the newest odometry reading.
     *
     * @param nowNs      System.nanoTime() for this reading
     * @param headingRad heading in radians, any range
     * @param turretRad  turret angle in radians (pass 0 if the camera is not on a turret)
     */
    public void addOdometry(long nowNs, double x, double y, double headingRad, double turretRad) {
        double heading = headingRad;

        if (count > 0) {
            double dt = (nowNs - timeNs[head]) * 1e-9;
            double dx = x - histX[head];
            double dy = y - histY[head];
            double dHeading = wrap(headingRad - histHeading[head]);
            heading = histHeading[head] + dHeading;

            if (dt <= 0) {
                // Same instant as the last reading: replace it rather than adding a zero-length step
                histX[head] = x;
                histY[head] = y;
                histHeading[head] = heading;
                histTurret[head] = turretRad;
                return;
            }

            // The further the robot drives, the less the odometry (and so the correction) is trusted
            double distance = Math.sqrt(dx * dx + dy * dy);
            variance += ODOM_VARIANCE_PER_INCH * distance + ODOM_VARIANCE_PER_SECOND * dt;
            if (variance > MAX_VARIANCE) variance = MAX_VARIANCE;

            double invDt = 1.0 / dt;
            velX += RATE_SMOOTHING * (dx * invDt - velX);
            velY += RATE_SMOOTHING * (dy * invDt - velY);
            turnRate += RATE_SMOOTHING * (dHeading * invDt - turnRate);
            turretRate += RATE_SMOOTHING * ((turretRad - histTurret[head]) * invDt - turretRate);
        }

        head = (head + 1) & MASK;
        if (count < HISTORY) count++;
        timeNs[head] = nowNs;
        histX[head] = x;
        histY[head] = y;
        histHeading[head] = heading;
        histTurret[head] = turretRad;
    }

    /**
     * Works out where the odometry said the robot was at an earlier moment, and puts the answer in
     * sampleX / sampleY / sampleHeading / sampleTurret.
     *
     * @return false if that moment is older than the history (or there is no history yet)
     */
    public boolean sample(long whenNs) {
        if (count == 0) return false;

        if (whenNs >= timeNs[head]) {
            sampleX = histX[head];
            sampleY = histY[head];
            sampleHeading = histHeading[head];
            sampleTurret = histTurret[head];
            return true;
        }

        // Camera frames are only a few loops old, so walking back from the newest entry is quickest
        int newer = head;
        for (int k = 1; k < count; k++) {
            int older = (head - k) & MASK;
            if (timeNs[older] <= whenNs) {
                double span = timeNs[newer] - timeNs[older];
                double f = span > 0 ? (whenNs - timeNs[older]) / span : 0;
                sampleX = histX[older] + f * (histX[newer] - histX[older]);
                sampleY = histY[older] + f * (histY[newer] - histY[older]);
                sampleHeading = histHeading[older] + f * (histHeading[newer] - histHeading[older]);
                sampleTurret = histTurret[older] + f * (histTurret[newer] - histTurret[older]);
                return true;
            }
            newer = older;
        }
        return false;
    }

    /**
     * Offers a camera position fix.
     *
     * @param frameNs      when the picture was taken (System.nanoTime() clock)
     * @param measuredX    where the camera says the robot was at that moment
     * @param measuredY    same, Y
     * @param measVariance how noisy this fix is, in inches squared
     * @param nowNs        System.nanoTime() now
     * @return true if the fix was used
     */
    public boolean addVision(long frameNs, double measuredX, double measuredY, double measVariance, long nowNs) {
        if (!sample(frameNs)) return false;

        // The correction this fix is asking for, and how far that is from the one we have
        double impliedOffsetX = measuredX - sampleX;
        double impliedOffsetY = measuredY - sampleY;
        double innovationX = impliedOffsetX - offsetX;
        double innovationY = impliedOffsetY - offsetY;
        double innovationSq = innovationX * innovationX + innovationY * innovationY;
        double total = variance + measVariance;
        lastInnovationIn = Math.sqrt(innovationSq);

        if (innovationSq <= VISION_GATE_CHI_SQUARED * total) {
            // Honest camera noise points a different way every frame and averages out to nothing.
            // If the fixes keep leaning the SAME way, the correction is off by more than we
            // believed (a small shove, a slipping pod), so admit it: that makes the next fixes
            // count for far more, and the position catches up in a few frames instead of seconds.
            leanX += LEAN_SMOOTHING * (innovationX - leanX);
            leanY += LEAN_SMOOTHING * (innovationY - leanY);
            double leanSq = leanX * leanX + leanY * leanY;
            if (leanSq > LEAN_TRIGGER * total) {
                if (leanSq > variance) {
                    variance = Math.min(leanSq, MAX_VARIANCE);
                    total = variance + measVariance;
                }
                leanX = 0;
                leanY = 0;
                catchUps++;
            }

            double gain = variance / total;
            offsetX += gain * innovationX;
            offsetY += gain * innovationY;
            variance *= (1.0 - gain);
            candidateCount = 0;
            accepted++;
            lastAcceptNs = nowNs;
            return true;
        }

        rejected++;

        // Rejected. Is it part of a run of rejected fixes that all say the same thing?
        double candDx = impliedOffsetX - candidateX;
        double candDy = impliedOffsetY - candidateY;
        boolean agrees = candidateCount > 0
                && nowNs - candidateTimeNs < RELOCK_CANDIDATE_TIMEOUT_NS
                && candDx * candDx + candDy * candDy < RELOCK_AGREE_IN * RELOCK_AGREE_IN;

        if (agrees) {
            candidateCount++;
            candidateX += candDx / candidateCount; // running average of the run
            candidateY += candDy / candidateCount;
        } else {
            candidateCount = 1;
            candidateX = impliedOffsetX;
            candidateY = impliedOffsetY;
        }
        candidateTimeNs = nowNs;

        if (candidateCount >= RELOCK_FRAMES) {
            offsetX = candidateX;
            offsetY = candidateY;
            variance = measVariance;
            candidateCount = 0;
            leanX = 0;
            leanY = 0;
            relocks++;
            lastAcceptNs = nowNs;
            return true;
        }
        return false;
    }

    // ---- results -------------------------------------------------------------------------------

    public boolean hasPose() { return count > 0; }
    public double getX() { return count > 0 ? histX[head] + offsetX : Double.NaN; }
    public double getY() { return count > 0 ? histY[head] + offsetY : Double.NaN; }
    /** Heading in radians, between -pi and pi. */
    public double getHeading() { return count > 0 ? wrap(histHeading[head]) : Double.NaN; }

    public double getOffsetX() { return offsetX; }
    public double getOffsetY() { return offsetY; }
    /** One-sigma uncertainty of the position, in inches. */
    public double getSigmaIn() { return Math.sqrt(variance); }

    public double getVelX() { return velX; }
    public double getVelY() { return velY; }
    public double getSpeed() { return Math.sqrt(velX * velX + velY * velY); }
    /** Robot turn rate in radians per second. */
    public double getTurnRate() { return turnRate; }
    /** Turret turn rate in radians per second. */
    public double getTurretRate() { return turretRate; }

    public int getAcceptedCount() { return accepted; }
    public int getRejectedCount() { return rejected; }
    public int getRelockCount() { return relocks; }
    /** How many times the fixes kept leaning one way and the correction was hurried along. */
    public int getCatchUpCount() { return catchUps; }
    /** How far the last camera fix was from the fused position, in inches. */
    public double getLastInnovationIn() { return lastInnovationIn; }
    /** Milliseconds since a camera fix was last used, or infinity if never. */
    public double msSinceVisionFix(long nowNs) {
        return lastAcceptNs == Long.MIN_VALUE ? Double.POSITIVE_INFINITY : (nowNs - lastAcceptNs) * 1e-6;
    }

    /** Brings an angle in radians into the range -pi to pi. */
    public static double wrap(double radians) {
        radians %= 2 * Math.PI;
        if (radians > Math.PI) radians -= 2 * Math.PI;
        else if (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}

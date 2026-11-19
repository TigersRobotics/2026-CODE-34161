package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.constants.RobotSpecs;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

/**
 * Automatic flywheel velocity-PIDF search.
 * <p>
 * The loop under test is the same one the robot runs: encoder velocity mode, with
 * {@code setPIDFCoefficients(RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF))}
 * and {@code setVelocity} in ticks per second. One coefficient is moved at a time.
 * Each step tests the low end of the remaining window, the high end, then the midpoint,
 * and keeps the side that recovers from a shot fastest.
 * <p>
 * A shot is a sudden drop in flywheel speed. The score is how long the wheel takes to
 * get back to the target and stay there. Nothing is read from the gamepad.
 */
@TeleOp(name = "Flywheel PIDF Tuning Automatic", group = "Tuning")
public class FlywheelPIDFTuningAutomatic extends LinearOpMode {

    private static final int P = 0;
    private static final int I = 1;
    private static final int D = 2;
    private static final int F = 3;
    private static final String[] AXIS_NAMES = {"kP", "kI", "kD", "kF"};

    /** Same high target the manual flywheel tuner commands, in ticks per second. */
    private static final double INITIAL_TARGET_TICKS = 1500.0;
    private static final double MIN_TARGET_TICKS = 400.0;
    /** Closed-loop spin-up budget. Past this, the target is treated as a hardware limit. */
    private static final long SPINUP_TIMEOUT_MS = 15_000L;
    private static final long OPEN_LOOP_CAP_MS = 8_000L;
    private static final long BLEED_MS = 1_500L;
    private static final long AT_SPEED_HOLD_MS = 250L;
    private static final long RECOVER_HOLD_MS = 150L;
    private static final long RECOVERY_CAP_MS = 5_000L;
    private static final long SUDDEN_DROP_WINDOW_NS = 400_000_000L;
    private static final long SAMPLE_MS = 20L;
    private static final long VELOCITY_RESEND_NS = 500_000_000L;

    private static final int MIN_PASSES = 2;
    private static final int MAX_PASSES = 4;
    private static final int MAX_ITERS = 7;
    private static final double TIE_MS = 30.0;
    private static final double IMPROVEMENT_MS = 30.0;
    private static final double UNREACHABLE = 1.0e9;
    private static final double SPEED_DROP = 0.85;

    private DcMotorEx flywheel;

    private double targetTicksPerSec = INITIAL_TARGET_TICKS;
    private double hardwareMaxTicks;
    private int speedReductions;
    private String targetNote = "";
    private String directionLabel = "forward";
    private boolean directionFlipped;

    private final double[] axisLo = new double[4];
    private final double[] axisHi = new double[4];
    private final double[] axisTol = new double[4];

    private final double[] cursor = new double[4];
    private final double[] seedGains = new double[4];
    private final double[] bestGains = new double[4];
    private double bestRecoveryMs = Double.NaN;

    private final Map<String, Double> cache = new HashMap<>();
    private boolean restartSearch;
    private boolean giveUp;
    private boolean searchComplete;
    private int consecutiveUnreachable;

    private int passNumber;
    private int axisIndex;
    private int iteration;
    private int probeInIteration;
    private int trialsMeasured;
    private int confirmations;

    private String phase = "Idle";
    private String tuningLabel = "—";
    private double testingValue;
    private double windowLo;
    private double windowHi;
    private double shownP;
    private double shownI;
    private double shownD;
    private double shownF;
    private double lastLowScore = Double.NaN;
    private double lastMidScore = Double.NaN;
    private double lastHighScore = Double.NaN;
    private double recoveryShown = Double.NaN;
    private double dropShown;
    private double spinUpElapsedMs;

    private double sentP = Double.NaN;
    private double sentI = Double.NaN;
    private double sentD = Double.NaN;
    private double sentF = Double.NaN;
    private double sentVelocity = Double.NaN;
    private long velocitySentAtNs;

    private final double[] velocityWindow = new double[3];
    private int velocityCount;
    private int velocityPos;
    private double lastRaw;
    private double lastVelocity;

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, HardwareNames.FLYWHEEL_NAME);
        // ShooterTuning and the manual PIDF tuner leave the default direction.
        // Reversing here spins the wheel backwards while the encoder still counts up.
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Flywheel PIDF Tuning Automatic");
        telemetry.addLine("No controller input.");
        telemetry.addLine("Feed balls through the wheel. A shot is a sudden RPM drop.");
        telemetry.addLine("The score is how long the wheel takes to recover.");
        telemetry.addData("Starting target", "%.0f ticks/s", INITIAL_TARGET_TICKS);
        telemetry.update();

        waitForStart();

        try {
            if (opModeIsActive()) {
                runSearch();
                holdResult();
            }
        } finally {
            shutdown();
        }
    }

    private void runSearch() {
        if (!chooseTargetFromHardware()) {
            giveUp = true;
            if (opModeIsActive()) {
                phase = "Stopped";
                targetNote = "Flywheel did not spin. Check the motor, encoder, and direction.";
            }
            return;
        }

        while (opModeIsActive() && !giveUp) {
            configureBounds();
            seed(seedGains);
            System.arraycopy(seedGains, 0, cursor, 0, 4);
            System.arraycopy(seedGains, 0, bestGains, 0, 4);
            show(seedGains);
            bestRecoveryMs = Double.NaN;
            cache.clear();
            restartSearch = false;
            consecutiveUnreachable = 0;
            passNumber = 0;
            axisIndex = 0;
            iteration = 0;
            probeInIteration = 0;

            tuningLabel = "reachable speed";
            windowLo = axisLo[F];
            windowHi = axisHi[F];
            testingValue = seedGains[F];
            if (!spinUp(seedGains)) {
                if (!opModeIsActive()) return;
                if (!lowerTarget("Closed loop did not reach speed within 15s.")) {
                    giveUp = true;
                    phase = "Stopped";
                    targetNote = "Could not reach even the lowest target. Check the flywheel hardware.";
                    break;
                }
                continue;
            }

            for (passNumber = 1; passNumber <= MAX_PASSES && opModeIsActive() && !restartSearch && !giveUp; passNumber++) {
                double before = bestRecoveryMs;
                if (!Double.isNaN(bestRecoveryMs)) {
                    System.arraycopy(bestGains, 0, cursor, 0, 4);
                } else {
                    System.arraycopy(seedGains, 0, cursor, 0, 4);
                }

                for (axisIndex = 0; axisIndex < 4 && opModeIsActive() && !restartSearch && !giveUp; axisIndex++) {
                    searchAxis(axisIndex);
                }

                if (restartSearch || giveUp || !opModeIsActive()) break;

                boolean improved = !Double.isNaN(bestRecoveryMs)
                        && (Double.isNaN(before) || bestRecoveryMs <= before - IMPROVEMENT_MS);
                if (passNumber >= MIN_PASSES && !improved) break;
            }

            if (!restartSearch) break;
        }
    }

    /**
     * Spins the wheel open-loop and keeps a target the hardware can actually hold.
     * The closed-loop search still lowers the target again if a reachable setpoint
     * cannot be held within {@link #SPINUP_TIMEOUT_MS}.
     */
    private boolean chooseTargetFromHardware() {
        phase = "Measuring hardware max";
        tuningLabel = "hardware max";
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        directionLabel = "forward";
        double peak = coastUp(DcMotor.Direction.FORWARD);

        // A negative encoder means this direction is backwards. Flip once.
        if (peak < 200.0) {
            double flippedPeak = coastUp(DcMotor.Direction.REVERSE);
            if (flippedPeak > peak) {
                peak = flippedPeak;
                directionLabel = "reverse";
                directionFlipped = true;
            } else {
                flywheel.setDirection(DcMotor.Direction.FORWARD);
                directionLabel = "forward";
            }
        }

        hardwareMaxTicks = peak;
        if (peak < 200.0) return false;

        double reachable = peak * 0.90;
        if (reachable < INITIAL_TARGET_TICKS) {
            targetTicksPerSec = reachable;
            speedReductions++;
            targetNote = String.format(Locale.US,
                    "Target lowered from %.0f to %.0f ticks/s. Open-loop max was %.0f.",
                    INITIAL_TARGET_TICKS, targetTicksPerSec, peak);
        } else {
            targetTicksPerSec = INITIAL_TARGET_TICKS;
            targetNote = String.format(Locale.US,
                    "Open-loop max %.0f ticks/s. Target stays %.0f.",
                    peak, targetTicksPerSec);
        }
        return targetTicksPerSec >= 150.0;
    }

    private double coastUp(DcMotor.Direction direction) {
        flywheel.setDirection(direction);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        forgetSentCommands();
        resetFilter();
        double peak = 0.0;
        try {
            flywheel.setPower(1.0);
            long startNs = System.nanoTime();
            long stableSinceNs = -1L;
            double previous = 0.0;
            while (opModeIsActive() && msSince(startNs) < OPEN_LOOP_CAP_MS) {
                readVelocity();
                // Median rejects a one-sample encoder spike so it cannot set the hardware max.
                if (velocityCount >= 3 && msSince(startNs) > 200.0 && lastVelocity > peak) {
                    peak = lastVelocity;
                    hardwareMaxTicks = Math.max(hardwareMaxTicks, peak);
                }
                phase = "Measuring hardware max";
                publish();

                double tolerance = Math.max(40.0, peak * 0.02);
                if (lastVelocity > 200.0 && Math.abs(lastVelocity - previous) <= tolerance) {
                    if (stableSinceNs < 0L) stableSinceNs = System.nanoTime();
                    if (msSince(stableSinceNs) >= 500.0) break;
                } else {
                    stableSinceNs = -1L;
                }
                previous = lastVelocity;
                pause();
            }
        } finally {
            flywheel.setPower(0.0);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            forgetSentCommands();
        }
        return peak;
    }

    private void searchAxis(int axis) {
        double lo = axisLo[axis];
        double hi = axisHi[axis];
        double bestValue = cursor[axis];
        double bestScore = UNREACHABLE;
        tuningLabel = AXIS_NAMES[axis];
        iteration = 0;

        while (opModeIsActive() && !restartSearch && !giveUp
                && iteration < MAX_ITERS
                && hi - lo > axisTol[axis]) {
            windowLo = lo;
            windowHi = hi;
            double mid = (lo + hi) / 2.0;

            probeInIteration = 0;
            double lowScore = probe(axis, lo);
            if (leaveSearch()) break;

            probeInIteration = 1;
            double highScore = probe(axis, hi);
            if (leaveSearch()) break;

            probeInIteration = 2;
            double midScore = probe(axis, mid);
            if (leaveSearch()) break;

            lastLowScore = lowScore;
            lastHighScore = highScore;
            lastMidScore = midScore;

            if (lowScore < bestScore) {
                bestScore = lowScore;
                bestValue = lo;
            }
            if (highScore < bestScore) {
                bestScore = highScore;
                bestValue = hi;
            }
            if (midScore < bestScore) {
                bestScore = midScore;
                bestValue = mid;
            }

            int pick = selectProbe(lowScore, midScore, highScore);
            if (pick < 0) {
                hi = mid;
            } else if (pick > 0) {
                lo = mid;
            } else {
                double nextLo = (lo + mid) / 2.0;
                double nextHi = (mid + hi) / 2.0;
                lo = nextLo;
                hi = nextHi;
            }
            iteration++;
        }

        cursor[axis] = bestValue;
        iteration = MAX_ITERS;
        probeInIteration = 0;
    }

    /** @return -1 when the low probe wins, 0 when the middle wins, 1 when the high probe wins */
    static int selectProbe(double lowScore, double midScore, double highScore) {
        double best = Math.min(lowScore, Math.min(midScore, highScore));
        boolean lowTied = lowScore <= best + TIE_MS;
        boolean midTied = midScore <= best + TIE_MS;
        boolean highTied = highScore <= best + TIE_MS;
        if (midTied) return 0;
        if (lowTied && !highTied) return -1;
        if (highTied && !lowTied) return 1;
        return -1;
    }

    private double probe(int axis, double value) {
        cursor[axis] = value;
        testingValue = value;
        show(cursor);
        publish();

        String cacheKey = key(cursor);
        Double cached = cache.get(cacheKey);
        if (cached != null) {
            remember(cursor, cached);
            recoveryShown = cached;
            return cached;
        }

        double[] trial = copy(cursor);
        double score = measure(trial);
        cache.put(key(trial), score);
        trialsMeasured++;
        remember(trial, score);
        if (opModeIsActive()) noteUnreachable(trial, score);
        recoveryShown = score;
        return score;
    }

    private double measure(double[] gains) {
        if (!spinUp(gains)) return UNREACHABLE;
        return waitForShotRecovery();
    }

    /**
     * Commands zero briefly so the wheel has to accelerate under the gains being scored,
     * then waits until it holds the target.
     */
    private boolean spinUp(double[] gains) {
        applyGains(gains);
        show(gains);

        long bleedStart = System.nanoTime();
        while (opModeIsActive() && msSince(bleedStart) < BLEED_MS) {
            commandVelocity(0.0);
            double velocity = readVelocity();
            phase = "Settling before trial";
            spinUpElapsedMs = 0.0;
            publish();
            if (velocity <= targetTicksPerSec * 0.50) break;
            pause();
        }
        if (!opModeIsActive()) return false;

        long spinStart = System.nanoTime();
        long inBandSince = -1L;
        while (opModeIsActive()) {
            commandVelocity(targetTicksPerSec);
            double velocity = readVelocity();
            spinUpElapsedMs = msSince(spinStart);
            if (!directionFlipped && velocity < -150.0 && spinUpElapsedMs > 400.0) {
                flipDirection();
                spinStart = System.nanoTime();
                inBandSince = -1L;
                resetFilter();
                continue;
            }
            if (Math.abs(velocity - targetTicksPerSec) <= speedBand()) {
                if (inBandSince < 0L) inBandSince = System.nanoTime();
                phase = "Spinning up";
                if (msSince(inBandSince) >= AT_SPEED_HOLD_MS) {
                    phase = "At speed";
                    publish();
                    return true;
                }
            } else {
                inBandSince = -1L;
                phase = "Spinning up";
            }
            publish();
            if (spinUpElapsedMs >= SPINUP_TIMEOUT_MS) return false;
            pause();
        }
        return false;
    }

    private double waitForShotRecovery() {
        long lostSince = -1L;
        long lastInBandNs = System.nanoTime();
        double reference = lastVelocity;
        int lowSamples = 0;
        boolean recovering = false;
        long shotAtNs = 0L;
        long recoverSince = -1L;

        while (opModeIsActive()) {
            commandVelocity(targetTicksPerSec);
            double velocity = readVelocity();
            long now = System.nanoTime();
            boolean inBand = Math.abs(velocity - targetTicksPerSec) <= speedBand();

            if (!recovering) {
                if (inBand) {
                    reference = velocity;
                    lastInBandNs = now;
                    lostSince = -1L;
                    lowSamples = 0;
                    phase = "Waiting for shot";
                } else {
                    boolean sudden = now - lastInBandNs <= SUDDEN_DROP_WINDOW_NS;
                    boolean deep = reference - velocity >= dropThreshold()
                            && velocity <= targetTicksPerSec - dropThreshold();
                    if (sudden && deep) {
                        lowSamples++;
                        phase = "Shot drop";
                        if (lowSamples >= 2) {
                            recovering = true;
                            shotAtNs = now;
                            dropShown = reference - velocity;
                            recoverSince = -1L;
                            phase = "Recovering";
                        }
                    } else {
                        lowSamples = 0;
                        phase = "Reacquiring speed";
                        if (lostSince < 0L) lostSince = now;
                        if (msSince(lostSince) >= SPINUP_TIMEOUT_MS) return UNREACHABLE;
                    }
                }
            } else if (inBand) {
                phase = "Recovering";
                if (recoverSince < 0L) recoverSince = now;
                if (msSince(recoverSince) >= RECOVER_HOLD_MS) {
                    double recovery = (recoverSince - shotAtNs) / 1.0e6;
                    recoveryShown = recovery;
                    return recovery;
                }
            } else {
                phase = "Recovering";
                recoverSince = -1L;
                if ((now - shotAtNs) / 1.0e6 >= RECOVERY_CAP_MS) {
                    recoveryShown = RECOVERY_CAP_MS;
                    return RECOVERY_CAP_MS;
                }
            }

            publish();
            pause();
        }
        return UNREACHABLE;
    }

    private void noteUnreachable(double[] trial, double score) {
        if (score < UNREACHABLE / 2.0) {
            consecutiveUnreachable = 0;
            return;
        }
        if (sameGains(trial, seedGains)) {
            if (!lowerTarget("Seed gains did not reach speed within 15s.")) giveUp = true;
            else restartSearch = true;
            return;
        }

        consecutiveUnreachable++;
        if (consecutiveUnreachable < 3) return;
        consecutiveUnreachable = 0;

        tuningLabel = "recheck speed";
        phase = "Rechecking hardware max";
        publish();
        if (!spinUp(seedGains)) {
            if (!opModeIsActive()) return;
            if (!lowerTarget("Best-known gains did not reach speed within 15s.")) giveUp = true;
            else restartSearch = true;
        }
    }

    private void holdResult() {
        searchComplete = true;
        if (giveUp) {
            phase = "Stopped";
            shutdown();
            while (opModeIsActive()) {
                readVelocity();
                publish();
                pause();
            }
            return;
        }
        if (!opModeIsActive()) return;

        double[] held = Double.isNaN(bestRecoveryMs) ? seedGains : bestGains;
        show(held);
        tuningLabel = "done";
        phase = "Holding optimal gains";
        if (!spinUp(held)) {
            if (!opModeIsActive()) return;
            targetNote = "Chosen gains did not hold speed. The best measured values are still on screen.";
            phase = "Holding optimal gains";
            while (opModeIsActive()) {
                readVelocity();
                publish();
                pause();
            }
            return;
        }

        while (opModeIsActive()) {
            phase = "Holding optimal gains";
            double recovery = waitForShotRecovery();
            if (!opModeIsActive()) return;
            if (recovery >= UNREACHABLE / 2.0) {
                if (!opModeIsActive()) return;
                if (!spinUp(held)) {
                    phase = "Holding optimal gains";
                    while (opModeIsActive()) {
                        readVelocity();
                        publish();
                        pause();
                    }
                    return;
                }
            } else {
                recoveryShown = recovery;
                confirmations++;
            }
        }
    }

    private boolean lowerTarget(String reason) {
        double next = targetTicksPerSec * SPEED_DROP;
        if (next < MIN_TARGET_TICKS) return false;
        double previous = targetTicksPerSec;
        targetTicksPerSec = next;
        speedReductions++;
        targetNote = String.format(Locale.US,
                "%s Target lowered from %.0f to %.0f ticks/s.",
                reason, previous, targetTicksPerSec);
        cache.clear();
        return true;
    }

    private void configureBounds() {
        axisLo[P] = 0.0;
        axisHi[P] = 20.0;
        axisTol[P] = 0.05;

        axisLo[I] = 0.0;
        axisHi[I] = 1.0;
        axisTol[I] = 0.01;

        axisLo[D] = 0.0;
        axisHi[D] = 5.0;
        axisTol[D] = 0.02;

        double seedF = 32767.0 / Math.max(hardwareMaxTicks, targetTicksPerSec);
        axisLo[F] = 0.0;
        axisHi[F] = Math.max(20.0, seedF * 2.0);
        axisTol[F] = Math.max(0.05, axisHi[F] / 128.0);
    }

    private void seed(double[] gains) {
        gains[P] = clamp(5.0, axisLo[P], axisHi[P]);
        gains[I] = 0.0;
        gains[D] = 0.0;
        double seedF = 32767.0 / Math.max(hardwareMaxTicks, 1.0);
        gains[F] = clamp(seedF, axisLo[F], axisHi[F]);
    }

    private void remember(double[] gains, double score) {
        if (score >= UNREACHABLE / 2.0) return;
        if (Double.isNaN(bestRecoveryMs) || score < bestRecoveryMs) {
            bestRecoveryMs = score;
            System.arraycopy(gains, 0, bestGains, 0, 4);
        }
    }

    private void applyGains(double[] gains) {
        if (gains[P] == sentP && gains[I] == sentI && gains[D] == sentD && gains[F] == sentF) return;

        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(gains[P], gains[I], gains[D], gains[F]));
        sentP = gains[P];
        sentI = gains[I];
        sentD = gains[D];
        sentF = gains[F];
        sentVelocity = Double.NaN;
    }

    private void commandVelocity(double ticksPerSec) {
        long now = System.nanoTime();
        boolean changed = ticksPerSec != sentVelocity;
        boolean stale = now - velocitySentAtNs > VELOCITY_RESEND_NS;
        if (!changed && !stale) return;

        flywheel.setVelocity(ticksPerSec);
        sentVelocity = ticksPerSec;
        velocitySentAtNs = now;
    }

    private double readVelocity() {
        double raw = flywheel.getVelocity();
        if (Double.isNaN(raw) || Double.isInfinite(raw)) raw = lastRaw;
        lastRaw = raw;
        velocityWindow[velocityPos] = raw;
        velocityPos = (velocityPos + 1) % velocityWindow.length;
        if (velocityCount < velocityWindow.length) velocityCount++;
        lastVelocity = median();
        return lastVelocity;
    }

    private double median() {
        if (velocityCount == 0) return 0.0;
        double[] sorted = new double[velocityCount];
        for (int i = 0; i < velocityCount; i++) sorted[i] = velocityWindow[i];
        Arrays.sort(sorted);
        return sorted[sorted.length / 2];
    }

    private void resetFilter() {
        velocityCount = 0;
        velocityPos = 0;
    }

    private void flipDirection() {
        DcMotor.Direction next = flywheel.getDirection() == DcMotor.Direction.REVERSE
                ? DcMotor.Direction.FORWARD
                : DcMotor.Direction.REVERSE;
        flywheel.setDirection(next);
        directionLabel = next == DcMotor.Direction.REVERSE ? "reverse" : "forward";
        directionFlipped = true;
        forgetSentCommands();
    }

    private void forgetSentCommands() {
        sentP = Double.NaN;
        sentI = Double.NaN;
        sentD = Double.NaN;
        sentF = Double.NaN;
        sentVelocity = Double.NaN;
    }

    private void publish() {
        double progress = searchComplete ? 100.0 : computeProgress();

        if ("Waiting for shot".equals(phase)) {
            telemetry.addLine("FIRE");
        } else {
            telemetry.addLine("Do not fire yet");
        }
        telemetry.addData("Progress", "%.1f%%", progress);
        telemetry.addData("Phase", phase);
        telemetry.addData("Pass", "%d / %d", Math.max(passNumber, 0), MAX_PASSES);
        telemetry.addData("Tuning", tuningLabel);
        telemetry.addData("Window", "%.4f to %.4f", windowLo, windowHi);
        telemetry.addData("Test value", "%.4f", testingValue);
        telemetry.addData("kP", "%.4f", shownP);
        telemetry.addData("kI", "%.4f", shownI);
        telemetry.addData("kD", "%.4f", shownD);
        telemetry.addData("kF", "%.4f", shownF);
        telemetry.addData("Best kP", "%.4f", bestGains[P]);
        telemetry.addData("Best kI", "%.4f", bestGains[I]);
        telemetry.addData("Best kD", "%.4f", bestGains[D]);
        telemetry.addData("Best kF", "%.4f", bestGains[F]);
        telemetry.addData("Best recovery", formatMs(bestRecoveryMs));
        telemetry.addData("Last recovery", formatMs(recoveryShown));
        telemetry.addData("Last min / max / mid", "%s / %s / %s",
                formatMs(lastLowScore), formatMs(lastHighScore), formatMs(lastMidScore));
        telemetry.addData("Target", "%.1f ticks/s   %.0f RPM", targetTicksPerSec, ticksToRpm(targetTicksPerSec));
        telemetry.addData("Velocity", "%.1f ticks/s   %.0f RPM", lastVelocity, ticksToRpm(lastVelocity));
        telemetry.addData("Raw velocity", "%.1f ticks/s   %.0f RPM", lastRaw, ticksToRpm(lastRaw));
        telemetry.addData("Error", "%.1f ticks/s", targetTicksPerSec - lastVelocity);
        telemetry.addData("Shot drop", "%.1f ticks/s   %.0f RPM", dropShown, ticksToRpm(dropShown));
        telemetry.addData("Hardware max", "%.1f ticks/s   %.0f RPM", hardwareMaxTicks, ticksToRpm(hardwareMaxTicks));
        telemetry.addData("Spin-up", "%.1f / %.0f s", spinUpElapsedMs / 1000.0, SPINUP_TIMEOUT_MS / 1000.0);
        telemetry.addData("Direction", directionLabel);
        telemetry.addData("Speed reductions", speedReductions);
        telemetry.addData("Trials scored", trialsMeasured);
        telemetry.addData("Confirm shots", confirmations);
        if (!targetNote.isEmpty()) telemetry.addLine(targetNote);
        if ("Waiting for shot".equals(phase)) {
            telemetry.addLine("FIRE — feed one ball through the wheel");
        }
        if (searchComplete && !giveUp && !Double.isNaN(bestRecoveryMs)) {
            telemetry.addLine(String.format(Locale.US,
                    "Optimal  kP %.4f  kI %.4f  kD %.4f  kF %.4f",
                    bestGains[P], bestGains[I], bestGains[D], bestGains[F]));
            String bounds = boundNote();
            if (!bounds.isEmpty()) telemetry.addLine(bounds);
        }
        telemetry.update();
    }

    private String boundNote() {
        StringBuilder names = new StringBuilder();
        for (int axis = 0; axis < 4; axis++) {
            boolean onLow = bestGains[axis] <= axisLo[axis] + axisTol[axis];
            boolean onHigh = bestGains[axis] >= axisHi[axis] - axisTol[axis];
            if (!onLow && !onHigh) continue;
            if (names.length() > 0) names.append(", ");
            names.append(AXIS_NAMES[axis]);
        }
        if (names.length() == 0) return "";
        return "On a search bound: " + names + ". The optimum may sit outside the window.";
    }

    private double computeProgress() {
        if (passNumber < 1) return 0.0;
        double done = (passNumber - 1) * 4.0 * MAX_ITERS
                + axisIndex * MAX_ITERS
                + iteration
                + probeInIteration / 3.0;
        double total = MAX_PASSES * 4.0 * MAX_ITERS;
        if (total <= 0.0) return 0.0;
        return Math.max(0.0, Math.min(99.9, 100.0 * done / total));
    }

    private void show(double[] gains) {
        shownP = gains[P];
        shownI = gains[I];
        shownD = gains[D];
        shownF = gains[F];
    }

    private double speedBand() {
        return Math.max(25.0, targetTicksPerSec * 0.03);
    }

    private double dropThreshold() {
        return Math.max(targetTicksPerSec * 0.05, speedBand() * 2.0);
    }

    private double ticksToRpm(double ticksPerSec) {
        if (RobotSpecs.FLYWHEEL_TICKS_PER_REV <= 0.0) return 0.0;
        return ticksPerSec * 60.0 / RobotSpecs.FLYWHEEL_TICKS_PER_REV;
    }

    private String key(double[] gains) {
        return String.format(Locale.US, "%.4f|%.4f|%.4f|%.4f|%.1f",
                gains[P], gains[I], gains[D], gains[F], targetTicksPerSec);
    }

    private boolean sameGains(double[] a, double[] b) {
        for (int i = 0; i < 4; i++) {
            if (Math.abs(a[i] - b[i]) > 0.0001) return false;
        }
        return true;
    }

    private boolean leaveSearch() {
        return !opModeIsActive() || restartSearch || giveUp;
    }

    private String formatMs(double ms) {
        if (Double.isNaN(ms)) return "n/a";
        if (ms >= UNREACHABLE / 2.0) return "did not reach speed";
        if (ms >= RECOVERY_CAP_MS) return String.format(Locale.US, "%.0f ms (capped)", ms);
        return String.format(Locale.US, "%.0f ms", ms);
    }

    private void pause() {
        if (!opModeIsActive()) return;
        sleep(SAMPLE_MS);
    }

    private void shutdown() {
        if (flywheel == null) return;
        try {
            flywheel.setPower(0.0);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setVelocity(0.0);
        } catch (RuntimeException ignored) {
            // Stopping should not hide the failure that got us here.
        }
    }

    private static double msSince(long startNs) {
        return (System.nanoTime() - startNs) / 1.0e6;
    }

    private static double clamp(double value, double lo, double hi) {
        return Math.max(lo, Math.min(hi, value));
    }

    private static double[] copy(double[] gains) {
        return new double[] {gains[0], gains[1], gains[2], gains[3]};
    }
}

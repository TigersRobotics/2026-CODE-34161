package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Flywheel PIDF tuning from shot recovery. No hood, no distances.
 *
 * You shoot balls. The opmode watches the flywheel velocity and measures, for every volley:
 *   - dip depth (how far the velocity dropped)
 *   - recovery time (from the bottom of the dip until it is back at speed and holding)
 *   - overshoot / ringing after it comes back
 *   - steady-state error and jitter once it has settled
 * Then it decides whether the term you are tuning is too HIGH or too LOW and moves it with a
 * binary search.
 *
 * Modes (Cross cycles them in tuning order): F -> P -> D -> I
 * Each mode only switches on the terms tuned before it:
 *   F : F only            (tuned with a hold test, no balls needed)
 *   P : F + P             too LOW = slow recovery, too HIGH = overshoot / ringing
 *   D : F + P + D         too HIGH = jittery velocity, too LOW = ringing after the shot
 *   I : F + P + D + I     too LOW = settles below target, too HIGH = overshoot / windup
 *
 * AUTO (Back button)
 *   F mode      : runs the hold test by itself (spin to target, wait, average, compare to target).
 *   P / D / I   : turns auto-adjust ON. Shoot a volley, it measures the recovery and adjusts the
 *                 value by binary search, then you shoot again. Repeats until the response is
 *                 GOOD or the bracket is narrow enough. Back again turns it off.
 * With auto-adjust OFF the opmode still measures every volley and shows what it would do, and you
 * can apply it yourself with Triangle (too high) / Square (too low) / Circle (lock).
 *
 * Load balls and let the wheel get back to speed between volleys. The opmode only measures a
 * volley once the wheel has been at speed for a moment ("READY").
 *
 * Controls (gamepad1)
 *   Cross ........... cycle mode F -> P -> D -> I
 *   Back ............ F: run hold test | P/D/I: auto-adjust on/off
 *   Triangle ........ value too HIGH   (manual binary search)
 *   Square .......... value too LOW    (manual binary search)
 *   Circle .......... lock this value (logs it)
 *   Right/Left bumper target velocity +/- 10
 *   Left trigger .... reset the search bracket of the current mode
 *   Right trigger ... hold to run the feeder
 *   Dpad up ......... feed pulse (a short timed feeder run)
 *   Start ........... flywheel on/off
 */

@Deprecated
@Disabled
@TeleOp(name = "Flywheel PIDF Tuning (Recovery)")
public class ShooterTuning extends OpMode {

    private static final String TAG = "ShooterTune";

    // ---------- CONFIG: change these to match your robot ----------
    private static final String FLYWHEEL_NAME = "flywheel";
    private static final String FEEDER_NAME = "uptake";   // optional; if missing you feed manually
    private static final double FEEDER_POWER = 1.0;
    private static final double FEED_PULSE_S = 0.4;       // length of the dpad-up feed pulse

    private static final double START_VELOCITY = 1500;
    private static final double VEL_MIN = 0, VEL_MAX = 3000;
    private static final double VEL_STEP = 10;

    // Starting PIDF values
    private static final double START_P = 11.0;
    private static final double START_I = 0.0;
    private static final double START_D = 0.0;
    private static final double START_F = 13.698;

    // Binary search brackets
    private static final double F_MIN = 0.0, F_MAX = 30.0;
    private static final double P_MIN = 0.0, P_MAX = 60.0;
    private static final double I_MIN = 0.0, I_MAX = 10.0;
    private static final double D_MIN = 0.0, D_MAX = 5.0;

    private static final double F_RESOLUTION = 0.01;
    private static final double P_RESOLUTION = 0.25;
    private static final double I_RESOLUTION = 0.05;
    private static final double D_RESOLUTION = 0.02;
    private static final int MAX_AUTO_ITERS = 14;

    // ---- F hold test ----
    private static final double F_SETTLE_S = 2.5;      // wait for the wheel to settle
    private static final double F_SAMPLE_S = 1.0;      // then average velocity over this long
    private static final double F_TOLERANCE = 0.005;   // within 0.5% of target = good

    // ---- shot measurement (P, I, D) ----
    private static final double SETTLE_BAND = 0.02;        // "at speed" = within 2% of target
    private static final double READY_S = 0.5;             // must be at speed this long before a shot counts
    private static final double DIP_TRIGGER = 0.03;        // a drop of >3% below target = a shot happened
    private static final double HOLD_S = 0.6;              // back in the band this long = recovered
    private static final double RECOVERY_TIMEOUT_S = 4.0;  // give up waiting and call it "never recovered"

    private static final double RECOVERY_LIMIT_S = 0.5;    // recovery slower than this = P too low
    private static final double OVERSHOOT_LIMIT = 0.03;    // >3% over target after the dip = overshoot
    private static final double OSC_DEADBAND = 0.015;      // crossing the target by >1.5% counts
    private static final int OSC_CROSSINGS = 2;            // over, then back under = ringing
    private static final double I_SS_TOL = 0.01;           // settled mean this far below target = I too low
    private static final double D_JITTER_LIMIT = 0.008;    // settled velocity std dev > 0.8% = D too high
    // ---------------------------------------------------------------

    private enum Mode { F, P, D, I }
    private enum Shot { WAIT_READY, ARMED, RECORDING }
    private enum Auto { IDLE, SETTLING, SAMPLING }          // the F hold test
    private enum Result { NONE, TOO_HIGH, TOO_LOW, GOOD }

    /** Binary search bracket. Heals itself if you nudge a value outside the bracket by hand. */
    private static class Search {
        final double minB, maxB;
        double lo, hi;
        int steps;

        Search(double minB, double maxB) {
            this.minB = minB;
            this.maxB = maxB;
            reset();
        }

        void reset() { lo = minB; hi = maxB; steps = 0; }
        double mid() { return (lo + hi) / 2.0; }
        double width() { return hi - lo; }

        void tooHigh(double cur) {
            hi = cur;
            if (lo >= hi) lo = minB;
            steps++;
        }

        void tooLow(double cur) {
            lo = cur;
            if (hi <= lo) hi = maxB;
            steps++;
        }
    }

    private DcMotorEx flywheel;
    private DcMotorEx feeder;   // may be null

    private final Search fSearch = new Search(F_MIN, F_MAX);
    private final Search pSearch = new Search(P_MIN, P_MAX);
    private final Search iSearch = new Search(I_MIN, I_MAX);
    private final Search dSearch = new Search(D_MIN, D_MAX);

    private Mode mode = Mode.F;

    private double targetVelocity = START_VELOCITY;
    private double P = START_P;
    private double I = START_I;
    private double D = START_D;
    private double F = START_F;
    private boolean flywheelOn = true;

    // what the hub was last told
    private double sentP = Double.NaN;
    private double sentI = Double.NaN;
    private double sentD = Double.NaN;
    private double sentF = Double.NaN;
    private double sentCmd = Double.NaN;

    // feeder pulse
    private final ElapsedTime pulseTimer = new ElapsedTime();
    private boolean pulsing = false;

    // F hold test
    private Auto auto = Auto.IDLE;
    private final ElapsedTime timer = new ElapsedTime();
    private double sampleSum = 0;
    private int sampleN = 0;
    private int fIter = 0;

    // shot monitor
    private final ElapsedTime clock = new ElapsedTime();   // never reset
    private Shot shot = Shot.WAIT_READY;
    private double readyStart = -1;
    private double shotStart = 0;
    private double minVel = 0, tMin = 0, maxVel = 0;
    private int crossings = 0, lastSign = 0;
    private double holdStart = -1;
    private double ssSum = 0, ssSumSq = 0;
    private int ssN = 0;

    // shot auto-adjust
    private boolean shotAuto = false;
    private int shotIter = 0;
    private int shotCount = 0;
    private String lastShotInfo = "no shot measured yet";
    private Result lastResult = Result.NONE;

    private String lastAction = "none";

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL_NAME);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        feeder = hardwareMap.tryGet(DcMotorEx.class, FEEDER_NAME);

        telemetry.addLine("Init complete");
        telemetry.addData("Feeder", feeder != null ? "found" : "not found (feed manually)");
        telemetry.update();
    }

    @Override
    public void loop() {
        double currVel = flywheel.getVelocity();

        // ----- feeder (off during the F hold test so it doesn't load the wheel) -----
        if (gamepad1.dpadUpWasPressed()) {
            pulsing = true;
            pulseTimer.reset();
        }
        if (pulsing && pulseTimer.seconds() >= FEED_PULSE_S) pulsing = false;
        boolean feed = (gamepad1.right_trigger > 0.5 || pulsing) && auto == Auto.IDLE;
        if (feeder != null) feeder.setPower(feed ? FEEDER_POWER : 0);

        // ----- buttons -----
        if (gamepad1.startWasPressed()) {
            flywheelOn = !flywheelOn;
            if (!flywheelOn) {
                cancelFTest("cancelled (flywheel off)");
                shotAuto = false;
            }
            resetShot();
        }

        if (gamepad1.crossWasPressed()) {
            cancelFTest("cancelled (mode changed)");
            shotAuto = false;
            mode = Mode.values()[(mode.ordinal() + 1) % Mode.values().length];
            resetShot();
        }

        if (auto == Auto.IDLE) {
            if (gamepad1.rightBumperWasPressed()) {
                targetVelocity = Math.min(VEL_MAX, targetVelocity + VEL_STEP);
                resetShot();
            }
            if (gamepad1.leftBumperWasPressed()) {
                targetVelocity = Math.max(VEL_MIN, targetVelocity - VEL_STEP);
                resetShot();
            }
            if (gamepad1.leftTriggerWasPressed()) {
                currentSearch().reset();
                lastAction = mode + " search bracket reset";
            }
        }

        if (gamepad1.backWasPressed()) {
            if (mode == Mode.F) {
                if (auto != Auto.IDLE) cancelFTest("cancelled");
                else startFTest();
            } else {
                toggleShotAuto();
            }
        }

        // manual verdicts (when nothing automatic is adjusting the value)
        if (auto == Auto.IDLE && !shotAuto) {
            if (gamepad1.triangleWasPressed()) applyVerdict(true);
            if (gamepad1.squareWasPressed()) applyVerdict(false);
            if (gamepad1.circleWasPressed()) lockValue();
        }

        // ----- F hold test and shot monitor -----
        runFTest(currVel);
        if (auto == Auto.IDLE && flywheelOn) runShotMonitor(currVel);

        // ----- send to the hub only when something changed (keeps the loop fast) -----
        // Each tuning mode only switches on the terms tuned before it:
        //   F: F only | P: F+P | D: F+P+D | I: everything
        double pSend = (mode == Mode.F) ? 0 : P;
        double dSend = (mode == Mode.D || mode == Mode.I) ? D : 0;
        double iSend = (mode == Mode.I) ? I : 0;
        if (pSend != sentP || iSend != sentI || dSend != sentD || F != sentF) {
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(pSend, iSend, dSend, F));
            sentP = pSend;
            sentI = iSend;
            sentD = dSend;
            sentF = F;
            resetShot();   // new coefficients, the next shot is a fresh measurement
        }

        double cmd = flywheelOn ? targetVelocity : 0;
        if (cmd != sentCmd) {
            flywheel.setVelocity(cmd);
            sentCmd = cmd;
        }

        double err = targetVelocity - currVel;

        // ----- telemetry -----
        String status;
        if (!flywheelOn) status = "flywheel OFF";
        else if (auto != Auto.IDLE) status = "F hold test: " + auto;
        else if (shot == Shot.ARMED) status = "READY, shoot";
        else if (shot == Shot.RECORDING) status = "measuring recovery...";
        else status = "wait, getting to speed";
        telemetry.addData("STATUS", status);
        telemetry.addData("Tuning", mode + "  (" + modeHint() + ")");
        if (mode != Mode.F) {
            telemetry.addData("Auto-adjust", shotAuto ? "ON, shoot a volley (iter " + (shotIter + 1) + "/" + MAX_AUTO_ITERS + ")" : "off");
        }
        telemetry.addData("Last", lastAction);
        telemetry.addData("Last shot #" + shotCount, lastShotInfo);
        if (mode != Mode.F) telemetry.addData("It says", lastResult);
        Search s = currentSearch();
        telemetry.addData("Search", "[%.4f , %.4f]  steps=%d  next=%.4f", s.lo, s.hi, s.steps, s.mid());
        telemetry.addLine("-----------------------");
        telemetry.addData("Target Velocity", "%.0f", targetVelocity);
        telemetry.addData("Current Velocity", "%.1f", currVel);
        telemetry.addData("Error", "%.1f", err);
        telemetry.addData("P", "%.3f", P);
        telemetry.addData("I", "%.3f", I);
        telemetry.addData("D", "%.3f", D);
        telemetry.addData("F", "%.4f", F);
        telemetry.addData("Active on hub", "P=%.3f I=%.3f D=%.3f F=%.4f", sentP, sentI, sentD, sentF);
        telemetry.addLine("-----------------------");
        telemetry.addLine("X: mode | Back: auto (F test / auto-adjust)");
        telemetry.addLine("Tri: too HIGH | Sq: too LOW | Circle: lock");
        telemetry.addLine("RB/LB: target vel | LT: reset bracket");
        telemetry.addLine("RT: feed | Dpad up: feed pulse | Start: flywheel");
        telemetry.update();
    }

    // ===================== search helpers =====================

    private Search currentSearch() {
        switch (mode) {
            case F: return fSearch;
            case P: return pSearch;
            case I: return iSearch;
            default: return dSearch;
        }
    }

    private double getValue() {
        switch (mode) {
            case F: return F;
            case P: return P;
            case I: return I;
            default: return D;
        }
    }

    private void setValue(double v) {
        switch (mode) {
            case F: F = Range.clip(v, F_MIN, F_MAX); break;
            case P: P = Range.clip(v, P_MIN, P_MAX); break;
            case I: I = Range.clip(v, I_MIN, I_MAX); break;
            default: D = Range.clip(v, D_MIN, D_MAX); break;
        }
    }

    private double resolution() {
        switch (mode) {
            case F: return F_RESOLUTION;
            case P: return P_RESOLUTION;
            case I: return I_RESOLUTION;
            default: return D_RESOLUTION;
        }
    }

    private String modeHint() {
        switch (mode) {
            case F: return "too HIGH = settles above target, too LOW = below";
            case P: return "too HIGH = overshoot/ringing, too LOW = slow recovery";
            case D: return "too HIGH = jittery, too LOW = ringing after shot";
            default: return "too HIGH = overshoot/windup, too LOW = ends below target";
        }
    }

    private void applyVerdict(boolean tooHigh) {
        Search s = currentSearch();
        double cur = getValue();
        if (tooHigh) s.tooHigh(cur);
        else s.tooLow(cur);
        setValue(s.mid());
        lastAction = String.format("%s at %.4f -> next %.4f  (bracket %.4f)",
                tooHigh ? "TOO HIGH" : "TOO LOW", cur, getValue(), s.width());
    }

    private void lockValue() {
        // Bracket is kept, so a later verdict keeps refining instead of restarting.
        lastAction = String.format("%s locked: P=%.3f I=%.3f D=%.3f F=%.4f", mode, P, I, D, F);
        RobotLog.ii(TAG, String.format("P=%.3f I=%.3f D=%.3f F=%.4f", P, I, D, F));
    }

    // ===================== shot measurement =====================

    private void resetShot() {
        shot = Shot.WAIT_READY;
        readyStart = -1;
    }

    private void toggleShotAuto() {
        if (shotAuto) {
            shotAuto = false;
            lastAction = "Auto-adjust off";
            return;
        }
        if (targetVelocity < 100) {
            lastAction = "Set a real target velocity first (bumpers)";
            return;
        }
        flywheelOn = true;
        shotAuto = true;
        shotIter = 0;

        Search s = currentSearch();
        s.reset();
        setValue(s.mid());   // the first probe is the middle of the full range
        lastAction = String.format("Auto-adjust %s on, first value %.3f. Shoot a volley.", mode, getValue());
    }

    private void runShotMonitor(double v) {
        double T = targetVelocity;
        if (T < 100) return;

        switch (shot) {
            case WAIT_READY:
                if (Math.abs(v - T) <= SETTLE_BAND * T) {
                    if (readyStart < 0) readyStart = clock.seconds();
                    if (clock.seconds() - readyStart >= READY_S) shot = Shot.ARMED;
                } else {
                    readyStart = -1;
                }
                break;

            case ARMED:
                if (v < T * (1 - DIP_TRIGGER)) beginRecording(v);
                break;

            case RECORDING:
                recordShot(v);
                break;
        }
    }

    private void beginRecording(double v) {
        shot = Shot.RECORDING;
        shotStart = clock.seconds();
        minVel = v;
        tMin = 0;
        maxVel = v;
        crossings = 0;
        lastSign = 0;
        holdStart = -1;
        ssSum = 0;
        ssSumSq = 0;
        ssN = 0;
    }

    private void recordShot(double v) {
        double T = targetVelocity;
        double t = clock.seconds() - shotStart;
        double e = v - T;

        // A deeper dip (next ball of the volley) restarts the "after the dip" stats.
        if (v < minVel) {
            minVel = v;
            tMin = t;
            maxVel = v;
            crossings = 0;
            lastSign = 0;
        }
        maxVel = Math.max(maxVel, v);

        // Count crossings of the target (outside a deadband so noise doesn't count).
        // Starting below target, the first crossing is the overshoot, the second is a
        // swing back under, which means ringing.
        double dead = OSC_DEADBAND * T;
        int sign = e > dead ? 1 : (e < -dead ? -1 : 0);
        if (sign != 0) {
            if (lastSign != 0 && sign != lastSign) crossings++;
            lastSign = sign;
        }

        // Recovered = back in the band and staying there for HOLD_S.
        if (Math.abs(e) <= SETTLE_BAND * T) {
            if (holdStart < 0) {
                holdStart = t;
                ssSum = 0;
                ssSumSq = 0;
                ssN = 0;
            }
            ssSum += v;
            ssSumSq += v * v;
            ssN++;
            if (t - holdStart >= HOLD_S) {
                finishShot(false);
                return;
            }
        } else {
            holdStart = -1;
        }

        if (t >= RECOVERY_TIMEOUT_S) finishShot(true);
    }

    private void finishShot(boolean timedOut) {
        double T = targetVelocity;
        double dip = (T - minVel) / T;
        double recovery = timedOut ? (RECOVERY_TIMEOUT_S - tMin) : (holdStart - tMin);
        double overshoot = (maxVel - T) / T;
        double ssMean = ssN > 0 ? ssSum / ssN : T;
        double ssErr = (ssMean - T) / T;
        double var = ssN > 1 ? Math.max(0, ssSumSq / ssN - ssMean * ssMean) : 0;
        double jitter = Math.sqrt(var) / T;
        boolean ringing = overshoot > OVERSHOOT_LIMIT || crossings >= OSC_CROSSINGS;

        shotCount++;
        lastShotInfo = String.format("dip=%.1f%%  recover=%s  over=%.1f%%  cross=%d  ssErr=%+.1f%%  jitter=%.2f%%",
                dip * 100,
                timedOut ? "NEVER" : String.format("%.2fs", recovery),
                overshoot * 100, crossings, timedOut ? 0.0 : ssErr * 100, timedOut ? 0.0 : jitter * 100);

        // Decide too HIGH / too LOW for the term being tuned
        boolean high = false;
        boolean low = false;
        switch (mode) {
            case P:
                high = ringing;
                low = !high && (timedOut || recovery > RECOVERY_LIMIT_S);
                break;
            case I:
                high = ringing || (!timedOut && ssErr > I_SS_TOL);
                low = !high && (timedOut || ssErr < -I_SS_TOL);
                break;
            case D:
                high = !timedOut && jitter > D_JITTER_LIMIT;
                low = !high && ringing;
                break;
            default:
                break;   // F is tuned with the hold test, shots are just displayed
        }

        if (mode == Mode.F) lastResult = Result.NONE;
        else if (high) lastResult = Result.TOO_HIGH;
        else if (low) lastResult = Result.TOO_LOW;
        else lastResult = Result.GOOD;

        if (shotAuto) adjustFromShot(lastResult);
        resetShot();
    }

    private void adjustFromShot(Result r) {
        Search s = currentSearch();
        double cur = getValue();

        if (r == Result.GOOD) {
            shotAuto = false;
            lastAction = String.format("%s locked at %.3f (good recovery)", mode, cur);
            RobotLog.ii(TAG, lastAction);
            return;
        }

        if (r == Result.TOO_HIGH) s.tooHigh(cur);
        else s.tooLow(cur);
        shotIter++;

        if (s.width() < resolution() || shotIter >= MAX_AUTO_ITERS) {
            setValue(s.lo);   // the stable / quiet side of the bracket
            shotAuto = false;
            lastAction = String.format("%s search done, using stable side %.3f", mode, getValue());
            RobotLog.ii(TAG, lastAction);
            return;
        }

        setValue(s.mid());
        lastAction = String.format("%s was %s at %.3f -> trying %.3f", mode, r, cur, getValue());
    }

    // ===================== F hold test =====================

    private void startFTest() {
        if (targetVelocity < 100) {
            lastAction = "Set a real target velocity first (bumpers)";
            return;
        }
        flywheelOn = true;
        shotAuto = false;
        fIter = 0;

        fSearch.reset();
        F = Range.clip(fSearch.mid(), F_MIN, F_MAX);
        auto = Auto.SETTLING;
        timer.reset();
        lastAction = "F hold test started";
    }

    private void cancelFTest(String why) {
        if (auto != Auto.IDLE) {
            auto = Auto.IDLE;
            lastAction = "F hold test " + why;
            resetShot();
        }
    }

    private void runFTest(double currVel) {
        switch (auto) {
            case IDLE:
                return;

            case SETTLING:
                if (timer.seconds() >= F_SETTLE_S) {
                    auto = Auto.SAMPLING;
                    sampleSum = 0;
                    sampleN = 0;
                    timer.reset();
                }
                break;

            case SAMPLING:
                sampleSum += currVel;
                sampleN++;
                if (timer.seconds() >= F_SAMPLE_S) finishFTest();
                break;
        }
    }

    private void finishFTest() {
        double avg = sampleSum / Math.max(1, sampleN);
        double errFrac = (avg - targetVelocity) / targetVelocity;
        String note = String.format("F=%.4f  avg=%.0f  (%+.2f%% vs target)", F, avg, errFrac * 100);
        lastShotInfo = note;

        if (Math.abs(errFrac) <= F_TOLERANCE) {
            endFTest(String.format("F locked at %.4f (within %.1f%%)", F, F_TOLERANCE * 100));
            return;
        }

        if (errFrac > 0) fSearch.tooHigh(F);
        else fSearch.tooLow(F);
        fIter++;
        F = Range.clip(fSearch.mid(), F_MIN, F_MAX);

        if (fSearch.width() < F_RESOLUTION || fIter >= MAX_AUTO_ITERS) {
            endFTest(String.format("F search done, using %.4f", F));
            return;
        }

        // next candidate
        auto = Auto.SETTLING;
        timer.reset();
    }

    private void endFTest(String msg) {
        auto = Auto.IDLE;
        lastAction = msg;
        RobotLog.ii(TAG, msg);
        resetShot();
    }

    @Override
    public void stop() {
        flywheel.setVelocity(0);
        if (feeder != null) feeder.setPower(0);
        RobotLog.ii(TAG, String.format("FINAL  P=%.3f I=%.3f D=%.3f F=%.4f", P, I, D, F));
    }
}
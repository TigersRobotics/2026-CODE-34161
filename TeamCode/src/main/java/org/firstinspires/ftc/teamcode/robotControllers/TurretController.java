package org.firstinspires.ftc.teamcode.robotControllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.utils.ShotUtil;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;

public class TurretController {
    private static boolean isInitialized;
    private static DcMotorEx flywheel;
    private static DcMotorEx rotation;
    private static Servo hoodServo;

    private static final long VELOCITY_RESEND_NS = 500_000_000L;

    private static double hoodPosition;
    private static double turretYaw;
    private static double flywheelRPM;

    private static int sentYawTicks = Integer.MIN_VALUE;
    private static double sentTicksPerSecond = Double.NaN;
    private static double sentHood = Double.NaN;
    private static double sentP = Double.NaN;
    private static double sentI = Double.NaN;
    private static double sentD = Double.NaN;
    private static double sentF = Double.NaN;
    private static long velocitySentAtNs;

    public static void setYaw(double degrees, double perpendicularVelocity, double ballSpeed,
            double trajectoryDegrees, double weight) {
        if (Double.isNaN(degrees)) return;
        degrees -= leadDegrees(perpendicularVelocity, ballSpeed, trajectoryDegrees, weight);
        degrees %= 360;
        if (degrees > 180) degrees -= 360;
        if (degrees < -180) degrees += 360;
        turretYaw = Math.round(degrees / TurretConstants.YAW_STEP_DEG) * TurretConstants.YAW_STEP_DEG;
    }

    public static void setYawFieldCentric(double fieldDegrees, double robotHeadingDegrees,
            double perpendicularVelocity, double ballSpeed, double trajectoryDegrees, double weight) {
        setYaw(fieldDegrees - robotHeadingDegrees, perpendicularVelocity, ballSpeed, trajectoryDegrees, weight);
    }

    private static double leadDegrees(double perpendicularVelocity, double ballSpeed,
            double trajectoryDegrees, double weight) {
        double horizontalSpeed = ballSpeed * Math.cos(Math.toRadians(trajectoryDegrees));
        if (Double.isNaN(perpendicularVelocity) || Double.isNaN(horizontalSpeed) || Math.abs(horizontalSpeed) < 1e-6) {
            return 0;
        }
        double ratio = weight * perpendicularVelocity / horizontalSpeed;
        ratio = Math.max(-1, Math.min(1, ratio));
        return Math.toDegrees(Math.asin(ratio));
    }

    public static void setRpm(double rpm) {
        if (Double.isNaN(rpm)) return;
        rpm = Math.max(TurretConstants.FLYWHEEL_RPM_MIN, Math.min(TurretConstants.FLYWHEEL_RPM_MAX, rpm));
        flywheelRPM = rpm;
    }

    public static void setHood(double degrees) {
        if (Double.isNaN(degrees)) return;
        degrees = Math.max(TurretConstants.HOOD_MIN_DEG, Math.min(TurretConstants.HOOD_MAX_DEG, degrees));
        hoodPosition = Math.round(degrees / TurretConstants.HOOD_STEP_DEG) * TurretConstants.HOOD_STEP_DEG;
    }

    public static double getRpm() {
        if (flywheel == null) return 0;
        return flywheel.getVelocity() * 60.0 / TurretConstants.FLYWHEEL_TICKS_PER_REV;
    }

    public static double getYaw() {
        if (rotation == null) return 0;
        return rotation.getCurrentPosition() * 360.0 / TurretConstants.YAW_TICKS_PER_REV; // deg
    }

    public static double getTargetYaw() {
        return turretYaw; // deg
    }

    public static void init(HardwareMap hardwareMap, boolean actuallyEnabled) {
        ShotUtil.reset();
        if(actuallyEnabled) {
            flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setDirection(DcMotor.Direction.FORWARD);

            rotation = hardwareMap.get(DcMotorEx.class, "yawMotor");
            rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretYaw = 0;
            rotation.setTargetPosition(0);
            rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotation.setPower(1);

            hoodServo = hardwareMap.get(Servo.class, "turretUp");

            sentYawTicks = 0;
            sentTicksPerSecond = Double.NaN;
            sentHood = Double.NaN;
            sentP = Double.NaN;
            sentI = Double.NaN;
            sentD = Double.NaN;
            sentF = Double.NaN;
            velocitySentAtNs = 0;

            isInitialized = true;
            TimedTelemetry.addLine("Turret Controller Initialized");
        } 
        else stop();
    }

    public static void init(HardwareMap hardwareMap) {
        init(hardwareMap, true);
    }

    public static void update() {
        if(!isInitialized) {
            TimedTelemetry.addLine("Turret updating but not initialized");
            return;
        }

        if (TurretConstants.FLYWHEEL_P != sentP
                || TurretConstants.FLYWHEEL_I != sentI
                || TurretConstants.FLYWHEEL_D != sentD
                || TurretConstants.FLYWHEEL_F != sentF) {
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(
                            TurretConstants.FLYWHEEL_P,
                            TurretConstants.FLYWHEEL_I,
                            TurretConstants.FLYWHEEL_D,
                            TurretConstants.FLYWHEEL_F));
            sentP = TurretConstants.FLYWHEEL_P;
            sentI = TurretConstants.FLYWHEEL_I;
            sentD = TurretConstants.FLYWHEEL_D;
            sentF = TurretConstants.FLYWHEEL_F;
        }

        int yawTicks = (int) Math.round(turretYaw * TurretConstants.YAW_TICKS_PER_REV / 360.0);
        if (yawTicks != sentYawTicks) {
            rotation.setTargetPosition(yawTicks);
            sentYawTicks = yawTicks;
        }

        long now = System.nanoTime();
        double ticksPerSecond = flywheelRPM * TurretConstants.FLYWHEEL_TICKS_PER_REV / 60.0;
        if (ticksPerSecond != sentTicksPerSecond || now - velocitySentAtNs > VELOCITY_RESEND_NS) {
            flywheel.setVelocity(ticksPerSecond);
            sentTicksPerSecond = ticksPerSecond;
            velocitySentAtNs = now;
        }

        double servo = (hoodPosition - TurretConstants.HOOD_MIN_DEG)
                / (TurretConstants.HOOD_MAX_DEG - TurretConstants.HOOD_MIN_DEG);
        servo = Math.max(0, Math.min(1, servo));
        if (servo != sentHood) {
            hoodServo.setPosition(servo);
            sentHood = servo;
        }
    }

    public static void stop() {
        if (flywheel != null) flywheel.setPower(0);
        if (rotation != null) rotation.setPower(0);

        flywheel = null;
        rotation = null;
        hoodServo = null;
        isInitialized = false;

        flywheelRPM = 0;
        turretYaw = 0;
        hoodPosition = TurretConstants.HOOD_MIN_DEG;
        sentYawTicks = Integer.MIN_VALUE;
        sentTicksPerSecond = Double.NaN;
        sentHood = Double.NaN;
        sentP = Double.NaN;
        sentI = Double.NaN;
        sentD = Double.NaN;
        sentF = Double.NaN;
    }
}

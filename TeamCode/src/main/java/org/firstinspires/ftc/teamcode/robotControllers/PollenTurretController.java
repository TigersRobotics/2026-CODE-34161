package org.firstinspires.ftc.teamcode.robotControllers;

import static org.firstinspires.ftc.teamcode.constants.HardwareNames.FLYWHEEL_NAME;
import static org.firstinspires.ftc.teamcode.constants.HardwareNames.TURRET_YAW_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.constants.RobotSpecs.MAX_YAW;
import static org.firstinspires.ftc.teamcode.constants.RobotSpecs.MIN_YAW;
import static org.firstinspires.ftc.teamcode.constants.RobotSpecs.TICKS_PER_REV;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.constants.TuningConstants;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.UtilMethods;


/**
 * Controls the flywheel, the turret yaw motor and the hood servo.
 * <p>
 * Every command sent to a hub costs a few milliseconds of loop time, so this class remembers the
 * last value it sent to each device and only talks to the hub when that value actually changes.
 */
public class PollenTurretController {
    /// Encoder ticks per degree of turret yaw, worked out once instead of every loop
    private static final double TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;
    /// Degrees of hood travel that map onto the full range of the hood servo
    private static final double HOOD_SERVO_RANGE_DEG = 20;
    /// An unchanged target is still re-sent this often, so a hub that reset mid-match gets it back
    private static final long RESEND_INTERVAL_NS = 500_000_000L;

    private static boolean isInitialized;
    private static DcMotorEx flywheel;
    private static DcMotorEx rotation;
    private static Servo hoodServo;

    private static double P = TuningConstants.FLYWHEEL_P_VALUE;
    private static double I = TuningConstants.FLYWHEEL_I_VALUE;
    private static double D = TuningConstants.FLYWHEEL_D_VALUE;
    private static double F = TuningConstants.FLYWHEEL_F_VALUE;
    private static double targetVel = 0;
    private static double yaw = 0; //This is in degrees

    // The last values actually sent to the hardware. NaN / MIN_VALUE mean "nothing sent yet".
    private static double sentP = Double.NaN;
    private static double sentI = Double.NaN;
    private static double sentD = Double.NaN;
    private static double sentF = Double.NaN;
    private static double sentVel = Double.NaN;
    private static int sentYawTicks = Integer.MIN_VALUE;
    private static double sentHoodPosition = Double.NaN;
    private static long velSentAtNs;
    private static long yawSentAtNs;

    private static boolean updateYaw = false;
    private static boolean updateVel = false;
    private static double flywheelVel = 0;

    public static void init(HardwareMap hardwareMap, boolean actuallyEnabled) {
        if(actuallyEnabled) {
            flywheel = (DcMotorEx) hardwareMap.get(DcMotor.class, FLYWHEEL_NAME);
            flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheel.setDirection(DcMotor.Direction.FORWARD);

            rotation = (DcMotorEx) hardwareMap.get(DcMotor.class, TURRET_YAW_MOTOR_NAME);
            rotation.setTargetPosition(0);
            rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION); ///Clockwise should be positive encoder position
            rotation.setDirection(DcMotor.Direction.FORWARD);
            rotation.setPower(1.0);

            hoodServo = hardwareMap.get(Servo.class, HardwareNames.TURRET_UP_ANGLE_SERVO_NAME);

            // The statics survive between OpMode runs but the hub's state does not, so forget
            // everything that was sent last time and send the coefficients again.
            forgetSentValues();
            applyPFCoefficients();

            isInitialized = true;

            TimedTelemetry.addLine("Pollen Turret Controller Initialized! :D", 1000);
            updateYaw = false;
            updateVel = false;
        } else stop();
    }

    public static void init(HardwareMap hardwareMap) {
        init(hardwareMap, true);
    }


    public static void update() {
        if(!isInitialized) {
            TimedTelemetry.addLine("Pollen Turret Updating but not Initialized! >:(");
            return;
        }

        long now = System.nanoTime();

        // setVelocity() is two hub commands every time it is called, so skip it while the target is unchanged
        if(targetVel != sentVel || now - velSentAtNs > RESEND_INTERVAL_NS) {
            flywheel.setVelocity(targetVel);
            sentVel = targetVel;
            velSentAtNs = now;
        }

        // The yaw motor only understands whole ticks, so a change smaller than a tick is not worth a command
        int yawTicks = (int) Math.round(yaw * TICKS_PER_DEGREE);
        if(yawTicks != sentYawTicks || now - yawSentAtNs > RESEND_INTERVAL_NS) {
            rotation.setTargetPosition(yawTicks);
            sentYawTicks = yawTicks;
            yawSentAtNs = now;
        }

        TimedTelemetry.addData("Target Position: ", yaw);
        TimedTelemetry.addData("Current Flywheel Speed: ", flywheel.getVelocity());
    }


    ///Sets the angle of the turret in degrees
    public static void setYaw(double yaw) {
        if(Double.isNaN(yaw)) return; // a bad odometry reading must not swing the turret back to 0
        updateYaw = true;
        PollenTurretController.yaw = UtilMethods.clip(yaw, MIN_YAW, MAX_YAW);
    }

    public static void setTargetVel(double targetVel) {
        updateVel = true;
        PollenTurretController.targetVel = targetVel;
    }

    /// Only talks to the hub when the coefficients are different to the ones it already has
    public static void setPFCoefficients(double P, double F) {
        if(isInitialized) {
            PollenTurretController.P = P;
            PollenTurretController.F = F;
            applyPFCoefficients();
        }
    }

    public static double getTargetVel() {
        return targetVel;
    }
    public static double getVelocity() {
        return flywheel == null ? 0 : flywheel.getVelocity();
    }

    public static double getCurrentYaw() {
        return rotation == null ? 0 : rotation.getCurrentPosition() / TICKS_PER_DEGREE;
    }
    public static void stop() {
        if(flywheel != null) flywheel.setPower(0);
        if(rotation != null) rotation.setPower(0);
        if(hoodServo != null) hoodServo.setPosition(0);
        flywheel = null;
        rotation = null;
        hoodServo = null;
        yaw = 0;
        targetVel = 0;
        forgetSentValues();
        isInitialized = false;
    }

    /// Sets the hood angle in degrees. NaN (no valid shot) keeps the hood where it is.
    public static void setHoodAngle(double degrees) {
        if(hoodServo == null || Double.isNaN(degrees)) return;


        double position = UtilMethods.clip((degrees - HOOD_SERVO_RANGE_DEG/2 - 5) / HOOD_SERVO_RANGE_DEG * 2, -1, 1);
        if(position != sentHoodPosition) {
            hoodServo.setPosition(position);
            sentHoodPosition = position;
        }
    }

    private static void applyPFCoefficients() {
        if(P == sentP && I == sentI && D == sentD && F == sentF) return;

        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
        sentP = P;
        sentI = I;
        sentD = D;
        sentF = F;
    }

    private static void forgetSentValues() {
        sentP = Double.NaN;
        sentI = Double.NaN;
        sentD = Double.NaN;
        sentF = Double.NaN;
        sentVel = Double.NaN;
        sentYawTicks = Integer.MIN_VALUE;
        sentHoodPosition = Double.NaN;
    }
}

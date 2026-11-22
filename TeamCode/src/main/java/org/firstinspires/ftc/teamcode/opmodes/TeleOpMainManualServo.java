package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.constants.RobotSpecs;
import org.firstinspires.ftc.teamcode.constants.TuningConstants;
import org.firstinspires.ftc.teamcode.robotControllers.MecanumDriveTrainController;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;
import org.firstinspires.ftc.teamcode.utils.UtilMethods;

@TeleOp(name = "TeleOpMain manual servo")
@Disabled
@Deprecated
public class TeleOpMainManualServo extends OpMode {
    static final double STICK_DEADBAND = 0.08;
    static final double HOOD_PER_MS = 0.0005;
    static final double FLYWHEEL_STEP = 150; // ticks/s
    static final double FLYWHEEL_MAX = 2800; // ticks/s

    DcMotorEx yawMotor;
    DcMotorEx flywheel;
    Servo hoodServo;

    double hood = 0.2;
    double flywheelTicks;
    double sentFlywheel = Double.NaN;
    long flywheelSentAtNs;

    @Override
    public void init() {
        Timer.init();
        TimedTelemetry.init(telemetry);

        MecanumDriveTrainController.init(hardwareMap, true, true);

        yawMotor = hardwareMap.get(DcMotorEx.class, HardwareNames.TURRET_YAW_MOTOR_NAME);
        yawMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yawMotor.setDirection(DcMotor.Direction.FORWARD);
        yawMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        yawMotor.setPower(0);

        flywheel = hardwareMap.get(DcMotorEx.class, HardwareNames.FLYWHEEL_NAME);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                TuningConstants.FLYWHEEL_P_VALUE,
                TuningConstants.FLYWHEEL_I_VALUE,
                TuningConstants.FLYWHEEL_D_VALUE,
                TuningConstants.FLYWHEEL_F_VALUE));

        hoodServo = hardwareMap.get(Servo.class, HardwareNames.TURRET_UP_ANGLE_SERVO_NAME);
        hoodServo.setPosition(hood);
    }

    @Override
    public void loop() {
        Timer.update();

        MecanumDriveTrainController.update(
                UtilMethods.expo(gamepad1.left_stick_y, 2),
                UtilMethods.expo(-gamepad1.left_stick_x, 2),
                UtilMethods.expo(-gamepad1.right_stick_x, 2),
                0);

        double spin = gamepad2.left_stick_x;
        if (Math.abs(spin) < STICK_DEADBAND) spin = 0;
        yawMotor.setPower(spin);

        double hoodStick = -gamepad2.right_stick_y;
        if (Math.abs(hoodStick) < STICK_DEADBAND) hoodStick = 0;
        hood += hoodStick * Timer.DTime() * HOOD_PER_MS;
        if (hood < 0) hood = 0;
        if (hood > 1) hood = 1;
        hoodServo.setPosition(hood);

        if (gamepad2.dpadUpWasPressed()) flywheelTicks += FLYWHEEL_STEP;
        if (gamepad2.dpadDownWasPressed()) flywheelTicks -= FLYWHEEL_STEP;
        if (flywheelTicks < 0) flywheelTicks = 0;
        if (flywheelTicks > FLYWHEEL_MAX) flywheelTicks = FLYWHEEL_MAX;

        long now = System.nanoTime();
        if (flywheelTicks != sentFlywheel || now - flywheelSentAtNs > 500_000_000L) {
            flywheel.setVelocity(flywheelTicks);
            sentFlywheel = flywheelTicks;
            flywheelSentAtNs = now;
        }

        double rpm = flywheel.getVelocity() * 60.0 / RobotSpecs.FLYWHEEL_TICKS_PER_REV;
        TimedTelemetry.addData("turret power", "%.2f", 1, spin);
        TimedTelemetry.addData("hood", "%.3f", 1, hood);
        TimedTelemetry.addData("flywheel target", "%.0f ticks/s", 1, flywheelTicks);
        TimedTelemetry.addData("flywheel rpm", "%.0f", 1, rpm);
        TimedTelemetry.addLine("g2 left stick turret | g2 right stick hood | g2 dpadU/D flywheel | g1 drive");
        TimedTelemetry.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        yawMotor.setPower(0);
        flywheel.setPower(0);
        MecanumDriveTrainController.stop();
    }
}

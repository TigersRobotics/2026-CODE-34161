package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.constants.HardwareNames.ODOMETRY_NAME;
import static org.firstinspires.ftc.teamcode.constants.HardwareNames.TURRET_YAW_MOTOR_NAME;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.RobotSpecs;
import org.firstinspires.ftc.teamcode.constants.SensingConstants;
import org.firstinspires.ftc.teamcode.robotControllers.MecanumDriveTrainController;
import org.firstinspires.ftc.teamcode.sensing.HiveSensingCore;
import org.firstinspires.ftc.teamcode.sensing.RobotSensing;
import org.firstinspires.ftc.teamcode.sensors.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;
import org.firstinspires.ftc.teamcode.utils.UtilMethods;

/**
 * Drive around and watch what the sensing system thinks.
 * <p>
 * WHAT TO CHECK
 * <ul>
 * <li><b>Hive state:</b> tip a hive by hand. RED/BLUE hive should flip within about half a second
 *     and the tip count should go up by exactly one.</li>
 * <li><b>Position:</b> park on a known spot and compare "Pose" with a tape measure.</li>
 * <li><b>Drift recovery:</b> lift the robot, put it down a foot away. "Camera fixes" should show
 *     rejections, then one re-lock, and the pose should jump to the right place.</li>
 * <li><b>Noise:</b> sit still looking at the hive. The "in from fused" number is the camera's
 *     scatter, which is what VISION_SIGMA_... in SensingConstants should roughly match.</li>
 * <li><b>Speed:</b> "Loop" is the whole loop time in milliseconds.</li>
 * </ul>
 * Controls: sticks drive, TRIANGLE (Y) swaps the alliance we are shooting for. The turret motor is
 * left unpowered so the turret can be turned by hand; it must point straight ahead at INIT.
 */
@TeleOp(name = "Sensing Test", group = "Sensing")
public class SensingTest extends OpMode {
    private static final boolean ENABLE_DRIVE = true;
    private static final String[] CLUSTER_NAMES = { "RED SCORING", "RED AUDIENCE", "BLUE AUDIENCE", "BLUE SCORING" };

    private GoBildaOdometry odometry;
    private DcMotorEx turretEncoder;

    @Override
    public void init() {
        Timer.init();
        TimedTelemetry.init(telemetry);

        if (ENABLE_DRIVE) MecanumDriveTrainController.init(hardwareMap, true);
        odometry = new GoBildaOdometry(hardwareMap, ODOMETRY_NAME);
        turretEncoder = openTurretEncoder();

        RobotSensing.init(hardwareMap);
        RobotSensing.setAlliance(RobotSensing.Alliance.RED);

        TimedTelemetry.update();
        telemetry.update();
    }

    @Override
    public void loop() {
        Timer.update();
        odometry.update();

        Pose2D pose = odometry.getPose();
        double turretDeg = turretEncoder == null ? 0
                : turretEncoder.getCurrentPosition() * 360.0 / RobotSpecs.TICKS_PER_REV;

        RobotSensing.update(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.RADIANS), turretDeg);

        if (gamepad1.triangleWasPressed()) {
            RobotSensing.setAlliance(RobotSensing.getAlliance() == RobotSensing.Alliance.RED
                    ? RobotSensing.Alliance.BLUE : RobotSensing.Alliance.RED);
        }

        if (ENABLE_DRIVE) {
            MecanumDriveTrainController.update(UtilMethods.expo(gamepad1.left_stick_y, 2),
                    UtilMethods.expo(-gamepad1.left_stick_x, 2.0), UtilMethods.expo(-gamepad1.right_stick_x, 2.0));
        }

        telemetry.addData("Loop", "%.1f ms", Timer.DTime());
        telemetry.addData("Odometry only", "%.1f, %.1f, %.1f   turret %.1f",
                pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.DEGREES), turretDeg);
        RobotSensing.addTelemetry(telemetry);

        // What the camera makes of each cluster it can currently see
        HiveSensingCore core = RobotSensing.getCore();
        long now = System.nanoTime();
        for (int c = 0; c < 4; c++) {
            if (!core.isSeenWithin(c, now, 300)) continue;
            telemetry.addData(CLUSTER_NAMES[c], "h %.1f in, cue %.2f, score %.2f, %.0f in away, %d%%",
                    core.getMeasuredHeight(c), core.getMeasuredTiltCue(c), core.getMeasuredScore(c),
                    core.getMeasuredRange(c), core.getMeasuredPercent(c));
        }

        TimedTelemetry.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        RobotSensing.stop();
        if (ENABLE_DRIVE) MecanumDriveTrainController.stop();
    }

    /** Reads the turret's encoder without powering the motor, so the turret can be turned by hand. */
    private DcMotorEx openTurretEncoder() {
        if (!SensingConstants.CAMERA_ON_TURRET) return null;
        try {
            DcMotorEx motor = (DcMotorEx) hardwareMap.get(DcMotor.class, TURRET_YAW_MOTOR_NAME);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0);
            return motor;
        } catch (RuntimeException e) {
            TimedTelemetry.addLine("No turret motor found: assuming the turret points straight ahead.", 5000);
            return null;
        }
    }
}

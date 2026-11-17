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
import org.firstinspires.ftc.teamcode.sensing.HiveSensingCore;
import org.firstinspires.ftc.teamcode.sensing.RobotSensing;
import org.firstinspires.ftc.teamcode.sensors.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;

/**
 * Measures where the hive's AprilTag clusters are, so the numbers can be typed into
 * SensingConstants.CLUSTER_WHEN_UP and CLUSTER_WHEN_DOWN.
 * <p>
 * BEFORE RUNNING: fill in sections 1 and 2 of SensingConstants (the camera and where it is mounted).
 * This OpMode can only be as accurate as those numbers.
 * <p>
 * STEPS
 * <ol>
 * <li>Point the turret straight ahead and put the robot exactly at the pose in section 6 of
 *     SensingConstants (CALIBRATION_ROBOT_...), where the camera can see the hive. Do not move it.</li>
 * <li>Press INIT, then START. Wait for the sample count to pass about 100.</li>
 * <li>Each cluster the camera can see gets a line like <code>{ 71.2, 60.4, 31.8, 0.42 }</code>.
 *     Look at the hive: if that cluster's cell is the one pointing UP, type the line into
 *     CLUSTER_WHEN_UP. If it is DOWN, type it into CLUSTER_WHEN_DOWN.</li>
 * <li>Tip the hive over by hand, press CROSS (A) to start the averages again, and repeat.</li>
 * <li>Move the robot to see the other hive or the far-side cells. Change the CALIBRATION_ROBOT_...
 *     numbers to the new spot, rebuild, and repeat until all 8 rows are filled.</li>
 * </ol>
 * The turret may be turned by hand while this runs (its motor is left unpowered), as long as it was
 * pointing straight ahead when INIT was pressed.
 */
@TeleOp(name = "Hive Tag Calibration", group = "Sensing")
public class HiveTagCalibration extends OpMode {
    private static final String[] CLUSTER_NAMES = { "0 RED SCORING", "1 RED AUDIENCE", "2 BLUE AUDIENCE", "3 BLUE SCORING" };

    private GoBildaOdometry odometry;
    private DcMotorEx turretEncoder;

    // Running averages for each cluster
    private final double[] sumX = new double[4];
    private final double[] sumY = new double[4];
    private final double[] sumZ = new double[4];
    private final double[] sumCue = new double[4];
    private final int[] samples = new int[4];
    private final long[] lastSeenNs = new long[4];

    @Override
    public void init() {
        TimedTelemetry.init(telemetry);

        odometry = new GoBildaOdometry(hardwareMap, ODOMETRY_NAME);
        odometry.setPose(new Pose2D(DistanceUnit.INCH,
                SensingConstants.CALIBRATION_ROBOT_X_IN, SensingConstants.CALIBRATION_ROBOT_Y_IN,
                AngleUnit.DEGREES, SensingConstants.CALIBRATION_ROBOT_HEADING_DEG));

        turretEncoder = openTurretEncoder();
        RobotSensing.init(hardwareMap);

        telemetry.addLine("Turret straight ahead, robot at the calibration pose, then press START.");
        telemetry.update();
    }

    @Override
    public void loop() {
        odometry.update();
        Pose2D pose = odometry.getPose();
        double turretDeg = turretEncoder == null ? 0
                : turretEncoder.getCurrentPosition() * 360.0 / RobotSpecs.TICKS_PER_REV;

        RobotSensing.update(pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.RADIANS), turretDeg);

        if (gamepad1.crossWasPressed()) {
            for (int c = 0; c < 4; c++) {
                sumX[c] = sumY[c] = sumZ[c] = sumCue[c] = 0;
                samples[c] = 0;
            }
        }

        HiveSensingCore core = RobotSensing.getCore();
        telemetry.addData("Robot (in, deg)", "%.1f, %.1f, %.1f   turret %.1f",
                pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), pose.getHeading(AngleUnit.DEGREES), turretDeg);
        if (!RobotSensing.isCameraWorking()) telemetry.addLine("CAMERA NOT WORKING - check the name in SensingConstants");
        telemetry.addLine("CROSS (A) = start the averages again");
        telemetry.addLine("Rows are { X, Y, height, tilt cue }");

        for (int c = 0; c < 4; c++) {
            long seen = core.getSeenNs(c);
            if (seen != lastSeenNs[c] && seen != Long.MIN_VALUE) {
                lastSeenNs[c] = seen;
                sumX[c] += core.getMeasuredX(c);
                sumY[c] += core.getMeasuredY(c);
                sumZ[c] += core.getMeasuredHeight(c);
                sumCue[c] += core.getMeasuredTiltCue(c);
                samples[c]++;
            }

            if (samples[c] == 0) {
                telemetry.addData(CLUSTER_NAMES[c], "not seen");
            } else {
                int n = samples[c];
                telemetry.addData(CLUSTER_NAMES[c], "{ %.2f, %.2f, %.2f, %.3f },  %d samples, %d%% of tags, %.0f in away",
                        sumX[c] / n, sumY[c] / n, sumZ[c] / n, sumCue[c] / n,
                        n, core.getMeasuredPercent(c), core.getMeasuredRange(c));
            }
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        RobotSensing.stop();
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
            telemetry.addLine("No turret motor found: assuming the turret points straight ahead.");
            return null;
        }
    }
}

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.robotControllers.MecanumDriveTrainController;
import org.firstinspires.ftc.teamcode.robotControllers.TurretController;
import org.firstinspires.ftc.teamcode.sensors.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.utils.ShotUtil;
import org.firstinspires.ftc.teamcode.utils.TimedTelemetry;
import org.firstinspires.ftc.teamcode.utils.Timer;
import org.firstinspires.ftc.teamcode.utils.UtilMethods;

import java.util.List;

@TeleOp(name = "Drive Tracking Test")
public class DriveTrackingTest extends OpMode {
    static final double START_HEADING_DEG = 90; // deg

    GoBildaOdometry odometry;
    List<LynxModule> hubs;
    boolean flywheelOn;
    boolean movingComp;
    boolean needZero = true;

    @Override
    public void init() {
        Timer.init();
        TimedTelemetry.init(telemetry);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        TurretController.init(hardwareMap);
        MecanumDriveTrainController.init(hardwareMap);
        ShotUtil.reset();
        odometry = new GoBildaOdometry(hardwareMap, HardwareNames.ODOMETRY_NAME);
        zeroPose();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        Timer.update();

        if (gamepad1.aWasPressed()) flywheelOn = !flywheelOn;
        if (gamepad1.bWasPressed()) movingComp = !movingComp;

        odometry.update();
        if (needZero && odometry.getStatus() != GoBildaPinpointDriver.DeviceStatus.NOT_READY) {
            zeroPose();
            needZero = false;
        }
        if (gamepad1.yWasPressed()) zeroPose();

        Pose2D pose = odometry.getPose();
        double x = pose.getX(DistanceUnit.METER); // m
        double y = pose.getY(DistanceUnit.METER); // m
        double heading = pose.getHeading(AngleUnit.DEGREES); // deg
        double vx = odometry.getVelX(DistanceUnit.METER); // m/s
        double vy = odometry.getVelY(DistanceUnit.METER); // m/s

        MecanumDriveTrainController.update(
                UtilMethods.expo(gamepad1.left_stick_y, 2),
                UtilMethods.expo(-gamepad1.left_stick_x, 2),
                UtilMethods.expo(-gamepad1.right_stick_x, 2));

        ShotUtil.ShotResult r = ShotUtil.compute(x, y, vx, vy, TurretController.getRpm(), movingComp);
        TurretController.setRpm(flywheelOn ? r.rpmTarget : 0);
        if (r.valid) TurretController.setHood(r.hoodDeg);
        TurretController.setYawFieldCentric(
                r.yawFieldDeg,
                heading,
                (movingComp && r.valid) ? r.vPerp : 0,
                r.ballSpeed,
                r.thetaDeg,
                TurretConstants.W_T);
        ShotUtil.Ready ready = ShotUtil.readyToShoot(
                r,
                TurretController.getYaw(),
                TurretController.getTargetYaw(),
                r.hoodDeg,
                TurretController.getRpm(),
                movingComp,
                vx,
                vy);
        TurretController.update();

        double relX = x - TurretConstants.HIVE_X;
        double relY = y - TurretConstants.HIVE_Y;
        double beta = Math.toDegrees(Math.atan2(Math.abs(relX), Math.abs(relY))); // deg

        TimedTelemetry.addData("x", "%.2f m", 1, x);
        TimedTelemetry.addData("y", "%.2f m", 1, y);
        TimedTelemetry.addData("heading", "%.1f deg", 1, heading);
        TimedTelemetry.addData("vx", "%.2f m/s", 1, vx);
        TimedTelemetry.addData("vy", "%.2f m/s", 1, vy);
        TimedTelemetry.addData("range", "%.2f m", 1, r.range);
        TimedTelemetry.addData("beta", "%.1f deg", 1, beta);
        TimedTelemetry.addData("mode", r.behind ? "BEHIND" : "FRONT");
        TimedTelemetry.addData("arc", r.steepArc ? "STEEP" : "SHALLOW");
        TimedTelemetry.addData("theta", "%.1f deg", 1, r.thetaDeg);
        TimedTelemetry.addData("phi", "%.1f deg", 1, r.phiDeg);
        TimedTelemetry.addData("hood", "%.1f deg", 1, r.hoodDeg);
        TimedTelemetry.addData("rpm target", "%.0f", 1, r.rpmTarget);
        TimedTelemetry.addData("rpm", "%.0f", 1, TurretController.getRpm());
        TimedTelemetry.addData("target yaw", "%.1f deg", 1, TurretController.getTargetYaw());
        TimedTelemetry.addData("actual yaw", "%.1f deg", 1, TurretController.getYaw());
        TimedTelemetry.addData("valid", r.valid + " " + r.reason);
        TimedTelemetry.addData("ready", ready.ready + " " + ready.reason);
        TimedTelemetry.addData("flywheelOn", flywheelOn);
        TimedTelemetry.addData("movingComp", movingComp);
        TimedTelemetry.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        MecanumDriveTrainController.stop();
        TurretController.stop();
    }

    private void zeroPose() {
        odometry.setPose(new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, START_HEADING_DEG));
    }
}

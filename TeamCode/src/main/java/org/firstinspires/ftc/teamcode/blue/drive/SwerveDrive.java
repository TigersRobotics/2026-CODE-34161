package org.firstinspires.ftc.teamcode.blue.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.blue.util.BlueConstants;

public class SwerveDrive {
    public final SwerveModule fl, fr, bl, br;
    public final SwerveModule[] modules;

    private final IMU imu;
    private final double[][] positions;
    private Odometry odometry;
    private double headingOffset = 0;
    private boolean cacheHeading = false;
    private double cachedHeading = 0;

    public SwerveDrive(HardwareMap hardwareMap) {
        fl = new SwerveModule(hardwareMap, "fl", BlueConstants.FL_OFFSET, BlueConstants.FL_REVERSED);
        fr = new SwerveModule(hardwareMap, "fr", BlueConstants.FR_OFFSET, BlueConstants.FR_REVERSED);
        bl = new SwerveModule(hardwareMap, "bl", BlueConstants.BL_OFFSET, BlueConstants.BL_REVERSED);
        br = new SwerveModule(hardwareMap, "br", BlueConstants.BR_OFFSET, BlueConstants.BR_REVERSED);
        modules = new SwerveModule[]{fl, fr, bl, br};

        double x = BlueConstants.TRACK_LENGTH / 2;
        double y = BlueConstants.TRACK_WIDTH / 2;
        positions = new double[][]{{x, y}, {x, -y}, {-x, y}, {-x, -y}};

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(BlueConstants.HUB_LOGO, BlueConstants.HUB_USB)));
    }

    public void useOdometry(Odometry odometry) {
        this.odometry = odometry;
    }

    public void cacheHeading(boolean on) {
        cacheHeading = on;
        if (on) updateHeading();
    }

    public void updateHeading() {
        cachedHeading = readHeading();
    }

    public double getHeading() {
        return cacheHeading ? cachedHeading : readHeading();
    }

    private double readHeading() {
        if (odometry != null) return AngleUnit.normalizeRadians(odometry.getHeading() - headingOffset);
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(getHeading());
    }

    public void resetHeading() {
        imu.resetYaw();
        if (odometry != null) headingOffset = odometry.getHeading();
        cachedHeading = 0;
    }

    public void drive(double forward, double left, double turn, boolean fieldCentric) {
        if (fieldCentric) {
            double h = -getHeading();
            double f = forward * Math.cos(h) - left * Math.sin(h);
            double l = forward * Math.sin(h) + left * Math.cos(h);
            forward = f;
            left = l;
        }

        move(forward, left, turn);
    }

    public void move(double forward, double left, double turn) {
        if (Math.abs(forward) < 0.001 && Math.abs(left) < 0.001 && Math.abs(turn) < 0.001) {
            for (SwerveModule m : modules) m.hold();
            return;
        }

        double r = Math.hypot(positions[0][0], positions[0][1]);
        double[] speeds = new double[4];
        double[] angles = new double[4];
        double max = 1;

        for (int i = 0; i < 4; i++) {
            double vx = forward - turn * positions[i][1] / r;
            double vy = left + turn * positions[i][0] / r;
            speeds[i] = Math.hypot(vx, vy);
            angles[i] = Math.atan2(vy, vx);
            max = Math.max(max, speeds[i]);
        }

        for (int i = 0; i < 4; i++) {
            modules[i].set(speeds[i] / max, angles[i]);
        }
    }

    public void lock() {
        fl.set(0, Math.PI / 4);
        fr.set(0, -Math.PI / 4);
        bl.set(0, -Math.PI / 4);
        br.set(0, Math.PI / 4);
    }

    public void stop() {
        for (SwerveModule m : modules) m.stop();
    }
}

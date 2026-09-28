package org.firstinspires.ftc.teamcode.blue.drive;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.blue.util.BlueConstants;

public class Odometry {
    private final GoBildaPinpointDriver pinpoint;
    private Pose pose = new Pose(0, 0, 0);

    public Odometry(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(BlueConstants.PINPOINT_X_OFFSET, BlueConstants.PINPOINT_Y_OFFSET, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                BlueConstants.PINPOINT_X_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                BlueConstants.PINPOINT_Y_REVERSED ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    public void update() {
        pinpoint.update();
        pose = new Pose(pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(AngleUnit.RADIANS));
    }

    public Pose getPose() {
        return pose;
    }

    public double getHeading() {
        return pose.heading;
    }

    public void setPose(Pose p) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, p.x, p.y, AngleUnit.RADIANS, p.heading));
        pose = p;
    }

    public double getSpeed() {
        return Math.hypot(pinpoint.getVelX(DistanceUnit.INCH), pinpoint.getVelY(DistanceUnit.INCH));
    }

    public int getEncoderX() {
        return pinpoint.getEncoderX();
    }

    public int getEncoderY() {
        return pinpoint.getEncoderY();
    }

    public String getStatus() {
        return pinpoint.getDeviceStatus().toString();
    }
}

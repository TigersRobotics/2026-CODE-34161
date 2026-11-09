package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.Competition;
import org.firstinspires.ftc.teamcode.constants.RobotSpecs;

public class OdometryV2 {

    private final GoBildaPinpointDriver odo;

    /**
     * Initializes the goBilda Pinpoint Odometry Computer.
     *
     * @param hardwareMap HardwareMap reference from the OpMode
     * @param deviceName  Device name configured in the Driver Station (e.g., "odo" or "pinpoint")
     */
    public OdometryV2(HardwareMap hardwareMap, String deviceName) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, deviceName);

        /*
         * Set physical pod offsets from the robot's center of rotation.
         *  - X Offset: Distance along X-axis (forward/backward pod offset)
         *  - Y Offset: Distance along Y-axis (perpendicular/strafe pod offset)
         */
        odo.setOffsets(RobotSpecs.OFFSETS_OF_ODOMETRY_PODS_CM_X, RobotSpecs.OFFSETS_OF_ODOMETRY_PODS_CM_Y, DistanceUnit.CM);

        /*
         * Define pod resolution type:
         * Options include:
         *   GoBildaPinpointDriver.GoBildaOdometryPods.GOBILDA_SWINGARM_POD
         *   GoBildaPinpointDriver.GoBildaOdometryPods.GOBILDA_4_BAR_POD
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        /*
         * Set encoder directions:
         * Options: EncoderDirection.FORWARD or EncoderDirection.REVERSED
         */
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Reset positions and recalibrate internal gyro orientation
        odo.resetPosAndIMU();
    }
    public void update() {
        odo.update();
    }

    public Pose2D getPose() {
        return odo.getPosition();
    }
    public double getX(DistanceUnit unit) {
        return odo.getPosition().getX(unit);
    }
    public double getY(DistanceUnit unit) {
        return odo.getPosition().getY(unit);
    }
    public double getHeading(AngleUnit unit) {
        return odo.getPosition().getHeading(unit);
    }
    public void reset() {
        odo.resetPosAndIMU();
    }

    public void setPose(Pose2D newPose) {
        odo.setPosition(newPose);
    }

    public GoBildaPinpointDriver.DeviceStatus getStatus() {
        return odo.getDeviceStatus();
    }
}
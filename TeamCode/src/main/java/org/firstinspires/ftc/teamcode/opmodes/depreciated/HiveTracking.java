package org.firstinspires.ftc.teamcode.opmodes.depreciated;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.HardwareNames;
import org.firstinspires.ftc.teamcode.field.Field;
import org.firstinspires.ftc.teamcode.robotControllers.MecanumDriveTrainController;
import org.firstinspires.ftc.teamcode.robotControllers.PollenTurretController;
import org.firstinspires.ftc.teamcode.sensors.FieldOdometry;
import org.firstinspires.ftc.teamcode.sensors.GoBildaOdometry;
import org.firstinspires.ftc.teamcode.utils.UtilMethods;

@Disabled
@Deprecated
@TeleOp(name = "Hive Tracking Test")
public class HiveTracking extends OpMode {
    FieldOdometry odo;
    boolean tipped;
    double trim; // deg

    @Override
    public void init() {
        MecanumDriveTrainController.init(hardwareMap);
        PollenTurretController.init(hardwareMap);
        odo = new FieldOdometry(new GoBildaOdometry(hardwareMap, HardwareNames.ODOMETRY_NAME));
        odo.zero(-72 + 6.5, -72 + 9, 90);
    }

    @Override
    public void loop() {
        odo.update();
        MecanumDriveTrainController.update(
                UtilMethods.expo(gamepad1.left_stick_y, 2),
                UtilMethods.expo(-gamepad1.left_stick_x, 2),
                UtilMethods.expo(-gamepad1.right_stick_x, 2));

        if (gamepad2.squareWasPressed()) tipped = !tipped;
        if (gamepad2.dpadRightWasPressed()) trim += 1;
        if (gamepad2.dpadLeftWasPressed()) trim -= 1;

        double[] t = Field.upCell(true, tipped);
        double fieldDeg = Math.toDegrees(Math.atan2(t[1] - odo.getY(), t[0] - odo.getX())); // deg
        double turret = wrap(Math.toDegrees(odo.getHeading()) - fieldDeg + trim); // deg
        if (odo.isZeroed()) PollenTurretController.setYaw(turret);
        PollenTurretController.update();

        telemetry.addData("x, y", "%.1f, %.1f in", odo.getX(), odo.getY());
        telemetry.addData("heading", "%.1f deg", Math.toDegrees(odo.getHeading()));
        telemetry.addData("turret cmd / actual", "%.1f / %.1f deg", turret, PollenTurretController.getCurrentYaw());
        telemetry.addData("tipped / trim", "%b / %.0f", tipped, trim);
        telemetry.update();
    }

    @Override
    public void stop() {
        MecanumDriveTrainController.stop();
        PollenTurretController.stop();
    }

    static double wrap(double deg) {
        deg %= 360;
        if (deg > 180) deg -= 360;
        if (deg < -180) deg += 360;
        return deg;
    }
}
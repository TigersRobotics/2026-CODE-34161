package org.firstinspires.ftc.teamcode.blue.tests;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.blue.Alliance;
import org.firstinspires.ftc.teamcode.blue.util.BlueConstants;
import org.firstinspires.ftc.teamcode.blue.util.DataLogger;
import org.firstinspires.ftc.teamcode.blue.vision.Limelight;
import org.firstinspires.ftc.teamcode.blue.vision.Target;

import java.util.List;

@TeleOp(name = "Limelight Test", group = "Blue Test")
public class LimelightTest extends OpMode {
    Limelight limelight;
    IMU imu;
    DataLogger tagLog;
    DataLogger targetLog;
    ElapsedTime timer = new ElapsedTime();

    Alliance alliance = Alliance.BLUE;
    int pipeline = Limelight.TAG_PIPELINE;

    @Override
    public void init() {
        limelight = new Limelight(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(BlueConstants.HUB_LOGO, BlueConstants.HUB_USB)));

        tagLog = new DataLogger("limelight_tags", "time", "pipeline", "id", "tx", "ty", "area", "distance", "botX", "botY", "botYaw");
        targetLog = new DataLogger("limelight_targets", "time", "type", "tx", "ty", "area", "confidence");
    }

    @Override
    public void start() {
        timer.reset();
        imu.resetYaw();
    }

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) alliance = Alliance.BLUE;
        if (gamepad1.bWasPressed()) alliance = Alliance.RED;
        if (gamepad1.dpadUpWasPressed()) pipeline = Math.min(9, pipeline + 1);
        if (gamepad1.dpadDownWasPressed()) pipeline = Math.max(0, pipeline - 1);

        limelight.setPipeline(pipeline);
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        limelight.update(heading);

        String t = String.format("%.3f", timer.seconds());
        Pose3D bot = limelight.getBotpose();
        String botX = bot == null ? "" : String.format("%.2f", bot.getPosition().x);
        String botY = bot == null ? "" : String.format("%.2f", bot.getPosition().y);
        String botYaw = bot == null ? "" : String.format("%.1f", bot.getOrientation().getYaw(AngleUnit.DEGREES));

        List<LLResultTypes.FiducialResult> tags = limelight.getTags();
        for (LLResultTypes.FiducialResult tag : tags) {
            double distance = limelight.getDistance(tag);
            tagLog.log(t, pipeline, tag.getFiducialId(),
                    String.format("%.2f", tag.getTargetXDegrees()),
                    String.format("%.2f", tag.getTargetYDegrees()),
                    String.format("%.3f", tag.getTargetArea()),
                    String.format("%.1f", distance), botX, botY, botYaw);
            telemetry.addData("Tag " + tag.getFiducialId(), "tx %.1f ty %.1f dist %.1f%s",
                    tag.getTargetXDegrees(), tag.getTargetYDegrees(), distance,
                    alliance.ownsTag(tag.getFiducialId()) ? "  (ours)" : "");
        }

        List<Target> targets = limelight.getTargets();
        for (Target target : targets) {
            targetLog.log(t, target.type,
                    String.format("%.2f", target.tx),
                    String.format("%.2f", target.ty),
                    String.format("%.3f", target.area),
                    String.format("%.2f", target.confidence));
            telemetry.addLine(target.toString());
        }

        LLResultTypes.FiducialResult cell = limelight.getCellTag(alliance);

        telemetry.addLine("X = blue, B = red, dpad U/D = pipeline");
        telemetry.addData("Connected", limelight.isConnected());
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Pipeline", pipeline);
        telemetry.addData("Valid", limelight.hasResult());
        telemetry.addData("Heading", "%.1f", heading);
        telemetry.addData("Cell tag", cell == null ? "none" : cell.getFiducialId());
        telemetry.addData("Botpose", bot == null ? "none" : botX + ", " + botY + "  yaw " + botYaw);
        telemetry.addData("Tags seen", tags.size());
        telemetry.addData("Targets seen", targets.size());
        telemetry.addData("Logs", tagLog.getRows() + " tag rows, " + targetLog.getRows() + " target rows");
    }

    @Override
    public void stop() {
        limelight.stop();
        tagLog.close();
        targetLog.close();
    }
}

package org.firstinspires.ftc.teamcode.blue.opmodes;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.blue.Alliance;
import org.firstinspires.ftc.teamcode.blue.drive.SwerveDrive;
import org.firstinspires.ftc.teamcode.blue.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.blue.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.blue.util.BlueConstants;
import org.firstinspires.ftc.teamcode.blue.util.PoseStorage;
import org.firstinspires.ftc.teamcode.blue.vision.Limelight;
import org.firstinspires.ftc.teamcode.blue.vision.Target;

@Autonomous(name = "Blue Auto Timed", group = "Blue", preselectTeleOp = "Blue TeleOp")
public class TimedAuto extends LinearOpMode {
    SwerveDrive drive;
    Limelight limelight;
    Intake intake;
    Launcher launcher;

    Alliance alliance = Alliance.BLUE;
    double delay = 0;
    boolean grabMore = true;

    @Override
    public void runOpMode() {
        drive = new SwerveDrive(hardwareMap);
        limelight = new Limelight(hardwareMap);
        intake = new Intake(hardwareMap);
        launcher = new Launcher(hardwareMap);

        while (opModeInInit()) {
            if (gamepad1.xWasPressed()) alliance = Alliance.BLUE;
            if (gamepad1.bWasPressed()) alliance = Alliance.RED;
            if (gamepad1.dpadUpWasPressed()) delay += 0.5;
            if (gamepad1.dpadDownWasPressed()) delay = Math.max(0, delay - 0.5);
            if (gamepad1.yWasPressed()) grabMore = !grabMore;

            limelight.update(drive.getHeadingDegrees());
            LLResultTypes.FiducialResult tag = limelight.getCellTag(alliance);

            telemetry.addLine("X = blue, B = red, dpad = delay, Y = grab more");
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Delay", delay);
            telemetry.addData("Grab more", grabMore);
            telemetry.addData("Sees cell", tag != null);
            telemetry.update();
        }

        if (isStopRequested()) return;

        drive.resetHeading();
        PoseStorage.alliance = alliance;
        sleep((long) (delay * 1000));

        launcher.setRpm(3000);
        driveFor(0.5, 0, 0, 0.6);
        aim(1.5);
        shoot(3.0);

        if (grabMore) {
            chasePollen(2.5);
            driveFor(-0.5, 0, 0, 0.8);
            aim(1.5);
            shoot(2.5);
        }

        launcher.stop();
        intake.stop();
        driveFor(-0.5, 0, 0, 1.0);
        drive.stop();
        limelight.stop();
    }

    void driveFor(double forward, double left, double turn, double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            drive.drive(forward, left, turn, false);
            idle();
        }
        drive.drive(0, 0, 0, false);
    }

    void aim(double seconds) {
        limelight.setPipeline(Limelight.TAG_PIPELINE);
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < seconds) {
            limelight.update(drive.getHeadingDegrees());
            LLResultTypes.FiducialResult tag = limelight.getCellTag(alliance);

            if (tag == null) {
                drive.drive(0, 0, 0, false);
                continue;
            }

            double tx = tag.getTargetXDegrees();
            if (Math.abs(tx) < BlueConstants.AIM_TOLERANCE) {
                launcher.aimFor(limelight.getDistance(tag));
                break;
            }
            drive.drive(0, 0, -BlueConstants.AIM_P * tx, false);

            telemetry.addData("tx", tx);
            telemetry.update();
        }
        drive.drive(0, 0, 0, false);
    }

    void shoot(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            drive.drive(0, 0, 0, false);
            if (launcher.isReady()) {
                launcher.feed();
                intake.in();
            } else {
                launcher.stopFeed();
                intake.stop();
            }
            telemetry.addData("rpm", launcher.getRpm());
            telemetry.update();
        }
        launcher.stopFeed();
        intake.stop();
    }

    void chasePollen(double seconds) {
        limelight.setPipeline(Limelight.DETECTOR_PIPELINE);
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < seconds) {
            limelight.update(drive.getHeadingDegrees());
            Target pollen = limelight.getClosest(Target.Type.POLLEN);
            intake.in();

            if (pollen == null) {
                drive.drive(0, 0, 0.3, false);
            } else {
                drive.drive(0.4, 0, -BlueConstants.PICKUP_P * pollen.tx, false);
            }
        }
        drive.drive(0, 0, 0, false);
        limelight.setPipeline(Limelight.TAG_PIPELINE);
    }
}

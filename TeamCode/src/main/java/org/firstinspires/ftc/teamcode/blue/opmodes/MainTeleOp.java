package org.firstinspires.ftc.teamcode.blue.opmodes;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.blue.Alliance;
import org.firstinspires.ftc.teamcode.blue.controls.Controls;
import org.firstinspires.ftc.teamcode.blue.drive.SwerveDrive;
import org.firstinspires.ftc.teamcode.blue.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.blue.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.blue.util.BlueConstants;
import org.firstinspires.ftc.teamcode.blue.vision.Limelight;
import org.firstinspires.ftc.teamcode.blue.vision.Target;

@TeleOp(name = "Blue TeleOp", group = "Blue")
public class MainTeleOp extends OpMode {
    SwerveDrive drive;
    Limelight limelight;
    Intake intake;
    Launcher launcher;
    Controls controls;

    Alliance alliance = Alliance.BLUE;
    boolean fieldCentric = true;
    boolean launcherOn = false;
    boolean autoDistance = true;
    double manualRpm = 3000;
    double manualHood = 0.5;

    @Override
    public void init() {
        drive = new SwerveDrive(hardwareMap);
        limelight = new Limelight(hardwareMap);
        intake = new Intake(hardwareMap);
        launcher = new Launcher(hardwareMap);
        controls = new Controls(gamepad1, gamepad2);
    }

    @Override
    public void init_loop() {
        if (gamepad1.xWasPressed()) alliance = Alliance.BLUE;
        if (gamepad1.bWasPressed()) alliance = Alliance.RED;

        telemetry.addLine("X = blue, B = red");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Limelight", limelight.isConnected() ? "connected" : "NOT CONNECTED");
    }

    @Override
    public void start() {
        limelight.setPipeline(Limelight.TAG_PIPELINE);
    }

    @Override
    public void loop() {
        limelight.update(drive.getHeadingDegrees());

        if (controls.resetHeading()) drive.resetHeading();
        if (controls.toggleFieldCentric()) fieldCentric = !fieldCentric;

        double speed = controls.slow() ? BlueConstants.SLOW_SPEED : 1.0;
        double forward = controls.forward() * speed;
        double strafe = controls.strafe() * speed;
        double turn = controls.turn() * speed;

        LLResultTypes.FiducialResult tag = limelight.getCellTag(alliance);
        double distance = tag != null ? limelight.getDistance(tag) : -1;
        boolean aimed = false;
        boolean chasing = false;

        if (controls.autoAim()) {
            limelight.setPipeline(Limelight.TAG_PIPELINE);
            if (tag != null) {
                turn = -BlueConstants.AIM_P * tag.getTargetXDegrees();
                aimed = Math.abs(tag.getTargetXDegrees()) < BlueConstants.AIM_TOLERANCE;
            }
        } else if (controls.chasePollen()) {
            limelight.setPipeline(Limelight.DETECTOR_PIPELINE);
            Target pollen = limelight.getClosest(Target.Type.POLLEN);
            if (pollen != null) {
                turn = -BlueConstants.PICKUP_P * pollen.tx;
                chasing = true;
            }
        } else {
            limelight.setPipeline(Limelight.TAG_PIPELINE);
        }

        if (controls.lockWheels()) {
            drive.lock();
        } else {
            drive.drive(forward, strafe, turn, fieldCentric);
        }

        if (controls.toggleLauncher()) launcherOn = !launcherOn;
        if (controls.toggleAutoDistance()) autoDistance = !autoDistance;

        if (controls.rpmUp()) manualRpm += 100;
        if (controls.rpmDown()) manualRpm -= 100;
        if (controls.hoodUp()) {
            manualHood += 0.05;
            autoDistance = false;
        }
        if (controls.hoodDown()) {
            manualHood -= 0.05;
            autoDistance = false;
        }

        if (!launcherOn) {
            launcher.stop();
        } else if (autoDistance && distance > 0) {
            launcher.aimFor(distance);
        } else {
            launcher.setRpm(manualRpm);
            launcher.setHood(manualHood);
        }

        boolean shooting = controls.shoot() && launcher.isReady();

        if (shooting) {
            launcher.feed();
        } else if (controls.unjam()) {
            launcher.unjam();
        } else {
            launcher.stopFeed();
        }

        if (controls.intakeOut()) {
            intake.out();
        } else if (controls.intakeIn() || chasing || shooting) {
            intake.in();
        } else {
            intake.stop();
        }

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Field centric", fieldCentric);
        telemetry.addData("Heading", "%.1f", drive.getHeadingDegrees());
        telemetry.addData("Tag", tag == null ? "none" : tag.getFiducialId() + "  tx " + String.format("%.1f", tag.getTargetXDegrees()));
        telemetry.addData("Distance", "%.1f", distance);
        telemetry.addData("Aimed", aimed);
        telemetry.addData("Launcher", "%s  %.0f / %.0f rpm  hood %.2f", launcherOn ? "ON" : "off", launcher.getRpm(), launcher.getTargetRpm(), launcher.getHood());
        telemetry.addData("Auto distance", autoDistance);
        telemetry.addData("Ready", launcher.isReady());
    }

    @Override
    public void stop() {
        drive.stop();
        intake.stop();
        launcher.stop();
        limelight.stop();
    }
}

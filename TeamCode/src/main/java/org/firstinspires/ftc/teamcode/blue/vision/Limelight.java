package org.firstinspires.ftc.teamcode.blue.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.blue.Alliance;
import org.firstinspires.ftc.teamcode.blue.util.BlueConstants;

import java.util.ArrayList;
import java.util.List;

public class Limelight {
    public static final int TAG_PIPELINE = 0;
    public static final int DETECTOR_PIPELINE = 1;

    private final Limelight3A limelight;
    private LLResult result;
    private int pipeline = -1;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        setPipeline(TAG_PIPELINE);
        limelight.start();
    }

    public void update(double headingDegrees) {
        limelight.updateRobotOrientation(headingDegrees);
        result = limelight.getLatestResult();
    }

    public void setPipeline(int index) {
        if (index == pipeline) return;
        limelight.pipelineSwitch(index);
        pipeline = index;
    }

    public int getPipeline() {
        return pipeline;
    }

    public boolean hasResult() {
        return result != null && result.isValid();
    }

    public LLResult getResult() {
        return result;
    }

    public boolean isConnected() {
        return limelight.isConnected();
    }

    public List<LLResultTypes.FiducialResult> getTags() {
        if (!hasResult()) return new ArrayList<>();
        return result.getFiducialResults();
    }

    public LLResultTypes.FiducialResult getCellTag(Alliance alliance) {
        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult tag : getTags()) {
            if (!alliance.ownsTag(tag.getFiducialId())) continue;
            if (best == null || tag.getTargetYDegrees() > best.getTargetYDegrees()) best = tag;
        }
        return best;
    }

    public double getDistance(LLResultTypes.FiducialResult tag) {
        double angle = Math.toRadians(BlueConstants.CAMERA_PITCH + tag.getTargetYDegrees());
        return (BlueConstants.TAG_HEIGHT - BlueConstants.CAMERA_HEIGHT) / Math.tan(angle);
    }

    public List<Target> getTargets() {
        List<Target> targets = new ArrayList<>();
        if (!hasResult()) return targets;

        for (LLResultTypes.DetectorResult d : result.getDetectorResults()) {
            if (d.getConfidence() < BlueConstants.MIN_CONFIDENCE) continue;
            targets.add(new Target(Target.typeFromName(d.getClassName()),
                    d.getTargetXDegrees(), d.getTargetYDegrees(), d.getTargetArea(), d.getConfidence()));
        }
        return targets;
    }

    public Target getClosest(Target.Type type) {
        Target best = null;
        for (Target t : getTargets()) {
            if (type != null && t.type != type) continue;
            if (best == null || t.area > best.area) best = t;
        }
        return best;
    }

    public Pose3D getBotpose() {
        if (!hasResult()) return null;
        return result.getBotpose_MT2();
    }

    public boolean snapshot(String name) {
        return limelight.captureSnapshot(name);
    }

    public boolean clearSnapshots() {
        return limelight.deleteSnapshots();
    }

    public void stop() {
        limelight.stop();
    }
}

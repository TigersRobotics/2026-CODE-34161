package org.firstinspires.ftc.teamcode.blue.vision;

import org.firstinspires.ftc.teamcode.blue.Alliance;

public class Target {
    public enum Type {
        POLLEN,
        RED_NECTAR,
        BLUE_NECTAR,
        UNKNOWN
    }

    public final Type type;
    public final double tx;
    public final double ty;
    public final double area;
    public final double confidence;

    public Target(Type type, double tx, double ty, double area, double confidence) {
        this.type = type;
        this.tx = tx;
        this.ty = ty;
        this.area = area;
        this.confidence = confidence;
    }

    public boolean isNectar() {
        return type == Type.RED_NECTAR || type == Type.BLUE_NECTAR;
    }

    public boolean isOurNectar(Alliance alliance) {
        return alliance == Alliance.RED ? type == Type.RED_NECTAR : type == Type.BLUE_NECTAR;
    }

    public static Type typeFromName(String name) {
        String n = name.toLowerCase();
        if (n.contains("pollen") || n.contains("yellow")) return Type.POLLEN;
        if (n.contains("red")) return Type.RED_NECTAR;
        if (n.contains("blue")) return Type.BLUE_NECTAR;
        return Type.UNKNOWN;
    }

    @Override
    public String toString() {
        return String.format("%s tx=%.1f ty=%.1f a=%.2f c=%.2f", type, tx, ty, area, confidence);
    }
}

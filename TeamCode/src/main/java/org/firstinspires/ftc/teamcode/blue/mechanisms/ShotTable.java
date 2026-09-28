package org.firstinspires.ftc.teamcode.blue.mechanisms;

public class ShotTable {
    public static double[] DISTANCES = {24, 36, 48, 60, 72, 96};
    public static double[] RPMS = {2400, 2700, 3000, 3300, 3600, 4200};
    public static double[] HOODS = {0.7, 0.6, 0.5, 0.45, 0.4, 0.35};

    public static double rpm(double distance) {
        return lookup(RPMS, distance);
    }

    public static double hood(double distance) {
        return lookup(HOODS, distance);
    }

    private static double lookup(double[] values, double distance) {
        if (distance <= DISTANCES[0]) return values[0];
        int last = DISTANCES.length - 1;
        if (distance >= DISTANCES[last]) return values[last];

        for (int i = 0; i < last; i++) {
            if (distance <= DISTANCES[i + 1]) {
                double t = (distance - DISTANCES[i]) / (DISTANCES[i + 1] - DISTANCES[i]);
                return values[i] + t * (values[i + 1] - values[i]);
            }
        }
        return values[last];
    }
}

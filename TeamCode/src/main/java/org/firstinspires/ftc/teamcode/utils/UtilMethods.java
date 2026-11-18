package org.firstinspires.ftc.teamcode.utils;


public final class UtilMethods {


    /**
     * Scales a number to an exponent, but keeps the sign of the number
     * e.g., expo(2, 2) = 4, expo(-2, 2) = -4
     * usually good for the controller inputs(0-1)
     * @param input  the input to reduce/increase
     * @param exponent  The scaling rate
     */
    public static double expo(double input, double exponent) {
        // Squaring is the common case (every drive stick, every loop) and does not need Math.pow
        if(exponent == 2) return input * Math.abs(input);
        return Math.signum(input)*Math.pow(Math.abs(input), exponent);
    }

    /**
     * Clips a number between two values
     */
    public static double clip(double num, double min, double max) {
        return Math.min(Math.max(num, min), max);
    }

}

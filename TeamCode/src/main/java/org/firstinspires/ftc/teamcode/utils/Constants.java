package org.firstinspires.ftc.teamcode.utils;


public final class Constants {


    /**
     * Scales a number to an exponent, but keeps the sign of the number
     * e.g., expo(2, 2) = 4, expo(-2, 2) = -4
     * usually good for the controller inputs(0-1)
     * @param input  the input to reduce/increase
     * @param exponent  The scaling rate
     */
    public static double expo(double input, double exponent) {
        return Math.signum(input)*Math.pow(Math.abs(input), exponent);
    }

}

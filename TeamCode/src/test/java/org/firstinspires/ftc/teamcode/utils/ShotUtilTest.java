package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class ShotUtilTest {
    @Test
    public void stationaryShotAtThreeFeet() {
        double previousShooterZ = TurretConstants.SHOOTER_Z;
        TurretConstants.SHOOTER_Z = 0.3048; // m
        try {
            ShotUtil.ShotResult result = ShotUtil.compute(0, 0.9144, 0, 0, 2350, false);
            assertTrue(result.valid);
            assertTrue(result.steepArc);
            assertEquals(5.31, result.ballSpeed, 0.02); // m/s
            assertEquals(75.6, result.thetaDeg, 0.1); // deg
            assertEquals(14.4, result.hoodDeg, 0.1); // deg
        } finally {
            TurretConstants.SHOOTER_Z = previousShooterZ;
        }
    }
}

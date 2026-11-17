package org.firstinspires.ftc.teamcode.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

public class CustomController {
    private final Gamepad gamepad;

    private Binding.Axis driveX;
    private Binding.Axis driveY;
    private Binding.Axis turnRight;
    private Binding.Axis turnLeft;

    private Binding.Button toggleIntake;
    private Binding.Axis uptake;


    ///Technician Bindings
    private Binding.Axis overrideTurretLeft;
    private Binding.Axis overrideTurretRight;
    private Binding.Axis overridePark;
    private Binding.Button toggleAutoPark;


    /**
     * This is for the driver controller
     * @param gamepad
     * @param driveX
     * @param driveY
     * @param turnRight
     * @param turnLeft
     * @param toggleIntake
     * @param uptake
     */
    public CustomController(Gamepad gamepad,
                            Binding.Axis driveX,
                            Binding.Axis driveY,
                            Binding.Axis turnRight,
                            Binding.Axis turnLeft,
                            Binding.Button toggleIntake,
                            Binding.Axis uptake) {
        this.gamepad = gamepad;
        this.driveX = driveX;
        this.driveY = driveY;
        this.turnRight = turnRight;
        this.turnLeft = turnLeft;
        this.toggleIntake = toggleIntake;
        this.uptake = uptake;
    }

    public CustomController(Gamepad gamepad,
                            Binding.Axis overrideTurretLeft,
                            Binding.Axis overrideTurretRight) {
        this.gamepad = gamepad;
        this.overrideTurretRight = overrideTurretRight;
        this.overrideTurretLeft = overrideTurretLeft;
    }
    public double getDriveX() {
        return driveX.applyAsDouble(gamepad);
    }
    public double getDriveY() {
        return driveY.applyAsDouble(gamepad);
    }
    public double getTurn() {
        return - turnRight.applyAsDouble(gamepad) + turnLeft.applyAsDouble(gamepad);
    }

    public boolean getToggleIntake() {return toggleIntake.test(gamepad);}
    public double getUptake() {return uptake.applyAsDouble(gamepad);}
}

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
    private Binding.Button degUpHood;
    private Binding.Button degDownHood;
    private Binding.Button toggleFlywheel;
    private Binding.Button turnUpFlywheelSpeed;
    private Binding.Button turnDownFlywheelSpeed;
    private Binding.Button changeHiveIndex;


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
                            Binding.Axis overrideTurretRight,
                            Binding.Button toggleAutoPark,
                            Binding.Button degUpHood,
                            Binding.Button degDownHood,
                            Binding.Button toggleFlywheel,
                            Binding.Button turnUpFlywheelSpeed,
                            Binding.Button turnDownFlywheelSpeed,
                            Binding.Button changeHiveIndex) {
        this.gamepad = gamepad;
        this.overrideTurretRight = overrideTurretRight;
        this.overrideTurretLeft = overrideTurretLeft;
        this.toggleAutoPark = toggleAutoPark;
        this.degUpHood = degUpHood;
        this.degDownHood = degDownHood;
        this.toggleFlywheel = toggleFlywheel;
        this.turnUpFlywheelSpeed = turnUpFlywheelSpeed;
        this.turnDownFlywheelSpeed = turnDownFlywheelSpeed;
        this.changeHiveIndex = changeHiveIndex;

    }

    public boolean turnUpFlywheelSpeed() {
        return turnUpFlywheelSpeed.test(gamepad);
    }
    public boolean turnDownFlywheelSpeed() {
        return turnDownFlywheelSpeed.test(gamepad);
    }
    public boolean getDegUpHood() {
        return degUpHood.test(gamepad);
    }
    public boolean getDegDownHood() {
        return degDownHood.test(gamepad);
    }
    public boolean toggleFlywheel() {
        return toggleFlywheel.test(gamepad);
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

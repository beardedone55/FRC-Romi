// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (C) Brian LePage

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ManualDrive extends CommandBase {
    private final RomiDrivetrain driveBaseSubsystem;
    private final DoubleSupplier speedInput;
    private final DoubleSupplier rotationInput;

    /**
     * Creates a new ManualDrive command.
     *
     * @param driveBaseSubsystem The drive base subsystem used by this command.
     * @param speedInput Method that provides speed input from user (joystick).
     * @param rotationInput Method that provides rotation input from user (joystick).
     */
    public ManualDrive(RomiDrivetrain driveBaseSubsystem, DoubleSupplier speedInput, DoubleSupplier rotationInput) {
        this.driveBaseSubsystem = driveBaseSubsystem;
        this.speedInput = speedInput;
        this.rotationInput = rotationInput;
        addRequirements(driveBaseSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        driveBaseSubsystem.arcadeDrive(speedInput.getAsDouble(), rotationInput.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}

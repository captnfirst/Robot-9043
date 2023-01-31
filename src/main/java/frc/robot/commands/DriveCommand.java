// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */
  DriveSubsystem mDriveSub;
  DoubleSupplier mspeed;
  DoubleSupplier mrotation;
  public DriveCommand(DriveSubsystem drive, DoubleSupplier speed, DoubleSupplier rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    mspeed= speed;
    mrotation = rotation;
    mDriveSub = drive;
    addRequirements(mDriveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDriveSub.arcadeDrive(mspeed.getAsDouble(), mrotation.getAsDouble());
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

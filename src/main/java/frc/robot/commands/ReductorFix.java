// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ReductorFix extends CommandBase {
  private final Drivetrain drive;

  public ReductorFix(Drivetrain robotDrive) {
    drive = robotDrive;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    drive.arcadeDrive(-0.5, 0);
  }

  @Override
  public void execute() {
    drive.arcadeDrive(-0.5, 0);
  }

  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

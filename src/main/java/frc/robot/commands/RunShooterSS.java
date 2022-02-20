// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSS;

public class RunShooterSS extends CommandBase {

  private final ShooterSS shooterState;

  NetworkTableEntry bottomShooterSpeed;

  /** Creates a new RunShooterState. */
  public RunShooterSS(ShooterSS shooterState, NetworkTableEntry bottomShooterSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterState = shooterState;
    addRequirements(shooterState);
    this.bottomShooterSpeed = bottomShooterSpeed;
  }

  @Override
  public void execute() {
    shooterState.run(this.bottomShooterSpeed.getDouble(0));
  }

  @Override
  public void end(boolean interrupted) {
    shooterState.run(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

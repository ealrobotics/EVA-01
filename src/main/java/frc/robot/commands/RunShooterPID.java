// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterPID;

public class RunShooterPID extends CommandBase {

  private final ShooterPID shooter;

  NetworkTableEntry bottomShooterSpeed;

  /** Creates a new RunShooter. */
  public RunShooterPID(ShooterPID shooter, NetworkTableEntry bottomShooterSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    addRequirements(shooter);
    this.bottomShooterSpeed = bottomShooterSpeed;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.run(this.bottomShooterSpeed.getDouble(0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

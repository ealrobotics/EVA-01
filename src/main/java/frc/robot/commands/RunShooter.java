// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class RunShooter extends CommandBase {

  private final Shooter m_shooter;

  NetworkTableEntry m_bottomShooterPower;
  NetworkTableEntry m_topShooterPower;

  public RunShooter(Shooter shooter, NetworkTableEntry bottomShooterPower, NetworkTableEntry topShooterPower) {
    m_shooter = shooter;
    m_bottomShooterPower = bottomShooterPower;
    m_topShooterPower = topShooterPower;

    addRequirements(shooter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.run(m_bottomShooterPower.getDouble(0.0), m_topShooterPower.getDouble(0.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.run(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

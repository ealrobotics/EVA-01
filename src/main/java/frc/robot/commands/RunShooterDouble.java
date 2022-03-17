// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class RunShooterDouble extends CommandBase {

  private final Shooter m_shooter;

  Double m_bottomShooterPower;
  Double m_topShooterPower;

  public RunShooterDouble(Shooter shooter, Double bottomShooterPower, Double topShooterPower) {
    m_shooter = shooter;
    m_bottomShooterPower = bottomShooterPower;
    m_topShooterPower = topShooterPower;

    addRequirements(shooter);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.run(m_bottomShooterPower, m_topShooterPower);
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

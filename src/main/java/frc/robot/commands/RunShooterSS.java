// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSS;

public class RunShooterSS extends CommandBase {

  private final ShooterSS m_shooterSS;

  NetworkTableEntry m_bottomShooterSpeed;

  public RunShooterSS(ShooterSS shooterState, NetworkTableEntry bottomShooterSpeed) {
    m_shooterSS = shooterState;
    m_bottomShooterSpeed = bottomShooterSpeed;

    addRequirements(shooterState);
  }

  @Override
  public void execute() {
    m_shooterSS.run(m_bottomShooterSpeed.getDouble(0.0));
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSS.run(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

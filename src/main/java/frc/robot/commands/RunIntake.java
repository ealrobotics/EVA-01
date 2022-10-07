// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends CommandBase {
  private final IntakeSubsystem m_intake;
  private final Double m_power;

  public RunIntake(IntakeSubsystem intake, Double power) {
    m_intake = intake;
    m_power = power;

    addRequirements(intake);
  }

  @Override
  public void execute() {
    m_intake.run(m_power);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.run(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

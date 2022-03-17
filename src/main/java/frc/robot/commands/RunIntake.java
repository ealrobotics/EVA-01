// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
  private final Intake m_intake;
  private final NetworkTableEntry m_power;

  public RunIntake(Intake intake, NetworkTableEntry power) {
    m_intake = intake;
    m_power = power;

    addRequirements(intake);
  }

  @Override
  public void execute() {
    m_intake.run(m_power.getDouble(0));
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

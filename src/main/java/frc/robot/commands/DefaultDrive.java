// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
//import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DefaultDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final DoubleSupplier m_forwardPower, m_turnPower;
  private final BooleanSupplier m_boost;

  public DefaultDrive(Drivetrain drivetrain, DoubleSupplier forwardPower, DoubleSupplier turnPower,
      BooleanSupplier boost) {
    m_drivetrain = drivetrain;
    m_forwardPower = forwardPower;
    m_turnPower = turnPower;
    m_boost = boost;

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    m_drivetrain.setMaxOutput(m_boost.getAsBoolean() ? 0.5 : 1);
    m_drivetrain.arcadeDrive(m_forwardPower.getAsDouble(), m_turnPower.getAsDouble());
  }
}

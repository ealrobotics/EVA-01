package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Drivetrain;

public class AutoDrive extends CommandBase {
  private final Drivetrain m_drivetrain;

  public AutoDrive(Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(0.3, 0.0);
  }
}
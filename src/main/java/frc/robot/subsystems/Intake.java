// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  // Intake motor
  private WPI_VictorSPX m_motor = new WPI_VictorSPX(CANIDConstants.intakeMotorID);

  public Intake() {
    m_motor.setInverted(IntakeConstants.motorInverted);
  }

  public void run(double pwr) {
    m_motor.set(ControlMode.PercentOutput, pwr);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

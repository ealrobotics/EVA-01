// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIDConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final CANSparkMax bottomMotor = new CANSparkMax(CANIDConstants.shooterBottomMotorID, MotorType.kBrushless);
  BangBangController controller = new BangBangController();
  Encoder encoder = new Encoder(Constants.ShooterConstants.kBottomEncoderPorts[0],
      Constants.ShooterConstants.kBottomEncoderPorts[1]);

  /** Creates a new Shooter. */
  public ShooterSubsystem() {
    bottomMotor.restoreFactoryDefaults();
    bottomMotor.setInverted(false);
    bottomMotor.setIdleMode(IdleMode.kCoast);

    encoder.setDistancePerPulse(Constants.ShooterConstants.kBottomEncoderDistancePerPulse);
  }

  public void run(Double setpoint) {
    bottomMotor.set(controller.calculate(encoder.getRate(), setpoint));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

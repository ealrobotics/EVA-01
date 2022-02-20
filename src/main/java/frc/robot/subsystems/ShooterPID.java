// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPID extends SubsystemBase {

  private CANSparkMax bottomMotor;
  // private CANSparkMax topMotor;
  private SparkMaxPIDController bottomPidContoller;
  // private SparkMaxPIDController topPidContoller;
  private RelativeEncoder bottomEncoder;
  // private RelativeEncoder topEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  /** Creates a new Shooter. */
  public ShooterPID() {
    bottomMotor = new CANSparkMax(CANIDConstants.shooterBottomMotorID, MotorType.kBrushless);
    bottomMotor.restoreFactoryDefaults();
    // topMotor = new CANSparkMax(CANIDConstants.shooterTopMotorID,
    // MotorType.kBrushed);
    // topMotor.restoreFactoryDefaults();

    bottomEncoder = bottomMotor.getAlternateEncoder(ShooterConstants.kEncType, ShooterConstants.kCPR);
    // topEncoder = topMotor.getAlternateEncoder(ShooterConstants.kEncType,
    // ShooterConstants.kCPR);

    bottomPidContoller = bottomMotor.getPIDController();
    // topPidContoller = topMotor.getPIDController();

    bottomPidContoller.setFeedbackDevice(bottomEncoder);
    // topPidContoller.setFeedbackDevice(topEncoder);

    // PID coefficients
    kP = 6e-5;
    kI = 0;
    kD = 0;
    kIz = 0;
    kFF = 0.000015;
    kMaxOutput = 1;
    kMinOutput = -1;
    maxRPM = 5700;

    // set PID coefficients
    bottomPidContoller.setP(kP);
    bottomPidContoller.setI(kI);
    bottomPidContoller.setD(kD);
    bottomPidContoller.setIZone(kIz);
    bottomPidContoller.setFF(kFF);
    bottomPidContoller.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

  }

  public void run(double bottomShooterSpeed) {
    double setPoint = bottomShooterSpeed;
    bottomPidContoller.setReference(setPoint, CANSparkMax.ControlType.kVelocity);

    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("ProcessVariable", bottomEncoder.getVelocity());
  }

  public void stop() {
    bottomMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller
    if ((p != kP)) {
      bottomPidContoller.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      bottomPidContoller.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      bottomPidContoller.setD(d);
      kD = d;
    }
    if ((iz != kIz)) {
      bottomPidContoller.setIZone(iz);
      kIz = iz;
    }
    if ((ff != kFF)) {
      bottomPidContoller.setFF(ff);
      kFF = ff;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      bottomPidContoller.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  // Drivebase motors' variables
  private CANSparkMax leftLeadMotor = new CANSparkMax(CANIDConstants.drivebaseLeftLeadMotorID, MotorType.kBrushed);
  private CANSparkMax leftFollowMotor = new CANSparkMax(CANIDConstants.drivebaseLeftFollowMotorID, MotorType.kBrushed);
  private CANSparkMax rightLeadMotor = new CANSparkMax(CANIDConstants.drivebaseRightLeadMotorID, MotorType.kBrushed);
  private CANSparkMax rightFollowMotor = new CANSparkMax(CANIDConstants.drivebaseRightFollowMotorID,
      MotorType.kBrushed);

  // The robot's drive's variable
  private final DifferentialDrive drive = new DifferentialDrive(leftLeadMotor, rightLeadMotor);

  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem() {
    // Reset motors
    leftLeadMotor.restoreFactoryDefaults();
    leftFollowMotor.restoreFactoryDefaults();
    rightLeadMotor.restoreFactoryDefaults();
    rightFollowMotor.restoreFactoryDefaults();

    rightLeadMotor.setInverted(true);
    rightFollowMotor.setInverted(true);

    // Make the motors on the same side follow each other
    leftFollowMotor.follow(leftLeadMotor);
    rightFollowMotor.follow(rightLeadMotor);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }
}
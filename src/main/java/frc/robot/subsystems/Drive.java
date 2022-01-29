// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drive extends SubsystemBase {
  // Drivebase motors' variables
  private CANSparkMax leftLeadMotor;
  private CANSparkMax leftFollowMotor;
  private CANSparkMax rightLeadMotor;
  private CANSparkMax rightFollowMotor;

  // The robot's drive's variable
  private final DifferentialDrive drive;

  /** Creates a new Drive. */
  public Drive() {
    // Drivebase motors
    leftLeadMotor = new CANSparkMax(CANIDConstants.drivebaseLeftLeadMotorID, MotorType.kBrushed);
    leftFollowMotor = new CANSparkMax(CANIDConstants.drivebaseLeftFollowMotorID, MotorType.kBrushed);
    rightLeadMotor = new CANSparkMax(CANIDConstants.drivebaseRightLeadMotorID, MotorType.kBrushed);
    rightFollowMotor = new CANSparkMax(CANIDConstants.drivebaseRightFollowMotorID, MotorType.kBrushed);

    // Reset motors
    leftLeadMotor.restoreFactoryDefaults();
    leftFollowMotor.restoreFactoryDefaults();
    rightLeadMotor.restoreFactoryDefaults();
    rightFollowMotor.restoreFactoryDefaults();

    // Make the motors on the same side follow each other
    leftFollowMotor.follow(leftLeadMotor);
    rightFollowMotor.follow(rightLeadMotor);

    // The robot's drive
    drive = new DifferentialDrive(leftLeadMotor, rightLeadMotor);
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

  // Set max output power to .5
  public void setMaxOutputToHalf() {
    drive.setMaxOutput(0.5);
  }

  // Set max output power to 1
  public void setMaxOutputToMax() {
    drive.setMaxOutput(1);
  }
}

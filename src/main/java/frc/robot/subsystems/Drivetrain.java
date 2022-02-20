// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDConstants;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
  // Drivebase motors' variables
  private CANSparkMax leftLeadMotor = new CANSparkMax(CANIDConstants.drivebaseLeftLeadMotorID, MotorType.kBrushed);
  private CANSparkMax leftFollowMotor = new CANSparkMax(CANIDConstants.drivebaseLeftFollowMotorID, MotorType.kBrushed);
  private CANSparkMax rightLeadMotor = new CANSparkMax(CANIDConstants.drivebaseRightLeadMotorID, MotorType.kBrushed);
  private CANSparkMax rightFollowMotor = new CANSparkMax(CANIDConstants.drivebaseRightFollowMotorID,
      MotorType.kBrushed);

  // The robot's drive's variable
  private final DifferentialDrive drive = new DifferentialDrive(leftLeadMotor, rightLeadMotor);

  // The left-side drive encoder
  private final Encoder leftEncoder = new Encoder(
      DrivetrainConstants.kLeftEncoderPorts[0],
      DrivetrainConstants.kLeftEncoderPorts[1],
      DrivetrainConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder rightEncoder = new Encoder(
      DrivetrainConstants.kRightEncoderPorts[0],
      DrivetrainConstants.kRightEncoderPorts[1],
      DrivetrainConstants.kRightEncoderReversed);

  // The gyro sensor
  private final ADIS16470_IMU imu = new ADIS16470_IMU();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public Drivetrain() {
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

    // Sets the distance per pulse for the encoders
    leftEncoder.setDistancePerPulse(DrivetrainConstants.kEncoderDistancePerPulse);
    leftEncoder.setDistancePerPulse(DrivetrainConstants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(imu.getAngle()));
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(imu.getAngle()), leftEncoder.getDistance(), rightEncoder.getDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(imu.getAngle()));
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
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftLeadMotor.setVoltage(leftVolts);
    rightLeadMotor.setVoltage(rightVolts);
    drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return rightEncoder;
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

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    imu.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return imu.getAngle();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -imu.getRate();
  }
}
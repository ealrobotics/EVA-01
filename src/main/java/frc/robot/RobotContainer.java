// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/* import java.util.List; */

import edu.wpi.first.cameraserver.CameraServer;
/* import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint; */
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
/* import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants; */
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.RunShooterDouble;
import frc.robot.commands.RunIntakeDouble;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
        // Subsystems
        private final Drivetrain m_drivetrain = new Drivetrain();
        /* private final ShooterSS m_shooterSS = new ShooterSS(); */
        private final Shooter m_shooter = new Shooter();
        private final Intake m_intake = new Intake();

        // XboxController on port
        XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);

        private final Command m_autoCommand = new InstantCommand(() -> m_shooter.run(0.56, 0.56), m_shooter)
                        .andThen(
                                        new WaitCommand(2),
                                        new InstantCommand(() -> m_intake.run(0.70), m_intake),
                                        new WaitCommand(2),
                                        new RunCommand(() -> m_drivetrain.arcadeDrive(-0.6, 0), m_drivetrain)
                                                        .withTimeout(1.2),
                                        new InstantCommand(() -> m_intake.run(0.80), m_intake),
                                        new InstantCommand(() -> m_shooter.run(0.63, 0.63), m_shooter),
                                        new RunCommand(() -> m_drivetrain.arcadeDrive(0.0, 0.0), m_drivetrain))
                        .withTimeout(8.0)
                        .andThen(() -> {
                                m_shooter.run(0.0, 0.0);
                                m_intake.run(0.0);
                                m_drivetrain.arcadeDrive(0.0, 0.0);
                        });

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Camera
                CameraServer.startAutomaticCapture();
                CameraServer.startAutomaticCapture();

                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                // Set the drive mode to arcade drive

                m_drivetrain.setDefaultCommand(
                                new DefaultDrive(m_drivetrain, () -> driverController.getLeftY(),
                                                () -> -driverController.getRightX(),
                                                () -> driverController.getRightTriggerAxis() > 0));

        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                new JoystickButton(driverController, Button.kA.value)
                                .whileHeld(new RunShooterDouble(m_shooter, 0.58, 0.58));
                new JoystickButton(driverController, Button.kB.value)
                                .whileHeld(new RunShooterDouble(m_shooter, -0.25, -0.25));
                new JoystickButton(driverController, Button.kRightBumper.value)
                                .whileHeld(new RunIntakeDouble(m_intake, 0.70));
                new JoystickButton(driverController, Button.kLeftBumper.value)
                                .whileHeld(new RunIntakeDouble(m_intake, -0.70));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                /*
                 * // Create a voltage constraint to ensure we don't accelerate too fast
                 * var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                 * new SimpleMotorFeedforward(
                 * DrivetrainConstants.ksVolts,
                 * DrivetrainConstants.kvVoltSecondsPerMeter,
                 * DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
                 * DrivetrainConstants.kDriveKinematics,
                 * 10);
                 * 
                 * // Create config for trajectory
                 * TrajectoryConfig config = new TrajectoryConfig(
                 * AutoConstants.kMaxSpeedMetersPerSecond,
                 * AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                 * // Add kinematics to ensure max speed is actually obeyed
                 * .setKinematics(DrivetrainConstants.kDriveKinematics)
                 * // Apply the voltage constraint
                 * .addConstraint(autoVoltageConstraint);
                 * 
                 * // An example trajectory to follow. All units in meters.
                 * Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                 * // Start at the origin facing the +X direction
                 * new Pose2d(0, 0, new Rotation2d(0)),
                 * // Pass through these two interior waypoints, making an 's' curve path
                 * List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                 * // End 3 meters straight ahead of where we started, facing forward
                 * new Pose2d(3, 0, new Rotation2d(0)),
                 * // Pass config
                 * config);
                 * 
                 * RamseteCommand ramseteCommand = new RamseteCommand(
                 * exampleTrajectory,
                 * m_drivetrain::getPose,
                 * new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                 * new SimpleMotorFeedforward(
                 * DrivetrainConstants.ksVolts,
                 * DrivetrainConstants.kvVoltSecondsPerMeter,
                 * DrivetrainConstants.kaVoltSecondsSquaredPerMeter),
                 * DrivetrainConstants.kDriveKinematics,
                 * m_drivetrain::getWheelSpeeds,
                 * new PIDController(DrivetrainConstants.kPDriveVel, 0, 0),
                 * new PIDController(DrivetrainConstants.kPDriveVel, 0, 0),
                 * // RamseteCommand passes volts to the callback
                 * m_drivetrain::tankDriveVolts,
                 * m_drivetrain);
                 * 
                 * // Reset odometry to the starting pose of the trajectory.
                 * m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());
                 * 
                 * // Run path following command, then stop at the end.
                 * return ramseteCommand.andThen(() -> m_drivetrain.tankDriveVolts(0, 0));
                 */
                return m_autoCommand;
        }
}

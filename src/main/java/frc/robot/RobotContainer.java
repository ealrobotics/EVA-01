// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
        // Subsystems
        private final DrivetrainSubsystem m_drivetrain = new DrivetrainSubsystem();
        private final ShooterSubsystem m_shooter = new ShooterSubsystem();
        private final IntakeSubsystem m_intake = new IntakeSubsystem();

        // XboxController on port
        XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Camera
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
                                .whileHeld(new RunShooter(m_shooter, 1000.0));
                new JoystickButton(driverController, Button.kB.value)
                                .whileHeld(new RunShooter(m_shooter, -1000.0));
                new JoystickButton(driverController, Button.kRightBumper.value)
                                .whileHeld(new RunIntake(m_intake, 1.0));
                new JoystickButton(driverController, Button.kLeftBumper.value)
                                .whileHeld(new RunIntake(m_intake, -1.0));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return new InstantCommand();
        }
}

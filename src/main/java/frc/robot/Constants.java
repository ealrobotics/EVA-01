// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.SparkMaxAlternateEncoder;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final class CANIDConstants {
        public static final int drivebaseLeftLeadMotorID = 11;
        public static final int drivebaseLeftFollowMotorID = 12;
        public static final int drivebaseRightLeadMotorID = 13;
        public static final int drivebaseRightFollowMotorID = 14;
        public static final int intakeMotorID = 21;
        public static final int shooterBottomMotorID = 31;
        public static final int shooterTopMotorID = 32;
    }

    public static final class DrivetrainConstants {
        public static final int[] kLeftEncoderPorts = new int[] { 9, 8 };
        public static final int[] kRightEncoderPorts = new int[] { 7, 6 };
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = false;

        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                kTrackwidthMeters);

        public static final int kEncoderCPR = 360;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 8.5;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class IntakeConstants {
        public static final boolean motorInverted = false;

    }

    public static final class ShooterConstants {
        public static final SparkMaxAlternateEncoder.Type kEncType = SparkMaxAlternateEncoder.Type.kQuadrature;
        public static final int kCPR = 360;

        // State-space
        public static final int[] kBottomEncoderPorts = new int[] { 5, 4 };
        public static final int kBottomEncoderCPR = 360;
        public static final double kBottomWheelDiameterMeters = 0.1;
        public static final double kBottomEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kBottomWheelDiameterMeters * Math.PI) / (double) kBottomEncoderCPR;

        // Volts per (radian per second)
        public static final double kFlywheelKv = 0.023;

        // Volts per (radian per second squared)
        public static final double kFlywheelKa = 0.001;
    }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final class CANIDConstants {
        public static final int drivebaseLeftLeadMotorID = 13;
        public static final int drivebaseLeftFollowMotorID = 14;
        public static final int drivebaseRightLeadMotorID = 11;
        public static final int drivebaseRightFollowMotorID = 12;
        public static final int intakeMotorID = 31;
        public static final int shooterBottomMotorID = 21;
        public static final int shooterTopMotorID = 22;
    }

    public static final class IntakeConstants {
        public static final boolean motorInverted = false;
    }

    public static final class ShooterConstants {
        public static final int[] kBottomEncoderPorts = new int[] { 3, 2 };
        public static final int kBottomEncoderCPR = 2048;
        public static final double kBottomWheelDiameterMeters = 0.1;
        public static final double kBottomEncoderDistancePerPulse =
                // Assumes the encoders are directly mounted on the wheel shafts
                (kBottomWheelDiameterMeters * Math.PI) / (double) kBottomEncoderCPR;
    }
}

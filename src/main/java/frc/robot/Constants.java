/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {

        public static final int kLeftFrontMotorPort = 10;
        public static final int kLeftRearMotorPort = 11;
        public static final int kRightFrontMotorPort = 12;
        public static final int kRightRearMotorPort = 13;

        public static final boolean kGyroReversed = false;

        public static final double kTurnP = 0.8; // 0.8
        public static final double kTurnI = 0;
        public static final double kTurnD = 0.045; // 0.045
        public static final double kMinCommand = 0.07;

        public static final double kMaxTurnRateDegPerS = 120;
        public static final double kMaxTurnAccelerationDegPerSSquared = 300;

        public static final double kTurnToleranceDeg = 2;
        public static final double kTurnRateToleranceDegPerS = 8; // 10

        public static final double ksVolts = 1.41;
        public static final double kvVoltSecondsPerMeter = 3.14;
        public static final double kaVoltSecondsSquaredPerMeter = 0.326;

        public static final double kPDriveVel = 12;
        public static final double kTrackWidthMeters = 0.572;
        public static final double kMaxSpeedMetersPerSecond = 2.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2.5;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kEncoderDistancePerPulse = 4096;

        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
                DriveConstants.kTrackWidthMeters);

    }

    public static final class ShooterConstants {
        public static final int kShooterMotor1Port = 20;
        public static final int kShooterMotor2Port = 21;

        public static final int kShooterEncoderA = 0;
        public static final int kShooterEncoderB = 1;
        public static final int kShooterEncoderPPR = 2048;
        public static final boolean kShooterEncoderIsReversed = true;

        public static final double kShootP = 0.2; // 0.02;
        public static final double kShootI = 0.000;
        public static final double kShootD = 0.000;// 0.01;
    }

    public static final class HopperConstants {
        public static final int kHopperMotor1Port = 30;
        public static final int kHopperMotor2Port = 31;
    }

    public static final class IntakeConstants {
        public static final int kIntakeMotorPort = 40; // PWM Port
    }

    public static final class ClimbConstants {
        public static final int kPCMPort = 0;
        public static final int kCompressorPort = 0;
        public static final int kClimbDoubleSolenoidPort1 = 6;
        public static final int kClimbDoubleSolenoidPort2 = 7;
    }

    public static final class MiscConstants {
        public static final int kLEDRelayPort = 2;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
    }
}

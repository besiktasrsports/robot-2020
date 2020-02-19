/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.trajectories;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Add your docs here.
 */
public class SneakyTrajectory {

    public Trajectory centerRightAutoBackwards, centerRightAutoForward, onlyBackwards;
    // public RamseteCommand centerRightAutoBackwardsCommand, centerRightAutoForwardCommand, onlyBackwardsCommand;
    private DriveSubsystem m_drive;

    public SneakyTrajectory(DriveSubsystem drive) {
        m_drive = drive;

        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics, 11); // 8

        TrajectoryConfig configReversed = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);
        configReversed.setReversed(true);

        TrajectoryConfig configForward = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint);
        double divisor = 1.0;
        centerRightAutoBackwards = TrajectoryGenerator.generateTrajectory(
                new Pose2d(13 / divisor, -5.9 / divisor, new Rotation2d(0)), List.of(new Translation2d(11.5 / divisor, -7.6 / divisor)),
                new Pose2d(8 / divisor, -7.55 / divisor, new Rotation2d(0)), configReversed);
        centerRightAutoForward = TrajectoryGenerator
                .generateTrajectory(List.of(new Pose2d(8 / divisor, -7.55 / divisor, new Rotation2d(0)), new Pose2d(10.5/divisor, -5.9/divisor, new Rotation2d(0)),
                        new Pose2d(13 / divisor, -5.9 / divisor, new Rotation2d(0))), configForward);
        onlyBackwards = TrajectoryGenerator.generateTrajectory(
                            new Pose2d(3.15 / 2, -2.4 / 2, new Rotation2d(0)), List.of(new Translation2d(1.8 / 2, -2.4 / 2)),
                            new Pose2d(-3.55 / 2, -2.4 / 2, new Rotation2d(0)), configReversed);
            
        /*
        centerRightAutoBackwardsCommand = new RamseteCommand(centerRightAutoBackwards, m_drive::getPose,
                new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics, m_drive::getWheelSpeeds,
                new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
                m_drive::tankDriveVolts, m_drive);

        centerRightAutoForwardCommand = new RamseteCommand(centerRightAutoForward, m_drive::getPose,
                new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics, m_drive::getWheelSpeeds,
                new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
                m_drive::tankDriveVolts, m_drive);
       
        onlyBackwardsCommand = new RamseteCommand(onlyBackwards, m_drive::getPose,
                new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics, m_drive::getWheelSpeeds,
                new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
                m_drive::tankDriveVolts, m_drive);
        */
    }

    public RamseteCommand getRamsete(Trajectory traj)
    {
        return new RamseteCommand(traj, m_drive::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_drive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        m_drive::tankDriveVolts, m_drive); 
    }
}

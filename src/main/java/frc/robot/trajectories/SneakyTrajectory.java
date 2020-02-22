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
        public Trajectory centerRightAutoBackwards, centerRightAutoForward;
        public Trajectory centerRightAuto8Cell_1, centerRightAuto8Cell_2, centerRightAuto8Cell_3,
                        centerRightAuto8Cell_4, centerRightAuto8Cell_5, centerRightAuto8Cell_6;
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
                                List.of(new Pose2d(13 / divisor, -5.9 / divisor, new Rotation2d(0)),
                                                new Pose2d(11.5 / divisor, -7.6 / divisor, new Rotation2d(0)),
                                                new Pose2d(8 / divisor, -7.55 / divisor, new Rotation2d(0))),
                                configReversed);
                centerRightAutoForward = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(8 / divisor, -7.55 / divisor, new Rotation2d(0)),
                                                new Pose2d(10.5 / divisor, -5.9 / divisor, new Rotation2d(0)),
                                                new Pose2d(13 / divisor, -5.9 / divisor, new Rotation2d(0))),
                                configForward);

                centerRightAuto8Cell_1 = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(13 / divisor, -5.9 / divisor, new Rotation2d(0)),
                                                new Pose2d(11.5 / divisor, -7.6 / divisor, new Rotation2d(0)),
                                                new Pose2d(8 / divisor, -7.55 / divisor, new Rotation2d(0))),
                                configReversed);
                centerRightAuto8Cell_2 = TrajectoryGenerator
                                .generateTrajectory(
                                                List.of(new Pose2d(8 / divisor, -7.55 / divisor, new Rotation2d(0)),
                                                                new Pose2d(10 / divisor, -6.1 / divisor,
                                                                                new Rotation2d(-Math.toRadians(67.5)))),
                                                configForward);
                centerRightAuto8Cell_3 = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(10 / divisor, -6.1 / divisor, new Rotation2d(-Math.toRadians(67.5))),
                                                new Pose2d(9.75 / divisor, -5.5 / divisor, new Rotation2d(-Math.toRadians(67.5)))),
                                configReversed);
                centerRightAuto8Cell_4 = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(9.75 / divisor, -5.5 / divisor, new Rotation2d(-Math.toRadians(67.5))),
                                                new Pose2d(10.3 / divisor, -5.9 / divisor, new Rotation2d(-Math.toRadians(67.5)))),
                                configForward);
                centerRightAuto8Cell_5 = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(10.3 / divisor, -5.9 / divisor, new Rotation2d(-Math.toRadians(67.5))),
                                                new Pose2d(10.1 / divisor, -5.4 / divisor, new Rotation2d(-Math.toRadians(67.5)))),
                                configReversed);
                centerRightAuto8Cell_6 = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(10.1 / divisor, -5.4 / divisor, new Rotation2d(-Math.toRadians(67.5))),
                                                new Pose2d(11 / divisor, -5.9 / divisor, new Rotation2d(0)),
                                                new Pose2d(13 / divisor, -5.9 / divisor, new Rotation2d(0))),
                                configForward);
        }

        public RamseteCommand getRamsete(Trajectory traj) {
                return new RamseteCommand(traj, m_drive::getPose,
                                new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
                                new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
                                                DriveConstants.kaVoltSecondsSquaredPerMeter),
                                DriveConstants.kDriveKinematics, m_drive::getWheelSpeeds,
                                new PIDController(DriveConstants.kPDriveVel, 0, 0),
                                new PIDController(DriveConstants.kPDriveVel, 0, 0), m_drive::tankDriveVolts, m_drive);
        }
}

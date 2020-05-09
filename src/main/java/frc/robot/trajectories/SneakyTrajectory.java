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
        public Trajectory centerRightAuto[];
        public Trajectory rightAuto[];
        public Trajectory leftAuto5Cell[];
        public Trajectory leftAuto6Cell[];
        public Trajectory centerRightAuto8Cell[];
        public Trajectory rightAuto8Cell[];
        private DriveSubsystem m_drive;
        private static double bottomRectAngle = -Math.toRadians(67.5);
        private static double topRectAngle = Math.toRadians(22.5);

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
                centerRightAuto[0] = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(13 / divisor, -5.8 / divisor, new Rotation2d(0)),
                                                new Pose2d(11.5 / divisor, -7.0 / divisor, new Rotation2d(0)),
                                                new Pose2d(8 / divisor, -7.0 / divisor, new Rotation2d(0))),
                                configReversed);
                centerRightAuto[1] = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(8 / divisor, -7.05 / divisor, new Rotation2d(0)),
                                                new Pose2d(10.5 / divisor, -5.80 / divisor, new Rotation2d(0)),
                                                new Pose2d(13 / divisor, -5.80 / divisor, new Rotation2d(0))),
                                configForward);
                rightAuto[0] = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(13 / divisor, -7 / divisor, new Rotation2d(0)),
                                                new Pose2d(11.5 / divisor, -7.50 / divisor, new Rotation2d(0)),
                                                new Pose2d(8 / divisor, -7.40 / divisor, new Rotation2d(0))),
                                configReversed);
                rightAuto[1] = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(8 / divisor, -7.55 / divisor, new Rotation2d(0)),
                                                new Pose2d(10.5 / divisor, -5.75 / divisor, new Rotation2d(0)),
                                                new Pose2d(13 / divisor, -5.75 / divisor, new Rotation2d(0))),
                                configForward);

                centerRightAuto8Cell[0] = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(13 / divisor, -5.8 / divisor, new Rotation2d(0)),
                                                new Pose2d(11.5 / divisor, -7.6 / divisor, new Rotation2d(0)),
                                                new Pose2d(8 / divisor, -7.55 / divisor, new Rotation2d(0))),
                                configReversed);
                centerRightAuto8Cell[1] = TrajectoryGenerator
                                .generateTrajectory(
                                                List.of(new Pose2d(8 / divisor, -7.55 / divisor, new Rotation2d(0)),
                                                                new Pose2d(10 / divisor, -6.2 / divisor,
                                                                                new Rotation2d(bottomRectAngle))),
                                                configForward);
                centerRightAuto8Cell[2] = TrajectoryGenerator.generateTrajectory(List.of(
                                new Pose2d(10 / divisor, -6.2 / divisor, new Rotation2d(bottomRectAngle)),
                                new Pose2d(9.8 / divisor, -5.65 / divisor, new Rotation2d(bottomRectAngle))),
                                configReversed);
                centerRightAuto8Cell[3] = TrajectoryGenerator.generateTrajectory(List.of(
                                new Pose2d(9.8 / divisor, -6.2 / divisor, new Rotation2d(bottomRectAngle)),
                                new Pose2d(10.45 / divisor, -6.1 / divisor, new Rotation2d(bottomRectAngle))),
                                configForward);
                centerRightAuto8Cell[4] = TrajectoryGenerator.generateTrajectory(List.of(
                                new Pose2d(10.45 / divisor, -6.1 / divisor, new Rotation2d(bottomRectAngle)),
                                new Pose2d(10.1 / divisor, -5.4 / divisor, new Rotation2d(bottomRectAngle))),
                                configReversed);
                centerRightAuto8Cell[5] = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(10.1 / divisor, -5.4 / divisor, new Rotation2d(bottomRectAngle)),
                                                new Pose2d(11 / divisor, -5.8 / divisor, new Rotation2d(0))
                                ), configForward);
                rightAuto8Cell[0] = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(13 / divisor, -7 / divisor, new Rotation2d(0)),
                                                new Pose2d(11.5 / divisor, -7.6 / divisor, new Rotation2d(0)),
                                                new Pose2d(8 / divisor, -7.55 / divisor, new Rotation2d(0))),
                                configReversed);
                rightAuto8Cell[1] = TrajectoryGenerator
                                .generateTrajectory(
                                                List.of(new Pose2d(8 / divisor, -7.55 / divisor, new Rotation2d(0)),
                                                                new Pose2d(10 / divisor, -6.1 / divisor,
                                                                                new Rotation2d(bottomRectAngle))),
                                                configForward);
                rightAuto8Cell[2] = TrajectoryGenerator.generateTrajectory(List.of(
                                new Pose2d(10 / divisor, -6.1 / divisor, new Rotation2d(bottomRectAngle)),
                                new Pose2d(9.75 / divisor, -5.5 / divisor, new Rotation2d(bottomRectAngle))),
                                configReversed);
                rightAuto8Cell[3] = TrajectoryGenerator.generateTrajectory(List.of(
                                new Pose2d(9.75 / divisor, -5.5 / divisor, new Rotation2d(bottomRectAngle)),
                                new Pose2d(10.3 / divisor, -5.75 / divisor, new Rotation2d(bottomRectAngle))),
                                configForward);
                rightAuto8Cell[4] = TrajectoryGenerator.generateTrajectory(List.of(
                                new Pose2d(10.3 / divisor, -5.75 / divisor, new Rotation2d(bottomRectAngle)),
                                new Pose2d(10.1 / divisor, -5.4 / divisor, new Rotation2d(bottomRectAngle))),
                                configReversed);
                rightAuto8Cell[5] = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(10.1 / divisor, -5.4 / divisor, new Rotation2d(bottomRectAngle)),
                                                new Pose2d(11 / divisor, -5.75 / divisor, new Rotation2d(0))
                                ), configForward);
                
                leftAuto5Cell[0] = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(13 / divisor, -4.25 / divisor, new Rotation2d(0)),
                                                new Pose2d(10.7 / divisor, -0.48 / divisor, new Rotation2d(0)),
                                                new Pose2d(9.7 / divisor, -0.48 / divisor, new Rotation2d(0))),
                                configReversed);
                leftAuto5Cell[1] = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(9.7 / divisor, -0.48 / divisor, new Rotation2d(0)),
                                                new Pose2d(10.7 / divisor, -0.95 / divisor, new Rotation2d(0))),
                                configForward);
                leftAuto5Cell[2] = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(10.7 / divisor, -0.95 / divisor, new Rotation2d(0)),
                                                new Pose2d(9.7 / divisor, -0.95 / divisor, new Rotation2d(0))),
                                configReversed);
                leftAuto5Cell[3] = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(9.7 / divisor, -0.95 / divisor, new Rotation2d(0)),
                                                new Pose2d(11.4 / divisor, -5.15 / divisor, new Rotation2d(0))),
                                configForward);
                

                leftAuto5Cell[0] = TrajectoryGenerator.generateTrajectory(
                        List.of(new Pose2d(13 / divisor, -5.10 / divisor, new Rotation2d(0)),
                                        new Pose2d(10.7 / divisor, -4.30 / divisor, new Rotation2d(topRectAngle))),
                        configReversed);
        
                leftAuto5Cell[1] = TrajectoryGenerator.generateTrajectory(
                        List.of(new Pose2d(10.7 / divisor, -4.30 / divisor, new Rotation2d(topRectAngle)),
                                        new Pose2d(11.7 / divisor, -3.95 / divisor, new Rotation2d(Math.toRadians(-112.5)))),
                        configForward);                

                leftAuto5Cell[2] = TrajectoryGenerator.generateTrajectory(
                        List.of(new Pose2d(11.7 / divisor, -3.95 / divisor, new Rotation2d(topRectAngle)),
                                        new Pose2d(10.25 / divisor, -3.75 / divisor, new Rotation2d(topRectAngle))),
                        configReversed);       
                leftAuto5Cell[3] = TrajectoryGenerator.generateTrajectory(
                        List.of(new Pose2d(10.25 / divisor, -3.75 / divisor, new Rotation2d(0)),
                                        new Pose2d(13 / divisor, -5.80 / divisor, new Rotation2d(0))), // 5.80
                        configForward);    

                leftAuto6Cell[0] = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(13 / divisor, -4.5 / divisor, new Rotation2d(0)),
                                new Pose2d(10.3 / divisor, -4.5 / divisor, new Rotation2d(topRectAngle))),
                configReversed);
                leftAuto6Cell[1] = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(10.3 / divisor, -4.5 / divisor, new Rotation2d(topRectAngle)),
                                                new Pose2d(11.1 / divisor, -3.8 / divisor, new Rotation2d(topRectAngle))),
                                configForward);
                leftAuto6Cell[2] = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(11.1 / divisor, -3.8 / divisor, new Rotation2d(topRectAngle)),
                                                new Pose2d(10.06 / divisor, -4.15 / divisor, new Rotation2d(topRectAngle))),
                                configReversed);
                leftAuto6Cell[3] = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(10.06 / divisor, -4.15 / divisor, new Rotation2d(topRectAngle)),
                                                new Pose2d(10.65 / divisor, -3.6 / divisor, new Rotation2d(topRectAngle))),
                                configForward);
                leftAuto6Cell[4] = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(10.65 / divisor, -3.6 / divisor, new Rotation2d(topRectAngle)),
                                                new Pose2d(10 / divisor, -3.7 / divisor, new Rotation2d(topRectAngle))),
                                configReversed);
                leftAuto6Cell[5] = TrajectoryGenerator.generateTrajectory(
                                List.of(new Pose2d(10 / divisor, -3.7 / divisor, new Rotation2d(topRectAngle)),
                                                new Pose2d(11.6 / divisor, -4.2 / divisor, new Rotation2d(-Math.toRadians(20)))),
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
